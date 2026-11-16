use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};
use std::io::Cursor;
use std::sync::Arc;
#[cfg(any(not(target_arch = "wasm32"), feature = "wasm-threads"))]
use std::sync::{Mutex, mpsc};
#[cfg(not(target_arch = "wasm32"))]
use std::thread::{self, JoinHandle};
use web_time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, ensure};
use ccl_calib::camera::inertial::ImuSample;
use ccl_calib::camera::{CalibrationImageSize, SampleTime};
use ccl_image::{Gray8View, detection};
use image::ImageDecoder;
use ccl_rosbag::schema::Projection;
use ccl_rosbag::{
    BagReader, CompiledSchema, ImageAdapter, MessageFilter, MessageSchema,
    SchemaEncoding, SchemaSource, StaticSchemaRegistry, TopicId, ValueRef, Visit,
};
use serde::{Deserialize, Serialize};
#[cfg(not(target_arch = "wasm32"))]
use ccl_rosbag::OpenOptions;

use crate::config::{
    BagSourceConfig, CalibrationConfig, CameraConfig, CameraTimestampReference, RunSettings,
};
use crate::detection::{
    DetectedDataset, DetectedFrame, DetectedView, DetectionStats, DetectionSummary, FrameDetection,
    FrameDetector,
};

mod performance;
pub use performance::PipelinePerformance;
const RAW_IMAGE_TYPE: &str = "sensor_msgs/Image";
const COMPRESSED_IMAGE_TYPE: &str = "sensor_msgs/CompressedImage";
const STRING_TYPE: &str = "std_msgs/String";
const IMU_TYPE: &str = "sensor_msgs/Imu";

const ROS2_IMAGE_SCHEMA: &str = "std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\nuint8 is_bigendian\nuint32 step\nuint8[] data\nMSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\nMSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n";
const ROS2_COMPRESSED_IMAGE_SCHEMA: &str = "std_msgs/Header header\nstring format\nuint8[] data\nMSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\nMSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n";
const ROS2_IMU_SCHEMA: &str = "std_msgs/Header header\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance\ngeometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance\ngeometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance\nMSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\nMSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\nMSG: geometry_msgs/Quaternion\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\nMSG: geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z\n";

fn standard_schema_registry() -> StaticSchemaRegistry {
    let mut registry = StaticSchemaRegistry::default();
    for (type_name, definition) in [
        ("sensor_msgs/msg/Image", ROS2_IMAGE_SCHEMA),
        (
            "sensor_msgs/msg/CompressedImage",
            ROS2_COMPRESSED_IMAGE_SCHEMA,
        ),
        ("sensor_msgs/msg/Imu", ROS2_IMU_SCHEMA),
    ] {
        registry.insert(MessageSchema {
            type_name: type_name.to_owned(),
            encoding: SchemaEncoding::Ros2Msg,
            definition: definition.to_owned(),
            fingerprint: None,
            source: SchemaSource::ExternalRegistry,
        });
    }
    registry
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TopicInspection {
    pub name: String,
    pub type_name: String,
    pub message_count: u64,
    pub first_record_time_ns: Option<i64>,
    pub last_record_time_ns: Option<i64>,
    pub header_time_valid: bool,
    pub status: String,
}

impl TopicInspection {
    #[must_use]
    pub fn rate_hz(&self) -> Option<f64> {
        let span = self
            .last_record_time_ns?
            .checked_sub(self.first_record_time_ns?)?;
        let count = u32::try_from(self.message_count.checked_sub(1)?).ok()?;
        let duration = Duration::from_nanos(u64::try_from(span).ok()?);
        (!duration.is_zero()).then(|| f64::from(count) / duration.as_secs_f64())
    }
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BagInspection {
    pub source: std::path::PathBuf,
    pub topics: Vec<TopicInspection>,
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
pub struct InputSummary {
    pub schema_version: u32,
    pub source_type: &'static str,
    pub bag_path: std::path::PathBuf,
    /// Anchor-camera messages that define the approximate shared-view timeline (including a
    /// final anchor frame without a partner in another camera). Exact-time union views used
    /// by continuous time can be more numerous when cameras are asynchronous.
    pub view_count: usize,
    pub frame_count_by_camera: BTreeMap<String, usize>,
    pub image_size_by_camera: BTreeMap<String, ImageSizeSummary>,
    pub first_timestamp_ns: Option<i64>,
    pub last_timestamp_ns: Option<i64>,
    pub imu_sample_count: usize,
    pub imu_first_timestamp_ns: Option<i64>,
    pub imu_last_timestamp_ns: Option<i64>,
    pub performance: PipelinePerformance,
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DetectionPreview {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
    pub corners: Vec<[f64; 2]>,
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(deny_unknown_fields)]
pub struct LatestImu {
    pub sample_count: usize,
    pub timestamp_ns: i64,
    pub gyro_rad_s: [f64; 3],
    pub specific_force_m_s2: [f64; 3],
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
pub struct DetectionProgress {
    pub timestamp_ns: i64,
    pub camera_id: String,
    pub frames_processed: usize,
    pub tags: usize,
    pub points: usize,
    pub frames_per_second: f64,
    pub average_frames_per_second: f64,
    pub workers: usize,
    /// Sum of worker decode/grayscale time; overlaps across parallel workers.
    pub decode_grayscale_worker_ms: f64,
    /// Sum of worker detection time; not additive to pipeline wall time.
    pub detection_worker_ms: f64,
    /// Browser-only sampled grayscale frame; absent from unsampled and native progress.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preview: Option<DetectionPreview>,
    /// Latest real IMU sample observed by the input visitor when this frame completed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub latest_imu: Option<LatestImu>,
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ImageSizeSummary {
    pub width: u32,
    pub height: u32,
}

struct TopicStats {
    count: u64,
    first: Option<i64>,
    last: Option<i64>,
    header_valid: bool,
    status: String,
}

fn open_bag(source: &BagSourceConfig) -> Result<Box<dyn BagReader>> {
    #[cfg(not(target_arch = "wasm32"))]
    let bag = ccl_rosbag::open(&source.path, OpenOptions::new(source.format.core()));
    #[cfg(target_arch = "wasm32")]
    let bag = crate::browser::open_bag(source);
    bag.with_context(|| format!("open bag {}", source.path.display()))
}

pub fn inspect(source: &BagSourceConfig) -> Result<BagInspection> {
    let bag = open_bag(source)?;
    let registry = standard_schema_registry();
    let candidates = bag
        .topics()
        .iter()
        .filter(|topic| {
            let name = normalize_type(&topic.type_name);
            name == RAW_IMAGE_TYPE || name == COMPRESSED_IMAGE_TYPE || name == IMU_TYPE
        })
        .collect::<Vec<_>>();
    let schemas = candidates
        .iter()
        .map(|topic| CompiledSchema::for_topic(topic, Some(&registry)))
        .collect::<Vec<_>>();
    let stamps = schemas
        .iter()
        .map(|schema| {
            schema
                .as_ref()
                .ok()
                .and_then(|schema| TimestampBinding::bind(schema).ok())
        })
        .collect::<Vec<_>>();
    let index = candidates
        .iter()
        .enumerate()
        .map(|(index, topic)| (topic.id, index))
        .collect::<HashMap<_, _>>();
    let mut stats = schemas
        .iter()
        .enumerate()
        .map(|(index, schema)| TopicStats {
            count: 0,
            first: None,
            last: None,
            header_valid: schema.is_ok() && stamps[index].is_some(),
            status: schema
                .as_ref()
                .err()
                .map_or_else(|| "ok".to_owned(), ToString::to_string),
        })
        .collect::<Vec<_>>();
    let filter = MessageFilter {
        topics: candidates.iter().map(|topic| topic.id).collect(),
        ..MessageFilter::default()
    };
    bag.visit(&filter, &mut |raw| {
        let Some(&candidate_index) = index.get(&raw.topic_id) else {
            return Ok(Visit::Continue);
        };
        let stat = &mut stats[candidate_index];
        stat.count += 1;
        stat.first.get_or_insert(raw.record_time_ns);
        stat.last = Some(raw.record_time_ns);
        if let (Ok(schema), Some(stamp)) = (&schemas[candidate_index], &stamps[candidate_index]) {
            match schema
                .view(raw.payload)
                .and_then(|message| stamp.read(&message))
            {
                Ok(_) => {}
                Err(error) => {
                    stat.header_valid = false;
                    stat.status = error.to_string();
                }
            }
        } else {
            stat.header_valid = false;
        }
        Ok(Visit::Continue)
    })?;
    let topics = candidates
        .into_iter()
        .zip(stats)
        .map(|(topic, stat)| TopicInspection {
            name: topic.name.clone(),
            type_name: normalize_type(&topic.type_name).to_owned(),
            message_count: stat.count,
            first_record_time_ns: stat.first,
            last_record_time_ns: stat.last,
            header_time_valid: stat.header_valid,
            status: stat.status,
        })
        .collect();
    Ok(BagInspection {
        source: source.path.clone(),
        topics,
    })
}

struct RawDetectedFrame {
    timestamp_ns: i64,
    ordinal: u64,
    detected: Option<DetectedFrame>,
}

type TimestampedDetectedFrame = (i64, Option<Arc<DetectedFrame>>);
type CameraDetectedFrames = Vec<TimestampedDetectedFrame>;

struct RawImuSample {
    timestamp_ns: i64,
    gyro_rad_s: [f64; 3],
    specific_force_m_s2: [f64; 3],
}

#[derive(Deserialize)]
struct FrameExposureMetadata {
    camera_timestamp_ns: i64,
    exposure: ExposureMetadata,
}

#[derive(Deserialize)]
struct ExposureMetadata {
    duration_ns: u64,
}

enum DetectionImage {
    Gray8 {
        width: u32,
        height: u32,
        pixels: Vec<u8>,
    },
    Compressed(Vec<u8>),
}

impl DetectionImage {
    fn into_buffer(self) -> Vec<u8> {
        match self {
            Self::Gray8 { pixels, .. } | Self::Compressed(pixels) => pixels,
        }
    }
}

struct DetectionTask {
    camera_index: usize,
    timestamp_ns: i64,
    ordinal: u64,
    context: String,
    image: DetectionImage,
    preview_requested: bool,
}

struct DetectionCompletion {
    camera_index: usize,
    timestamp_ns: i64,
    ordinal: u64,
    result: Result<(
        CalibrationImageSize,
        FrameDetection,
        Option<DetectionPreview>,
    )>,
    decode_grayscale: Duration,
    detection: Duration,
    input_buffer: Vec<u8>,
    worker_wait: Duration,
    completed_at: Instant,
    preview_time: Duration,
    core: ccl_image::apriltag::AprilTagWorkspaceStats,
    retained_bytes: usize,
}

#[derive(Default)]
struct DetectionBuffers {
    decoded: Vec<u8>,
    gray: Vec<u8>,
}

struct InputBufferCache {
    buffers: RefCell<Vec<Vec<u8>>>,
    max_cached: usize,
}

impl InputBufferCache {
    fn new(max_cached: usize) -> Self {
        Self {
            buffers: RefCell::new(Vec::with_capacity(max_cached)),
            max_cached,
        }
    }

    fn take(&self, required_capacity: usize) -> Vec<u8> {
        let mut buffers = self.buffers.borrow_mut();
        let index = buffers
            .iter()
            .rposition(|buffer| buffer.capacity() >= required_capacity)
            .or_else(|| {
                buffers
                    .iter()
                    .enumerate()
                    .max_by_key(|(_, buffer)| buffer.capacity())
                    .map(|(index, _)| index)
            });
        let mut buffer = index.map_or_else(Vec::new, |index| buffers.swap_remove(index));
        drop(buffers);
        buffer.clear();
        if buffer.capacity() < required_capacity {
            buffer.reserve(required_capacity);
        }
        buffer
    }

    fn recycle(&self, mut buffer: Vec<u8>) {
        buffer.clear();
        let mut buffers = self.buffers.borrow_mut();
        if buffers.len() < self.max_cached {
            buffers.push(buffer);
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
type DetectionWorker = JoinHandle<()>;

#[cfg(all(target_arch = "wasm32", feature = "wasm-threads"))]
type DetectionWorker = mpsc::Receiver<()>;

#[cfg(any(not(target_arch = "wasm32"), feature = "wasm-threads"))]
fn spawn_detection_worker(task: impl FnOnce() + Send + 'static) -> DetectionWorker {
    #[cfg(not(target_arch = "wasm32"))]
    { thread::spawn(task) }
    #[cfg(all(target_arch = "wasm32", feature = "wasm-threads"))]
    {
        let (sender, receiver) = mpsc::channel();
        rayon::spawn(move || { task(); let _ = sender.send(()); });
        receiver
    }
}

#[cfg(any(not(target_arch = "wasm32"), feature = "wasm-threads"))]
struct DetectionPool {
    sender: Option<mpsc::SyncSender<DetectionTask>>,
    receiver: mpsc::Receiver<DetectionCompletion>,
    handles: Vec<DetectionWorker>,
    input_buffers: InputBufferCache,
    workers: usize,
}

#[cfg(any(not(target_arch = "wasm32"), feature = "wasm-threads"))]
impl DetectionPool {
    fn new(detector: &FrameDetector) -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        let workers = thread::available_parallelism().map_or(1, std::num::NonZero::get);
        #[cfg(all(target_arch = "wasm32", feature = "wasm-threads"))]
        let workers = rayon::current_num_threads();
        let (task_sender, task_receiver) = mpsc::sync_channel(workers.saturating_mul(2));
        let (result_sender, result_receiver) = mpsc::channel();
        let task_receiver = Arc::new(Mutex::new(task_receiver));
        let handles = (0..workers)
            .map(|_| {
                let tasks = Arc::clone(&task_receiver);
                let results = result_sender.clone();
                let detector = detector.clone();
                spawn_detection_worker(move || {
                    let mut buffers = DetectionBuffers::default();
                    let mut workspace = ccl_image::apriltag::AprilTagWorkspace::default();
                    loop {
                        let waiting = Instant::now();
                        let task = {
                            let receiver = tasks
                                .lock()
                                .unwrap_or_else(std::sync::PoisonError::into_inner);
                            receiver.recv()
                        };
                        let Ok(task) = task else { break };
                        let waited = waiting.elapsed();
                        let mut completion = process_detection_task(&detector, task, &mut buffers, &mut workspace);
                        completion.worker_wait = waited;
                        if results.send(completion).is_err() {
                            break;
                        }
                    }
                })
            })
            .collect();
        drop(result_sender);
        Self {
            sender: Some(task_sender),
            receiver: result_receiver,
            handles,
            input_buffers: InputBufferCache::new(workers.saturating_mul(3)),
            workers,
        }
    }

    fn copy_input(&self, bytes: &[u8]) -> Vec<u8> {
        let mut buffer = self.input_buffers.take(bytes.len());
        buffer.extend_from_slice(bytes);
        buffer
    }

    fn recycle_completion(&self, completion: &mut DetectionCompletion) {
        self.input_buffers
            .recycle(std::mem::take(&mut completion.input_buffer));
    }

    fn submit(&self, task: DetectionTask) -> Result<()> {
        let sender = self.sender.as_ref().context("detection pool is closed")?;
        sender.send(task).map_err(|error| {
            self.input_buffers.recycle(error.0.image.into_buffer());
            anyhow!("detection worker pool stopped")
        })
    }

    fn try_recv(&self) -> Option<DetectionCompletion> {
        self.receiver.try_recv().ok()
    }

    fn finish(&mut self) -> Result<Vec<DetectionCompletion>> {
        self.sender.take();
        let mut worker_error = None;
        for handle in self.handles.drain(..) {
            #[cfg(not(target_arch = "wasm32"))]
            let stopped = handle.join().is_err();
            #[cfg(all(target_arch = "wasm32", feature = "wasm-threads"))]
            let stopped = handle.recv().is_err();
            if stopped && worker_error.is_none() {
                worker_error = Some(anyhow!("detection worker stopped unexpectedly"));
            }
        }
        if let Some(error) = worker_error {
            return Err(error);
        }
        Ok(self.receiver.try_iter().collect())
    }
}

#[cfg(all(target_arch = "wasm32", not(feature = "wasm-threads")))]
struct DetectionPool {
    detector: FrameDetector,
    completion: RefCell<Option<DetectionCompletion>>,
    buffers: RefCell<DetectionBuffers>,
    workspace: RefCell<ccl_image::apriltag::AprilTagWorkspace>,
    input_buffers: InputBufferCache,
    workers: usize,
}

#[cfg(all(target_arch = "wasm32", not(feature = "wasm-threads")))]
impl DetectionPool {
    fn new(detector: &FrameDetector) -> Self {
        Self {
            detector: detector.clone(),
            completion: RefCell::new(None),
            buffers: RefCell::new(DetectionBuffers::default()),
            workspace: RefCell::new(ccl_image::apriltag::AprilTagWorkspace::default()),
            input_buffers: InputBufferCache::new(3),
            workers: 1,
        }
    }

    fn copy_input(&self, bytes: &[u8]) -> Vec<u8> {
        let mut buffer = self.input_buffers.take(bytes.len());
        buffer.extend_from_slice(bytes);
        buffer
    }

    fn recycle_completion(&self, completion: &mut DetectionCompletion) {
        self.input_buffers
            .recycle(std::mem::take(&mut completion.input_buffer));
    }

    fn submit(&self, task: DetectionTask) -> Result<()> {
        if self.completion.borrow().is_some() {
            self.input_buffers.recycle(task.image.into_buffer());
            return Err(anyhow!("previous detection result was not consumed"));
        }
        let completion = process_detection_task(
            &self.detector,
            task,
            &mut self.buffers.borrow_mut(),
            &mut self.workspace.borrow_mut(),
        );
        *self.completion.borrow_mut() = Some(completion);
        Ok(())
    }

    fn try_recv(&self) -> Option<DetectionCompletion> {
        self.completion.borrow_mut().take()
    }

    fn finish(&mut self) -> Result<Vec<DetectionCompletion>> {
        Ok(self.completion.borrow_mut().take().into_iter().collect())
    }
}

fn process_detection_task(
    detector: &FrameDetector,
    task: DetectionTask,
    buffers: &mut DetectionBuffers,
    workspace: &mut ccl_image::apriltag::AprilTagWorkspace,
) -> DetectionCompletion {
    let DetectionTask {
        camera_index,
        timestamp_ns,
        ordinal,
        context,
        mut image,
        preview_requested,
    } = task;
    let mut decode_grayscale = Duration::ZERO;
    let mut detection = Duration::ZERO;
    let mut preview_time = Duration::ZERO;
    let result = (|| {
        let decode_started = Instant::now();
        let (width, height, pixels) = match &mut image {
            DetectionImage::Gray8 {
                width,
                height,
                pixels,
            } => (*width, *height, pixels.as_slice()),
            DetectionImage::Compressed(bytes) => {
                let decoded = decode_compressed_gray(bytes, buffers)
                    .with_context(|| format!("decode compressed image {context}"))?;
                (decoded.width, decoded.height, decoded.pixels)
            }
        };
        decode_grayscale = decode_started.elapsed();
        let size = CalibrationImageSize::new(width, height)?;
        let view = Gray8View::new(width, height, width as usize, pixels)?;
        let detection_started = Instant::now();
        let detected = detector.detect(
            camera_index,
            size,
            detection::ImageView::Gray8(view),
            &context,
            workspace,
        )?;
        detection = detection_started.elapsed();
        let preview_started = Instant::now();
        #[cfg(target_arch = "wasm32")]
        let preview = preview_requested
            .then(|| detection_preview(width, height, pixels, detected.frame.as_ref()));
        #[cfg(not(target_arch = "wasm32"))]
        let _ = preview_requested;
        #[cfg(not(target_arch = "wasm32"))]
        let preview: Option<DetectionPreview> = None;
        preview_time = preview_started.elapsed();
        Ok((size, detected, preview))
    })();
    DetectionCompletion {
        camera_index,
        timestamp_ns,
        ordinal,
        result,
        decode_grayscale,
        detection,
        input_buffer: image.into_buffer(),
        worker_wait: Duration::ZERO,
        completed_at: Instant::now(),
        preview_time,
        core: workspace.stats(),
        retained_bytes: workspace.stats().retained_bytes + buffers.decoded.capacity() + buffers.gray.capacity(),
    }
}

struct DecodedGray<'a> {
    width: u32,
    height: u32,
    pixels: &'a [u8],
}

fn decode_compressed_gray<'a>(
    bytes: &[u8],
    buffers: &'a mut DetectionBuffers,
) -> Result<DecodedGray<'a>> {
    let reader = image::ImageReader::new(Cursor::new(bytes)).with_guessed_format()?;
    let mut decoder = reader.into_decoder()?;
    let (width, height) = decoder.dimensions();
    let color = decoder.color_type();
    let total_bytes = decoder.total_bytes();
    let mut limits = image::Limits::default();
    limits.reserve(total_bytes)?;
    decoder.set_limits(limits)?;
    let total_bytes = usize::try_from(total_bytes)
        .context("decoded image is too large for this platform")?;

    let DetectionBuffers { decoded, gray } = buffers;
    decoded.resize(total_bytes, 0);
    decoder.read_image(decoded)?;
    if color == image::ColorType::L8 {
        return Ok(DecodedGray {
            width,
            height,
            pixels: decoded,
        });
    }
    decoded_pixels_to_gray(color, decoded, gray)?;
    Ok(DecodedGray {
        width,
        height,
        pixels: gray,
    })
}

fn decoded_pixels_to_gray(
    color: image::ColorType,
    decoded: &[u8],
    gray: &mut Vec<u8>,
) -> Result<()> {
    let bytes_per_pixel = usize::from(color.bytes_per_pixel());
    ensure!(
        decoded.len().is_multiple_of(bytes_per_pixel),
        "decoded image byte count does not match {color:?}"
    );
    let pixel_count = decoded.len() / bytes_per_pixel;
    gray.resize(pixel_count, 0);
    match color {
        image::ColorType::L8 => gray.copy_from_slice(decoded),
        image::ColorType::La8 => {
            for (source, output) in decoded.chunks_exact(2).zip(gray.iter_mut()) {
                *output = source[0];
            }
        }
        image::ColorType::Rgb8 => {
            for (source, output) in decoded.chunks_exact(3).zip(gray.iter_mut()) {
                *output = image_luma8(
                    f32::from(source[0]) * (1.0 / 255.0),
                    f32::from(source[1]) * (1.0 / 255.0),
                    f32::from(source[2]) * (1.0 / 255.0),
                );
            }
        }
        image::ColorType::Rgba8 => {
            for (source, output) in decoded.chunks_exact(4).zip(gray.iter_mut()) {
                *output = image_luma8(
                    f32::from(source[0]) * (1.0 / 255.0),
                    f32::from(source[1]) * (1.0 / 255.0),
                    f32::from(source[2]) * (1.0 / 255.0),
                );
            }
        }
        image::ColorType::L16 => {
            for (source, output) in decoded.chunks_exact(2).zip(gray.iter_mut()) {
                *output = narrow_luma16(u16::from_ne_bytes([source[0], source[1]]));
            }
        }
        image::ColorType::La16 => {
            for (source, output) in decoded.chunks_exact(4).zip(gray.iter_mut()) {
                *output = narrow_luma16(u16::from_ne_bytes([source[0], source[1]]));
            }
        }
        image::ColorType::Rgb16 => {
            for (source, output) in decoded.chunks_exact(6).zip(gray.iter_mut()) {
                *output = image_luma8(
                    f32::from(u16::from_ne_bytes([source[0], source[1]])) * (1.0 / 65535.0),
                    f32::from(u16::from_ne_bytes([source[2], source[3]])) * (1.0 / 65535.0),
                    f32::from(u16::from_ne_bytes([source[4], source[5]])) * (1.0 / 65535.0),
                );
            }
        }
        image::ColorType::Rgba16 => {
            for (source, output) in decoded.chunks_exact(8).zip(gray.iter_mut()) {
                *output = image_luma8(
                    f32::from(u16::from_ne_bytes([source[0], source[1]])) * (1.0 / 65535.0),
                    f32::from(u16::from_ne_bytes([source[2], source[3]])) * (1.0 / 65535.0),
                    f32::from(u16::from_ne_bytes([source[4], source[5]])) * (1.0 / 65535.0),
                );
            }
        }
        image::ColorType::Rgb32F => {
            for (source, output) in decoded.chunks_exact(12).zip(gray.iter_mut()) {
                *output = image_luma8(
                    f32::from_ne_bytes(source[0..4].try_into().expect("four-byte red")),
                    f32::from_ne_bytes(source[4..8].try_into().expect("four-byte green")),
                    f32::from_ne_bytes(source[8..12].try_into().expect("four-byte blue")),
                );
            }
        }
        image::ColorType::Rgba32F => {
            for (source, output) in decoded.chunks_exact(16).zip(gray.iter_mut()) {
                *output = image_luma8(
                    f32::from_ne_bytes(source[0..4].try_into().expect("four-byte red")),
                    f32::from_ne_bytes(source[4..8].try_into().expect("four-byte green")),
                    f32::from_ne_bytes(source[8..12].try_into().expect("four-byte blue")),
                );
            }
        }
        _ => return Err(anyhow!("unsupported decoded color type {color:?}")),
    }
    Ok(())
}

fn narrow_luma16(value: u16) -> u8 {
    u8::try_from((u32::from(value) + 128) / 257).expect("normalized luma16 is bounded")
}

fn image_luma8(red: f32, green: f32, blue: f32) -> u8 {
    // `DynamicImage::into_luma8` derives this BT.709 row from image's sRGB primaries.
    // Keep the same f32 operation order and target-dependent FMA choice as image 0.25.
    const COEFFICIENTS: [f32; 3] = [0.212_647_84, 0.715_179_2, 0.072_173_04];
    let mut luma = 0.0;
    luma = image_multiply_accumulate(luma, red, COEFFICIENTS[0]);
    luma = image_multiply_accumulate(luma, green, COEFFICIENTS[1]);
    luma = image_multiply_accumulate(luma, blue, COEFFICIENTS[2]);
    (luma * 255.0).round() as u8
}

#[cfg(any(
    all(any(target_arch = "x86", target_arch = "x86_64"), target_feature = "fma"),
    all(target_arch = "aarch64", target_feature = "neon")
))]
fn image_multiply_accumulate(accumulator: f32, value: f32, coefficient: f32) -> f32 {
    value.mul_add(coefficient, accumulator)
}

#[cfg(not(any(
    all(any(target_arch = "x86", target_arch = "x86_64"), target_feature = "fma"),
    all(target_arch = "aarch64", target_feature = "neon")
)))]
fn image_multiply_accumulate(accumulator: f32, value: f32, coefficient: f32) -> f32 {
    accumulator + value * coefficient
}

#[cfg(target_arch = "wasm32")]
fn detection_preview(
    source_width: u32,
    source_height: u32,
    source_pixels: &[u8],
    detected: Option<&DetectedFrame>,
) -> DetectionPreview {
    const MAX_WIDTH: u32 = 320;
    const MAX_HEIGHT: u32 = 240;
    let (width, height) = if source_width <= MAX_WIDTH && source_height <= MAX_HEIGHT {
        (source_width, source_height)
    } else if u64::from(source_width) * u64::from(MAX_HEIGHT)
        >= u64::from(source_height) * u64::from(MAX_WIDTH)
    {
        (
            MAX_WIDTH,
            u32::try_from(
                (u64::from(source_height) * u64::from(MAX_WIDTH))
                    / u64::from(source_width),
            )
            .unwrap_or(MAX_HEIGHT)
            .max(1),
        )
    } else {
        (
            u32::try_from(
                (u64::from(source_width) * u64::from(MAX_HEIGHT))
                    / u64::from(source_height),
            )
            .unwrap_or(MAX_WIDTH)
            .max(1),
            MAX_HEIGHT,
        )
    };
    let mut pixels = Vec::with_capacity(width as usize * height as usize);
    for y in 0..height {
        let source_y = u32::try_from(
            u64::from(y) * u64::from(source_height) / u64::from(height),
        )
        .expect("preview source y is bounded");
        for x in 0..width {
            let source_x = u32::try_from(
                u64::from(x) * u64::from(source_width) / u64::from(width),
            )
            .expect("preview source x is bounded");
            pixels.push(
                source_pixels[source_y as usize * source_width as usize + source_x as usize],
            );
        }
    }
    let scale_x = f64::from(width) / f64::from(source_width);
    let scale_y = f64::from(height) / f64::from(source_height);
    let corners = detected.map_or_else(Vec::new, |frame| {
        frame
            .pixels
            .iter()
            .map(|point| [point[0] * scale_x, point[1] * scale_y])
            .collect()
    });
    DetectionPreview {
        width,
        height,
        pixels,
        corners,
    }
}

fn owned_gray(
    image: ccl_rosbag::ImageView<'_>,
    mut pixels: Vec<u8>,
) -> Result<DetectionImage> {
    pixels.clear();
    let (width, height) = match image {
        ccl_rosbag::ImageView::Gray8(view) => {
            let (width, height) = (view.width(), view.height());
            pixels.reserve(width as usize * height as usize);
            for y in 0..height {
                pixels.extend_from_slice(view.row(y).expect("validated image row"));
            }
            (width, height)
        }
        ccl_rosbag::ImageView::Gray16Le(view) => {
            let (width, height) = (view.width(), view.height());
            pixels.reserve(width as usize * height as usize);
            for y in 0..height {
                for x in 0..width {
                    let value = view.sample(x, y).expect("validated image coordinate");
                    pixels.push(narrow_luma16(value));
                }
            }
            (width, height)
        }
        ccl_rosbag::ImageView::Rgba8(view) => {
            let (width, height) = (view.width(), view.height());
            pixels.reserve(width as usize * height as usize);
            let row_bytes = width as usize * 4;
            for row in view.pixels().chunks_exact(view.stride()).take(height as usize) {
                for pixel in row[..row_bytes].chunks_exact(4) {
                    pixels.push(
                        u8::try_from(
                            (54 * u32::from(pixel[0])
                                + 183 * u32::from(pixel[1])
                                + 19 * u32::from(pixel[2])
                                + 128)
                                / 256,
                        )
                        .expect("normalized RGBA luminance is bounded"),
                    );
                }
            }
            (width, height)
        }
    };
    Ok(DetectionImage::Gray8 {
        width,
        height,
        pixels,
    })
}

#[cfg(any(target_arch = "wasm32", test))]
const PREVIEW_INTERVAL: Duration = Duration::from_millis(250);

#[cfg(any(target_arch = "wasm32", test))]
struct PreviewSampler {
    last_sampled: Vec<Option<Duration>>,
}

#[cfg(any(target_arch = "wasm32", test))]
impl PreviewSampler {
    fn new(camera_count: usize) -> Self {
        Self {
            last_sampled: vec![None; camera_count],
        }
    }

    fn should_sample(&mut self, camera_index: usize, elapsed: Duration) -> bool {
        let last_sampled = &mut self.last_sampled[camera_index];
        let due = last_sampled.is_none_or(|previous| {
            elapsed.saturating_sub(previous) >= PREVIEW_INTERVAL
        });
        if due {
            *last_sampled = Some(elapsed);
        }
        due
    }
}

struct DetectionRate {
    started: Instant,
    window_started: Instant,
    window_frames: usize,
    total_frames: usize,
    current_fps: f64,
    frames_by_camera: Vec<usize>,
    decode_grayscale: Duration,
    detection: Duration,
    performance: PipelinePerformance,
}

impl DetectionRate {
    fn new(camera_count: usize) -> Self {
        let now = Instant::now();
        Self {
            started: now,
            window_started: now,
            window_frames: 0,
            total_frames: 0,
            current_fps: 0.0,
            frames_by_camera: vec![0; camera_count],
            decode_grayscale: Duration::ZERO,
            detection: Duration::ZERO,
            performance: PipelinePerformance::default(),
        }
    }

    fn record(&mut self, camera_index: usize) -> (usize, f64, f64) {
        self.total_frames += 1;
        self.window_frames += 1;
        self.frames_by_camera[camera_index] += 1;
        let window_elapsed = self.window_started.elapsed();
        if window_elapsed >= Duration::from_millis(250) {
            self.current_fps = f64::from(u32::try_from(self.window_frames).unwrap_or(u32::MAX))
                / window_elapsed.as_secs_f64();
            self.window_started = Instant::now();
            self.window_frames = 0;
        }
        let average = f64::from(u32::try_from(self.total_frames).unwrap_or(u32::MAX))
            / self.started.elapsed().as_secs_f64();
        (
            self.frames_by_camera[camera_index],
            self.current_fps,
            average,
        )
    }
}

#[allow(clippy::too_many_arguments)]
fn collect_detection(
    completion: DetectionCompletion,
    settings: &RunSettings,
    stats: &mut DetectionStats<'_>,
    frames_by_camera: &mut [Vec<RawDetectedFrame>],
    image_sizes: &mut [Option<CalibrationImageSize>],
    rate: &mut DetectionRate,
    workers: usize,
    latest_imu: Option<LatestImu>,
    event: &mut dyn FnMut(DetectionProgress),
) -> Result<()> {
    let (size, detected, preview) = completion.result?;
    rate.decode_grayscale += completion.decode_grayscale;
    rate.detection += completion.detection;
    let performance = &mut rate.performance;
    performance.worker_wait_ms += completion.worker_wait.as_secs_f64() * 1000.0;
    let lag_ms = completion.completed_at.elapsed().as_secs_f64() * 1000.0;
    performance.completion_lag_ms += lag_ms;
    performance.completion_lag_max_ms = performance.completion_lag_max_ms.max(lag_ms);
    performance.decode_grayscale_worker_ms += completion.decode_grayscale.as_secs_f64() * 1000.0;
    performance.detection_worker_ms += completion.detection.as_secs_f64() * 1000.0;
    performance.preview_worker_ms += completion.preview_time.as_secs_f64() * 1000.0;
    if let Some(preview) = &preview {
        performance.preview_count += 1;
        performance.preview_bytes += preview.pixels.len() as u64;
    }
    performance.max_worker_retained_bytes = performance.max_worker_retained_bytes.max(completion.retained_bytes);
    performance.core_worker_ms.record(&completion.core);
    update_image_size(
        &mut image_sizes[completion.camera_index],
        size,
        &settings.cameras[completion.camera_index].id,
    )?;
    let tags = detected.accepted_tags;
    let points = detected.points;
    stats.record(completion.camera_index, &detected);
    frames_by_camera[completion.camera_index].push(RawDetectedFrame {
        timestamp_ns: completion.timestamp_ns,
        ordinal: completion.ordinal,
        detected: detected.frame,
    });
    let (frames_processed, frames_per_second, average_frames_per_second) =
        rate.record(completion.camera_index);
    event(DetectionProgress {
        timestamp_ns: completion.timestamp_ns,
        camera_id: settings.cameras[completion.camera_index].id.clone(),
        frames_processed,
        tags,
        points,
        frames_per_second,
        average_frames_per_second,
        workers,
        decode_grayscale_worker_ms: rate.decode_grayscale.as_secs_f64() * 1000.0,
        detection_worker_ms: rate.detection.as_secs_f64() * 1000.0,
        preview,
        latest_imu,
    });
    Ok(())
}

enum ImageBinding<'schema> {
    Raw(ImageAdapter<'schema>),
    Compressed(Projection<'schema>),
}

struct ImuBinding<'schema> {
    values: Projection<'schema>,
}

impl<'schema> ImuBinding<'schema> {
    fn bind(schema: &'schema CompiledSchema) -> Result<Self> {
        Ok(Self {
            values: schema.projection(&[
                "angular_velocity.x",
                "angular_velocity.y",
                "angular_velocity.z",
                "linear_acceleration.x",
                "linear_acceleration.y",
                "linear_acceleration.z",
            ])?,
        })
    }

    fn read(
        &self,
        message: &ccl_rosbag::schema::DynamicMessageView<'_, '_>,
    ) -> Result<([f64; 3], [f64; 3])> {
        let mut values = [None; 6];
        message.project(&self.values, &mut |field| {
            if let ValueRef::F64(value) = field.value {
                values[field.slot] = Some(value);
                Ok(())
            } else {
                Err(ccl_rosbag::BagError::InvalidPayload(
                    "sensor_msgs/Imu vector field is not float64".to_owned(),
                ))
            }
        })?;
        let values = values
            .into_iter()
            .enumerate()
            .map(|(index, value)| {
                value.with_context(|| format!("missing IMU projection slot {index}"))
            })
            .collect::<Result<Vec<_>>>()?;
        Ok((
            [values[0], values[1], values[2]],
            [values[3], values[4], values[5]],
        ))
    }
}

enum TimestampBinding<'schema> {
    Time(Projection<'schema>),
    Parts(Projection<'schema>),
}

impl<'schema> TimestampBinding<'schema> {
    fn bind(schema: &'schema CompiledSchema) -> Result<Self> {
        if let Ok(projection) = schema.projection(&["header.stamp"]) {
            return Ok(Self::Time(projection));
        }
        Ok(Self::Parts(schema.projection(&[
            "header.stamp.sec",
            "header.stamp.nanosec",
        ])?))
    }

    fn read(
        &self,
        message: &ccl_rosbag::schema::DynamicMessageView<'_, '_>,
    ) -> Result<i64, ccl_rosbag::BagError> {
        match self {
            Self::Time(projection) => {
                let mut timestamp = None;
                message.project(projection, &mut |field| {
                    if let ValueRef::Time { sec, nanosec } = field.value {
                        timestamp = combine_timestamp(i64::from(sec), nanosec);
                        Ok(())
                    } else {
                        Err(ccl_rosbag::BagError::InvalidPayload(
                            "header.stamp is not a ROS time field".to_owned(),
                        ))
                    }
                })?;
                timestamp.ok_or_else(|| {
                    ccl_rosbag::BagError::InvalidPayload("missing header.stamp".to_owned())
                })
            }
            Self::Parts(projection) => {
                let mut sec = None;
                let mut nanosec = None;
                message.project(projection, &mut |field| {
                    match (field.slot, field.value) {
                        (0, ValueRef::I32(value)) => sec = Some(i64::from(value)),
                        (0, ValueRef::U32(value)) => sec = Some(i64::from(value)),
                        (1, ValueRef::U32(value)) => nanosec = Some(value),
                        _ => {
                            return Err(ccl_rosbag::BagError::InvalidPayload(
                                "invalid ROS 2 header stamp".to_owned(),
                            ));
                        }
                    }
                    Ok(())
                })?;
                combine_timestamp(
                    sec.ok_or_else(|| {
                        ccl_rosbag::BagError::InvalidPayload("missing header.stamp.sec".to_owned())
                    })?,
                    nanosec.ok_or_else(|| {
                        ccl_rosbag::BagError::InvalidPayload(
                            "missing header.stamp.nanosec".to_owned(),
                        )
                    })?,
                )
                .ok_or_else(|| {
                    ccl_rosbag::BagError::InvalidPayload("header timestamp overflow".to_owned())
                })
            }
        }
    }
}

#[allow(
    clippy::too_many_lines,
    reason = "one bag visitor owns schema bindings, borrowed payloads, detection, grouping, and summaries"
)]
pub fn load_and_detect(
    settings: &RunSettings,
    cancelled: &dyn Fn() -> bool,
    event: &mut dyn FnMut(DetectionProgress),
) -> Result<(DetectedDataset, InputSummary, DetectionSummary)> {
    let pipeline_started = Instant::now();
    settings.validate()?;
    let bag = open_bag(&settings.source)?;
    let open_time = pipeline_started.elapsed();
    let mut selected_topics = Vec::with_capacity(settings.cameras.len() + 1);
    for camera in &settings.cameras {
        let topic = bag
            .topics()
            .iter()
            .find(|topic| topic.name == camera.topic)
            .with_context(|| format!("image topic {} is absent from bag", camera.topic))?;
        ensure!(
            is_image_type(&topic.type_name),
            "topic {} is not sensor_msgs/Image or CompressedImage",
            camera.topic
        );
        selected_topics.push(topic);
    }
    let imu_topic = settings
        .source
        .imu_topic
        .as_ref()
        .map(|name| {
            bag.topics()
                .iter()
                .find(|topic| topic.name == *name)
                .with_context(|| format!("IMU topic {name} is absent from bag"))
        })
        .transpose()?;
    if let Some(topic) = imu_topic {
        ensure!(
            normalize_type(&topic.type_name) == IMU_TYPE,
            "topic {} is not sensor_msgs/Imu",
            topic.name
        );
        selected_topics.push(topic);
    }

    let registry = standard_schema_registry();
    let schemas = selected_topics
        .iter()
        .map(|topic| CompiledSchema::for_topic(topic, Some(&registry)))
        .collect::<Result<Vec<_>, _>>()?;
    let stamps = schemas
        .iter()
        .map(TimestampBinding::bind)
        .collect::<Result<Vec<_>>>()?;
    let image_bindings = schemas[..settings.cameras.len()]
        .iter()
        .zip(&selected_topics[..settings.cameras.len()])
        .map(|(schema, topic)| match normalize_type(&topic.type_name) {
            RAW_IMAGE_TYPE => Ok(ImageBinding::Raw(ImageAdapter::bind(schema)?)),
            COMPRESSED_IMAGE_TYPE => Ok(ImageBinding::Compressed(schema.projection(&["data"])?)),
            _ => unreachable!(),
        })
        .collect::<Result<Vec<_>>>()?;
    let imu_binding = imu_topic
        .map(|_| {
            schemas
                .last()
                .context("IMU schema is missing")
                .and_then(ImuBinding::bind)
        })
        .transpose()?;
    let topic_indices = selected_topics
        .iter()
        .enumerate()
        .map(|(index, topic)| (topic.id, index))
        .collect::<HashMap<TopicId, usize>>();
    let filter = MessageFilter {
        topics: selected_topics.iter().map(|topic| topic.id).collect(),
        ..MessageFilter::default()
    };
    let detector = FrameDetector::new(settings);
    let mut pool = DetectionPool::new(&detector);
    let workers = pool.workers;
    let mut stats = DetectionStats::new(settings);
    let mut rate = DetectionRate::new(settings.cameras.len());
    rate.performance.workers = workers;
    rate.performance.open_ms = open_time.as_secs_f64() * 1000.0;
    #[cfg(target_arch = "wasm32")]
    let preview_started = Instant::now();
    #[cfg(target_arch = "wasm32")]
    let preview_enabled = crate::browser::preview_enabled();
    #[cfg(target_arch = "wasm32")]
    let mut preview_sampler = PreviewSampler::new(settings.cameras.len());
    let mut frames_by_camera = (0..settings.cameras.len())
        .map(|_| Vec::new())
        .collect::<Vec<Vec<RawDetectedFrame>>>();
    let mut image_sizes = vec![None; settings.cameras.len()];
    let mut imu_rows = Vec::new();
    let mut processing_error = None;
    let mut ordinal = 0_u64;
    let mut submitted_by_camera = vec![0_usize; settings.cameras.len()];
    let mut latest_imu = None;
    #[cfg(target_arch = "wasm32")]
    let mut last_imu_event_at: Option<Instant> = None;
    #[cfg(target_arch = "wasm32")]
    let mut last_imu_event_count = 0_usize;

    let visit_started = Instant::now();
    let visit_result = bag.visit(&filter, &mut |raw| {
        if cancelled() {
            processing_error = Some(anyhow!("calibration cancelled"));
            return Ok(Visit::Stop);
        }
        let Some(&selected_index) = topic_indices.get(&raw.topic_id) else {
            return Ok(Visit::Continue);
        };
        let result = (|| -> Result<()> {
            let message = schemas[selected_index].view(raw.payload)?;
            let timestamp_ns = stamps[selected_index].read(&message)?;
            if selected_index < settings.cameras.len() {
                let camera_index = selected_index;
                let context = format!(
                    "{} at header={}",
                    settings.cameras[camera_index].topic, timestamp_ns
                );
                let prepare_started = Instant::now();
                let image = match &image_bindings[camera_index] {
                    ImageBinding::Raw(adapter) => owned_gray(
                        adapter.to_view(&message)?,
                        pool.copy_input(&[]),
                    )?,
                    ImageBinding::Compressed(projection) => DetectionImage::Compressed(
                        pool.copy_input(projected_bytes(&message, projection)?),
                    ),
                };
                rate.performance.input_prepare_ms += prepare_started.elapsed().as_secs_f64() * 1000.0;
                rate.performance.input_payload_bytes += match &image {
                    DetectionImage::Compressed(bytes) | DetectionImage::Gray8 { pixels: bytes, .. } => bytes.len() as u64,
                };
                let submitted = submitted_by_camera[camera_index];
                submitted_by_camera[camera_index] = submitted
                    .checked_add(1)
                    .context("camera frame count overflow")?;
                #[cfg(target_arch = "wasm32")]
                let preview_requested = preview_enabled &&
                    preview_sampler.should_sample(camera_index, preview_started.elapsed());
                #[cfg(not(target_arch = "wasm32"))]
                let preview_requested = false;
                let submit_started = Instant::now();
                pool.submit(DetectionTask {
                    camera_index,
                    timestamp_ns,
                    ordinal,
                    context,
                    image,
                    preview_requested,
                })?;
                rate.performance.submit_ms += submit_started.elapsed().as_secs_f64() * 1000.0;
                ordinal = ordinal.checked_add(1).context("frame ordinal overflow")?;
            } else {
                let binding = imu_binding.as_ref().context("missing IMU binding")?;
                let (gyro_rad_s, specific_force_m_s2) = binding.read(&message)?;
                imu_rows.push(RawImuSample {
                    timestamp_ns,
                    gyro_rad_s,
                    specific_force_m_s2,
                });
                let progress = LatestImu {
                    sample_count: imu_rows.len(),
                    timestamp_ns,
                    gyro_rad_s,
                    specific_force_m_s2,
                };
                latest_imu = Some(progress);
                #[cfg(target_arch = "wasm32")]
                {
                    let now = Instant::now();
                    if last_imu_event_at.map_or(true, |previous| {
                        now.duration_since(previous) >= Duration::from_millis(100)
                    }) {
                        crate::browser::emit_imu_progress(progress)?;
                        last_imu_event_at = Some(now);
                        last_imu_event_count = imu_rows.len();
                    }
                }
            }
            while let Some(mut completion) = pool.try_recv() {
                pool.recycle_completion(&mut completion);
                collect_detection(
                    completion,
                    settings,
                    &mut stats,
                    &mut frames_by_camera,
                    &mut image_sizes,
                    &mut rate,
                    workers,
                    latest_imu,
                    event,
                )?;
            }
            Ok(())
        })();
        if let Err(error) = result {
            processing_error = Some(error);
            return Ok(Visit::Stop);
        }
        Ok(Visit::Continue)
    });
    rate.performance.visit_ms = visit_started.elapsed().as_secs_f64() * 1000.0;
    let finish_started = Instant::now();
    let mut remaining = pool.finish()?;
    for completion in &mut remaining {
        pool.recycle_completion(completion);
    }
    rate.performance.finish_ms = finish_started.elapsed().as_secs_f64() * 1000.0;
    visit_result?;
    #[cfg(target_arch = "wasm32")]
    if processing_error.is_none()
        && let Some(progress) = latest_imu
        && progress.sample_count != last_imu_event_count
    {
        crate::browser::emit_imu_progress(progress)?;
    }
    if processing_error.is_none() {
        for completion in remaining {
            collect_detection(
                completion,
                settings,
                &mut stats,
                &mut frames_by_camera,
                &mut image_sizes,
                &mut rate,
                workers,
                latest_imu,
                event,
            )?;
        }
    }
    if let Some(error) = processing_error {
        return Err(error);
    }
    ensure!(
        frames_by_camera.iter().all(|frames| !frames.is_empty()),
        "one or more selected image topics contain no frames"
    );
    let image_sizes = image_sizes
        .into_iter()
        .enumerate()
        .map(|(index, size)| {
            size.with_context(|| {
                format!(
                    "camera {} has no decodable image",
                    settings.cameras[index].id
                )
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let anchor_camera = synchronization_anchor(settings)?;
    let anchor_frame_count = frames_by_camera[anchor_camera].len();
    let counts = frames_by_camera.iter().map(Vec::len).collect::<Vec<_>>();
    let exposure_durations_ns = load_exposure_durations(settings, bag.as_ref())?;
    let per_camera =
        detected_frames_by_camera(settings, &mut frames_by_camera, &exposure_durations_ns)?;
    // One detector pass feeds both groupings: exact-time union views for the continuous-time
    // solve, and approximate shared views (on the anchor camera's timeline) for the visual rig
    // seed and the non-continuous-time modes.
    let exact_groups = exact_time_views(&per_camera);
    ensure!(!exact_groups.is_empty(), "no usable AprilGrid observations");
    let synchronized_groups = synchronized_views(
        &per_camera,
        anchor_camera,
        settings.source.synchronization.max_delta_ns,
        settings.source.synchronization.require_all_cameras,
    )?;
    ensure!(
        !synchronized_groups.is_empty(),
        "no synchronized camera views found"
    );
    let first_timestamp_ns = exact_groups.first().map(|group| group.0);
    let last_timestamp_ns = exact_groups.last().map(|group| group.0);
    let views = detected_views(exact_groups);
    let shared_views = detected_views(synchronized_groups);
    let usable_view_count = shared_views.len();
    let input_view_count = anchor_frame_count;

    // Bag backends expose storage order, which is not guaranteed to be timestamp order.
    // The visual path sorts each camera in `detected_frames_by_camera`; inertial integration
    // needs the same explicit ordering before deriving bounds and constructing `ImuSequence`.
    imu_rows.sort_by_key(|sample| sample.timestamp_ns);
    let view_origin = first_timestamp_ns.context("missing first image timestamp")?;
    let time_origin_ns = imu_rows
        .first()
        .map_or(view_origin, |sample| sample.timestamp_ns.min(view_origin));
    let imu_bounds = (
        imu_rows.first().map(|sample| sample.timestamp_ns),
        imu_rows.last().map(|sample| sample.timestamp_ns),
    );
    let imu_samples = (!imu_rows.is_empty())
        .then(|| {
            imu_rows
                .iter()
                .map(|sample| {
                    Ok(ImuSample {
                        time: SampleTime::from_nanoseconds(sample.timestamp_ns, time_origin_ns)?,
                        gyro_rad_s: sample.gyro_rad_s,
                        specific_force_m_s2: sample.specific_force_m_s2,
                    })
                })
                .collect::<Result<Vec<_>>>()
        })
        .transpose()?;
    let frame_count_by_camera = settings
        .cameras
        .iter()
        .enumerate()
        .map(|(index, camera)| (camera.id.clone(), counts[index]))
        .collect();
    let image_size_by_camera = settings
        .cameras
        .iter()
        .enumerate()
        .map(|(index, camera)| {
            let size = image_sizes[index];
            (
                camera.id.clone(),
                ImageSizeSummary {
                    width: size.width,
                    height: size.height,
                },
            )
        })
        .collect();
    rate.performance.total_ms = pipeline_started.elapsed().as_secs_f64() * 1000.0;
    let summary = InputSummary {
        schema_version: 2,
        source_type: "rosbag",
        bag_path: settings.source.path.clone(),
        view_count: input_view_count,
        frame_count_by_camera,
        image_size_by_camera,
        first_timestamp_ns,
        last_timestamp_ns,
        imu_sample_count: imu_samples.as_ref().map_or(0, Vec::len),
        imu_first_timestamp_ns: imu_bounds.0,
        imu_last_timestamp_ns: imu_bounds.1,
        performance: rate.performance,
    };
    let detection_summary = stats.summary(input_view_count, usable_view_count);
    Ok((
        DetectedDataset {
            views,
            synchronized_views: shared_views,
            image_sizes,
            imu_samples,
            time_origin_ns: Some(time_origin_ns),
        },
        summary,
        detection_summary,
    ))
}

/// The shared-view anchor is fixed by the calibration mode: the selected camera for
/// single-camera, `reference_camera` for multi-camera, and `visual_reference_camera` for
/// camera-IMU. Only this camera's header times define shared-view times.
fn synchronization_anchor(settings: &RunSettings) -> Result<usize> {
    let id = match &settings.calibration {
        CalibrationConfig::SingleCamera { camera, .. } => camera,
        CalibrationConfig::MultiCamera {
            reference_camera, ..
        } => reference_camera,
        CalibrationConfig::MultiCameraImu {
            visual_reference_camera,
            ..
        } => visual_reference_camera,
    };
    settings
        .cameras
        .iter()
        .position(|camera| camera.id.as_str() == id.as_str())
        .with_context(|| format!("unknown synchronization anchor camera {id}"))
}

fn load_exposure_durations(
    settings: &RunSettings,
    bag: &dyn BagReader,
) -> Result<Vec<HashMap<i64, u64>>> {
    let configured = settings
        .cameras
        .iter()
        .enumerate()
        .filter_map(|(camera_index, camera)| {
            camera
                .timing
                .exposure_metadata_topic
                .as_deref()
                .map(|name| (camera_index, name))
        })
        .map(|(camera_index, name)| {
            let topic = bag
                .topics()
                .iter()
                .find(|topic| topic.name == name)
                .with_context(|| format!("exposure metadata topic {name} is absent from bag"))?;
            ensure!(
                normalize_type(&topic.type_name) == STRING_TYPE,
                "exposure metadata topic {name} is not std_msgs/String"
            );
            Ok((camera_index, topic))
        })
        .collect::<Result<Vec<_>>>()?;
    let mut durations = (0..settings.cameras.len())
        .map(|_| HashMap::new())
        .collect::<Vec<HashMap<i64, u64>>>();
    if configured.is_empty() {
        return Ok(durations);
    }
    let schemas = configured
        .iter()
        .map(|(_, topic)| CompiledSchema::for_topic(topic, None))
        .collect::<Result<Vec<_>, _>>()?;
    let projections = schemas
        .iter()
        .map(|schema| schema.projection(&["data"]))
        .collect::<Result<Vec<_>, _>>()?;
    let bindings = configured
        .iter()
        .enumerate()
        .map(|(binding, (camera_index, topic))| (topic.id, (binding, *camera_index)))
        .collect::<HashMap<_, _>>();
    let filter = MessageFilter {
        topics: configured.iter().map(|(_, topic)| topic.id).collect(),
        ..MessageFilter::default()
    };
    bag.visit(&filter, &mut |raw| {
        let (binding, camera_index) = bindings[&raw.topic_id];
        let message = schemas[binding].view(raw.payload)?;
        message.project(&projections[binding], &mut |field| {
            let ValueRef::String(json) = field.value else {
                return Err(ccl_rosbag::BagError::InvalidPayload(
                    "exposure metadata data field is not a string".to_owned(),
                ));
            };
            let metadata: FrameExposureMetadata = serde_json::from_str(json).map_err(|error| {
                ccl_rosbag::BagError::InvalidPayload(format!("invalid exposure metadata: {error}"))
            })?;
            if metadata.exposure.duration_ns == 0
                || metadata.exposure.duration_ns > i64::MAX.cast_unsigned()
            {
                return Err(ccl_rosbag::BagError::InvalidPayload(
                    "exposure duration must be positive and representable".to_owned(),
                ));
            }
            if let Some(previous) = durations[camera_index]
                .insert(metadata.camera_timestamp_ns, metadata.exposure.duration_ns)
                && previous != metadata.exposure.duration_ns
            {
                return Err(ccl_rosbag::BagError::InvalidPayload(format!(
                    "conflicting exposure durations at timestamp {}",
                    metadata.camera_timestamp_ns
                )));
            }
            Ok(())
        })?;
        Ok(Visit::Continue)
    })?;
    for (camera_index, topic) in configured {
        ensure!(
            !durations[camera_index].is_empty(),
            "exposure metadata topic {} contains no usable messages",
            topic.name
        );
    }
    Ok(durations)
}

/// Preserve missing detections while matching source timestamps: a detector miss must
/// not shift a camera to a different exposure. Duplicate midpoint timestamps are errors.
fn detected_frames_by_camera(
    settings: &RunSettings,
    frames_by_camera: &mut [Vec<RawDetectedFrame>],
    exposure_durations_ns: &[HashMap<i64, u64>],
) -> Result<Vec<CameraDetectedFrames>> {
    frames_by_camera
        .iter_mut()
        .enumerate()
        .map(|(camera_index, frames)| {
            let camera = &settings.cameras[camera_index];
            for frame in frames.iter_mut() {
                let duration_s = if camera.timing.exposure_metadata_topic.is_some() {
                    let duration_ns = exposure_durations_ns[camera_index]
                        .get(&frame.timestamp_ns)
                        .copied()
                        .with_context(|| {
                            format!(
                                "camera {} has no exposure metadata for header timestamp {} ns",
                                camera.id, frame.timestamp_ns
                            )
                        })?;
                    #[allow(clippy::cast_precision_loss)]
                    let exposure_seconds = duration_ns as f64 * 1.0e-9;
                    Some(exposure_seconds)
                } else {
                    camera.timing.fixed_exposure_duration_s
                };
                frame.timestamp_ns =
                    camera_midpoint_timestamp_ns(camera, frame.timestamp_ns, duration_s)?;
                if let Some(detected) = &mut frame.detected {
                    detected.exposure_duration_s = duration_s;
                }
            }
            frames.sort_by_key(|frame| (frame.timestamp_ns, frame.ordinal));
            if let Some(pair) = frames
                .windows(2)
                .find(|pair| pair[0].timestamp_ns == pair[1].timestamp_ns)
            {
                return Err(anyhow!(
                    "camera {} has duplicate exposure-midpoint timestamp {} ns",
                    camera.id,
                    pair[0].timestamp_ns
                ));
            }
            Ok(frames
                .drain(..)
                .map(|frame| (frame.timestamp_ns, frame.detected.map(Arc::new)))
                .collect())
        })
        .collect()
}

/// Exact-time union: every distinct source timestamp becomes one view holding exactly the
/// cameras that published at that instant. Async cameras therefore keep their own stamps;
/// nothing is retimed to an anchor and no observed frame is dropped for being unmatched.
fn exact_time_views(
    frames_by_camera: &[CameraDetectedFrames],
) -> Vec<(i64, Vec<Arc<DetectedFrame>>)> {
    let mut by_timestamp: BTreeMap<i64, Vec<Arc<DetectedFrame>>> = BTreeMap::new();
    for frames in frames_by_camera {
        for (timestamp_ns, frame) in frames {
            let Some(frame) = frame else {
                continue;
            };
            by_timestamp
                .entry(*timestamp_ns)
                .or_default()
                .push(Arc::clone(frame));
        }
    }
    by_timestamp.into_iter().collect()
}

/// Approximate shared views anchored on `anchor_camera`'s own header times: for every anchor
/// frame that has a usable `AprilGrid` observation, take the unique nearest frame of every other
/// camera within `max_delta_ns`. An equal nearest distance is ambiguous and rejected rather
/// than broken arbitrarily. One message belongs to at most one shared view. With `require_all`,
/// a group missing any camera is dropped; without it, a partial group is kept.
fn synchronized_views(
    frames_by_camera: &[CameraDetectedFrames],
    anchor_camera: usize,
    max_delta_ns: i64,
    require_all: bool,
) -> Result<Vec<(i64, Vec<Arc<DetectedFrame>>)>> {
    let max_delta = max_delta_ns.cast_unsigned();
    let mut cursors = vec![0_usize; frames_by_camera.len()];
    let mut groups = Vec::new();
    for anchor_index in 0..frames_by_camera[anchor_camera].len() {
        let anchor_time = frames_by_camera[anchor_camera][anchor_index].0;
        if frames_by_camera[anchor_camera][anchor_index].1.is_none() {
            continue;
        }
        let mut selected = vec![None; frames_by_camera.len()];
        selected[anchor_camera] = Some(anchor_index);
        for camera_index in 0..frames_by_camera.len() {
            if camera_index == anchor_camera {
                continue;
            }
            let frames = &frames_by_camera[camera_index];
            let cursor = &mut cursors[camera_index];
            let upper = *cursor + frames[*cursor..].partition_point(|frame| frame.0 < anchor_time);
            let mut nearest: Option<(usize, u64)> = None;
            for candidate in [
                upper.checked_sub(1).filter(|index| *index >= *cursor),
                Some(upper),
            ]
            .into_iter()
            .flatten()
            {
                let Some(frame) = frames.get(candidate) else {
                    continue;
                };
                let delta = frame.0.abs_diff(anchor_time);
                if delta > max_delta {
                    continue;
                }
                match nearest {
                    Some((_, best)) if best < delta => {}
                    Some((_, best)) if best == delta => {
                        return Err(anyhow!(
                            "ambiguous nearest frame for camera {camera_index} at anchor timestamp {anchor_time} ns: two frames are {delta} ns away; adjust source.synchronization.max_delta_ns"
                        ));
                    }
                    _ => nearest = Some((candidate, delta)),
                }
            }
            selected[camera_index] = nearest.map(|(index, _)| index);
        }
        if require_all && selected.iter().any(Option::is_none) {
            continue;
        }
        let mut frames = Vec::with_capacity(selected.len());
        for (camera_index, frame_index) in selected.into_iter().enumerate() {
            let Some(frame_index) = frame_index else {
                continue;
            };
            if camera_index != anchor_camera {
                cursors[camera_index] = frame_index + 1;
            }
            if let Some(frame) = &frames_by_camera[camera_index][frame_index].1 {
                frames.push(Arc::clone(frame));
            }
        }
        groups.push((anchor_time, frames));
    }
    Ok(groups)
}

fn detected_views(groups: Vec<(i64, Vec<Arc<DetectedFrame>>)>) -> Vec<DetectedView> {
    groups
        .into_iter()
        .enumerate()
        .filter_map(|(index, (timestamp_ns, frames))| {
            (!frames.is_empty()).then_some(DetectedView {
                id: format!("view-{index:06}"),
                timestamp_ns: Some(timestamp_ns),
                frames,
            })
        })
        .collect()
}

fn update_image_size(
    current: &mut Option<CalibrationImageSize>,
    next: CalibrationImageSize,
    camera: &str,
) -> Result<()> {
    if let Some(current) = current {
        ensure!(
            *current == next,
            "camera {camera} changes image size within the bag"
        );
    } else {
        *current = Some(next);
    }
    Ok(())
}

fn projected_bytes<'payload>(
    message: &ccl_rosbag::schema::DynamicMessageView<'_, 'payload>,
    projection: &Projection<'_>,
) -> Result<&'payload [u8]> {
    let mut bytes = None;
    message.project(projection, &mut |field| {
        if let ValueRef::Bytes(value) = field.value {
            bytes = Some(value);
            Ok(())
        } else {
            Err(ccl_rosbag::BagError::InvalidPayload(
                "CompressedImage.data is not bytes".to_owned(),
            ))
        }
    })?;
    bytes.context("missing CompressedImage.data")
}

#[allow(clippy::cast_possible_truncation)]
fn camera_midpoint_timestamp_ns(
    camera: &CameraConfig,
    header_timestamp_ns: i64,
    exposure_duration_s: Option<f64>,
) -> Result<i64> {
    if matches!(
        camera.timing.timestamp_reference,
        CameraTimestampReference::ExposureMidpoint
    ) {
        return Ok(header_timestamp_ns);
    }
    let duration_s = exposure_duration_s.context("trigger timestamp requires exposure duration")?;
    let half_duration_ns = (duration_s * 0.5 * 1.0e9).round() as i64;
    header_timestamp_ns
        .checked_add(half_duration_ns)
        .with_context(|| format!("camera {} midpoint timestamp overflow", camera.id))
}

fn combine_timestamp(sec: i64, nanosec: u32) -> Option<i64> {
    sec.checked_mul(1_000_000_000)?
        .checked_add(i64::from(nanosec))
}

fn normalize_type(type_name: &str) -> &str {
    match type_name {
        "sensor_msgs/msg/Image" => RAW_IMAGE_TYPE,
        "sensor_msgs/msg/CompressedImage" => COMPRESSED_IMAGE_TYPE,
        "sensor_msgs/msg/Imu" => IMU_TYPE,
        "std_msgs/msg/String" => STRING_TYPE,
        _ => type_name,
    }
}

fn is_image_type(type_name: &str) -> bool {
    matches!(
        normalize_type(type_name),
        RAW_IMAGE_TYPE | COMPRESSED_IMAGE_TYPE
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(camera_index: usize) -> Arc<DetectedFrame> {
        Arc::new(DetectedFrame {
            camera_index,
            image_size: CalibrationImageSize::new(640, 480).unwrap(),
            target_indices: (0..4).collect(),
            pixels: vec![[0.0; 2]; 4],
            exposure_duration_s: None,
        })
    }

    fn timeline(entries: &[(i64, usize)]) -> CameraDetectedFrames {
        entries
            .iter()
            .map(|&(timestamp_ns, camera_index)| (timestamp_ns, Some(frame(camera_index))))
            .collect()
    }

    fn camera_indices(frames: &[Arc<DetectedFrame>]) -> Vec<usize> {
        frames.iter().map(|frame| frame.camera_index).collect()
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn detection_settings() -> RunSettings {
        let camera_0 = timing_camera(CameraTimestampReference::ExposureMidpoint);
        let mut camera_1 = camera_0.clone();
        camera_1.id = "cam1".to_owned();
        camera_1.topic = "/cam1/image".to_owned();
        RunSettings {
            source: BagSourceConfig {
                path: std::path::PathBuf::from("recording.bag"),
                format: crate::config::BagFormatConfig::Ros1V2,
                imu_topic: None,
                synchronization: crate::config::SynchronizationConfig::default(),
            },
            cameras: vec![camera_0, camera_1],
            target: crate::config::TargetConfig::default(),
            detector: crate::config::DetectorConfig::default(),
            calibration: CalibrationConfig::SingleCamera {
                camera: "cam0".to_owned(),
                initial_intrinsics: crate::config::InitialIntrinsicsConfig::default(),
            },
            solver: crate::config::SolverConfig::default(),
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn detection_pool_preserves_independent_camera_task_identity() {
        let settings = detection_settings();
        let detector = FrameDetector::new(&settings);
        let mut pool = DetectionPool::new(&detector);
        let expected = [
            (0_usize, 100_i64, 0_u64, 32_u32),
            (1, 250, 1, 33),
            (0, 300, 2, 34),
            (0, 700, 3, 35),
            (1, 900, 4, 36),
        ];
        for &(camera_index, timestamp_ns, ordinal, width) in &expected {
            pool.submit(DetectionTask {
                camera_index,
                timestamp_ns,
                ordinal,
                context: format!("camera {camera_index} frame {ordinal}"),
                image: DetectionImage::Gray8 {
                    width,
                    height: 24,
                    pixels: vec![255; width as usize * 24],
                },
                preview_requested: false,
            })
            .unwrap();
        }

        let mut actual = pool
            .finish()
            .unwrap()
            .into_iter()
            .map(|completion| {
                let size = completion.result.unwrap().0;
                (
                    completion.camera_index,
                    completion.timestamp_ns,
                    completion.ordinal,
                    size.width,
                )
            })
            .collect::<Vec<_>>();
        actual.sort_by_key(|entry| entry.2);
        assert_eq!(actual, expected);
        assert_eq!(
            actual.iter().filter(|entry| entry.0 == 0).count(),
            3
        );
        assert_eq!(
            actual.iter().filter(|entry| entry.0 == 1).count(),
            2
        );
    }

    #[test]
    fn forwards_every_detected_view_without_solver_side_thinning() {
        let groups = (0..100)
            .map(|index| (i64::from(index), vec![frame(0)]))
            .collect();
        let views = detected_views(groups);
        let timestamps = views
            .iter()
            .map(|view| view.timestamp_ns.unwrap())
            .collect::<Vec<_>>();
        assert_eq!(views.len(), 100);
        assert_eq!(timestamps, (0_i64..100).collect::<Vec<_>>());
    }

    #[test]
    fn exact_time_views_keep_asynchronous_non_anchor_frames() {
        // cam1 exposes 3 ms after cam0. Union views must not retime cam1 to cam0 or drop it.
        let frames_by_camera = [
            timeline(&[(0, 0), (10_000_000, 0), (20_000_000, 0)]),
            timeline(&[(3_000_000, 1), (13_000_000, 1)]),
        ];
        let groups = exact_time_views(&frames_by_camera);
        let timestamps = groups.iter().map(|group| group.0).collect::<Vec<_>>();
        assert_eq!(
            timestamps,
            vec![0, 3_000_000, 10_000_000, 13_000_000, 20_000_000]
        );
        assert_eq!(camera_indices(&groups[1].1), vec![1]);
        assert_eq!(camera_indices(&groups[3].1), vec![1]);
    }

    #[test]
    fn exact_time_views_keep_unmatched_last_anchor_frame() {
        // cam0's last frame has no cam1 partner. The exact union retains it with its own
        // timestamp; the approximate shared views still drop it when every camera is required.
        let frames_by_camera = [
            timeline(&[(0, 0), (10, 0), (20, 0)]),
            timeline(&[(0, 1), (10, 1)]),
        ];
        let exact = exact_time_views(&frames_by_camera);
        assert_eq!(exact.len(), 3);
        assert_eq!(exact[2].0, 20);
        assert_eq!(camera_indices(&exact[2].1), vec![0]);
        let synchronized = synchronized_views(&frames_by_camera, 0, 0, true).unwrap();
        assert_eq!(
            synchronized.iter().map(|group| group.0).collect::<Vec<_>>(),
            vec![0, 10]
        );
    }

    #[test]
    fn synchronized_views_follow_the_anchor_camera_timeline() {
        // Camera 1 is asynchronous by 3 ms but is the anchor: shared-view times and frame
        // pairing follow its own stamps, so the visual rig seed keeps a reference timeline.
        let frames_by_camera = [
            timeline(&[(0, 0), (10_000_000, 0)]),
            timeline(&[(3_000_000, 1), (13_000_000, 1)]),
        ];
        let groups = synchronized_views(&frames_by_camera, 1, 5_000_000, true).unwrap();
        assert_eq!(
            groups.iter().map(|group| group.0).collect::<Vec<_>>(),
            vec![3_000_000, 13_000_000]
        );
        for (_, frames) in &groups {
            assert_eq!(camera_indices(frames), vec![0, 1]);
        }
    }

    #[test]
    fn synchronized_views_reject_ambiguous_nearest_frames() {
        // cam1 has frames exactly 100 ns on either side of the cam0 anchor. Picking either
        // one silently would mispair the shared view.
        let frames_by_camera = [timeline(&[(100, 0)]), timeline(&[(0, 1), (200, 1)])];
        assert!(synchronized_views(&frames_by_camera, 0, 200, true).is_err());
    }
    #[test]
    fn nearest_exposure_is_selected_before_detection_filtering() {
        let nearest = frame(1);
        let frames = [
            timeline(&[(100, 0)]),
            vec![
                (10, Some(frame(1))),
                (20, Some(frame(1))),
                (99, Some(Arc::clone(&nearest))),
            ],
        ];
        let groups = synchronized_views(&frames, 0, 100, true).unwrap();
        assert!(Arc::ptr_eq(&groups[0].1[1], &nearest));
        let frames = [
            timeline(&[(100, 0)]),
            vec![(99, None), (101, Some(frame(1)))],
        ];
        assert!(synchronized_views(&frames, 0, 5, true).is_err());
        let frames = [
            timeline(&[(100, 0)]),
            vec![(100, None), (101, Some(frame(1)))],
        ];
        let groups = synchronized_views(&frames, 0, 5, true).unwrap();
        assert_eq!(camera_indices(&groups[0].1), vec![0]);
    }

    fn native_u16_bytes(values: &[u16]) -> Vec<u8> {
        values.iter().flat_map(|value| value.to_ne_bytes()).collect()
    }

    fn native_f32_bytes(values: &[f32]) -> Vec<u8> {
        values.iter().flat_map(|value| value.to_ne_bytes()).collect()
    }

    fn assert_gray_matches_image(
        color: image::ColorType,
        decoded: &[u8],
        image: image::DynamicImage,
    ) {
        let expected = image.into_luma8().into_raw();
        let mut actual = Vec::new();
        decoded_pixels_to_gray(color, decoded, &mut actual).unwrap();
        assert_eq!(actual, expected, "{color:?} grayscale differs");
    }

    #[test]
    fn decoded_grayscale_is_bitexact_with_image_for_every_color_type() {
        let l8 = vec![0, 1, 127, 128, 254, 255];
        assert_gray_matches_image(
            image::ColorType::L8,
            &l8,
            image::DynamicImage::ImageLuma8(image::GrayImage::from_raw(6, 1, l8.clone()).unwrap()),
        );

        let la8 = vec![0, 255, 1, 0, 127, 31, 128, 17, 254, 3, 255, 1];
        assert_gray_matches_image(
            image::ColorType::La8,
            &la8,
            image::DynamicImage::ImageLumaA8(
                image::GrayAlphaImage::from_raw(6, 1, la8.clone()).unwrap(),
            ),
        );

        let rgb8 = vec![
            0, 0, 0, 255, 255, 255, 255, 0, 0, 0, 255, 0, 0, 0, 255, 1, 1, 1,
            17, 83, 201, 254, 127, 3,
        ];
        assert_gray_matches_image(
            image::ColorType::Rgb8,
            &rgb8,
            image::DynamicImage::ImageRgb8(image::RgbImage::from_raw(8, 1, rgb8.clone()).unwrap()),
        );

        let rgba8 = rgb8
            .chunks_exact(3)
            .enumerate()
            .flat_map(|(index, rgb)| [rgb[0], rgb[1], rgb[2], index as u8 * 31])
            .collect::<Vec<_>>();
        assert_gray_matches_image(
            image::ColorType::Rgba8,
            &rgba8,
            image::DynamicImage::ImageRgba8(
                image::RgbaImage::from_raw(8, 1, rgba8.clone()).unwrap(),
            ),
        );

        let l16 = vec![0, 1, 128, 32_767, 32_768, 65_534, 65_535];
        assert_gray_matches_image(
            image::ColorType::L16,
            &native_u16_bytes(&l16),
            image::DynamicImage::ImageLuma16(
                image::ImageBuffer::<image::Luma<u16>, _>::from_raw(7, 1, l16.clone()).unwrap(),
            ),
        );

        let la16 = l16
            .iter()
            .enumerate()
            .flat_map(|(index, &luma)| [luma, u16::MAX - index as u16])
            .collect::<Vec<_>>();
        assert_gray_matches_image(
            image::ColorType::La16,
            &native_u16_bytes(&la16),
            image::DynamicImage::ImageLumaA16(
                image::ImageBuffer::<image::LumaA<u16>, _>::from_raw(7, 1, la16.clone()).unwrap(),
            ),
        );

        let rgb16 = vec![
            0, 0, 0, 65_535, 65_535, 65_535, 65_535, 0, 0, 0, 65_535, 0, 0, 0, 65_535,
            1, 257, 65_534, 32_767, 32_768, 11_111,
        ];
        assert_gray_matches_image(
            image::ColorType::Rgb16,
            &native_u16_bytes(&rgb16),
            image::DynamicImage::ImageRgb16(
                image::ImageBuffer::<image::Rgb<u16>, _>::from_raw(7, 1, rgb16.clone()).unwrap(),
            ),
        );

        let rgba16 = rgb16
            .chunks_exact(3)
            .enumerate()
            .flat_map(|(index, rgb)| [rgb[0], rgb[1], rgb[2], index as u16 * 5_000])
            .collect::<Vec<_>>();
        assert_gray_matches_image(
            image::ColorType::Rgba16,
            &native_u16_bytes(&rgba16),
            image::DynamicImage::ImageRgba16(
                image::ImageBuffer::<image::Rgba<u16>, _>::from_raw(7, 1, rgba16.clone()).unwrap(),
            ),
        );

        let rgb32 = vec![
            0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.125, 0.5, 0.875,
        ];
        assert_gray_matches_image(
            image::ColorType::Rgb32F,
            &native_f32_bytes(&rgb32),
            image::DynamicImage::ImageRgb32F(
                image::Rgb32FImage::from_raw(4, 1, rgb32.clone()).unwrap(),
            ),
        );

        let rgba32 = rgb32
            .chunks_exact(3)
            .enumerate()
            .flat_map(|(index, rgb)| [rgb[0], rgb[1], rgb[2], index as f32 / 3.0])
            .collect::<Vec<_>>();
        assert_gray_matches_image(
            image::ColorType::Rgba32F,
            &native_f32_bytes(&rgba32),
            image::DynamicImage::ImageRgba32F(
                image::Rgba32FImage::from_raw(4, 1, rgba32).unwrap(),
            ),
        );
    }

    #[test]
    fn l8_decoder_uses_decoded_buffer_without_gray_copy() {
        let encoded = b"P5\n3 1\n255\n\x00\x80\xff";
        let expected = image::load_from_memory(encoded).unwrap().into_luma8();
        let mut buffers = DetectionBuffers::default();
        {
            let decoded = decode_compressed_gray(encoded, &mut buffers).unwrap();
            assert_eq!((decoded.width, decoded.height), (3, 1));
            assert_eq!(decoded.pixels, expected.as_raw());
        }
        assert!(buffers.gray.is_empty());
    }

    #[test]
    fn input_buffer_cache_reuses_storage_and_stays_bounded() {
        let cache = InputBufferCache::new(2);
        let mut first = cache.take(64);
        first.extend_from_slice(&[1, 2, 3]);
        let allocation = first.as_ptr();
        cache.recycle(first);

        let reused = cache.take(32);
        assert_eq!(reused.as_ptr(), allocation);
        cache.recycle(reused);
        cache.recycle(Vec::with_capacity(16));
        cache.recycle(Vec::with_capacity(8));
        assert_eq!(cache.buffers.borrow().len(), 2);
    }

    #[test]
    fn preview_sampling_is_time_bounded_per_camera() {
        let mut sampler = PreviewSampler::new(2);
        assert!(sampler.should_sample(0, Duration::ZERO));
        assert!(!sampler.should_sample(0, Duration::from_millis(249)));
        assert!(sampler.should_sample(1, Duration::from_millis(249)));
        assert!(sampler.should_sample(0, Duration::from_millis(250)));
        assert!(!sampler.should_sample(1, Duration::from_millis(498)));
        assert!(sampler.should_sample(1, Duration::from_millis(499)));
    }

    #[test]
    fn metadata_duration_is_attached_and_trigger_is_normalized_to_midpoint() {
        let mut camera = timing_camera(CameraTimestampReference::Trigger);
        camera.timing.fixed_exposure_duration_s = None;
        camera.timing.exposure_metadata_topic = Some("/cam0/frame_metadata".to_owned());
        let settings = RunSettings {
            source: BagSourceConfig {
                path: std::path::PathBuf::from("recording.bag"),
                format: crate::config::BagFormatConfig::Ros1V2,
                imu_topic: None,
                synchronization: crate::config::SynchronizationConfig::default(),
            },
            cameras: vec![camera],
            target: crate::config::TargetConfig::default(),
            detector: crate::config::DetectorConfig::default(),
            calibration: CalibrationConfig::SingleCamera {
                camera: "cam0".to_owned(),
                initial_intrinsics: crate::config::InitialIntrinsicsConfig::default(),
            },
            solver: crate::config::SolverConfig::default(),
        };
        let mut raw = vec![vec![RawDetectedFrame {
            timestamp_ns: 1_000_000,
            ordinal: 0,
            detected: Some((*frame(0)).clone()),
        }]];
        let durations = vec![HashMap::from([(1_000_000, 8_000_000)])];
        let normalized = detected_frames_by_camera(&settings, &mut raw, &durations).unwrap();
        assert_eq!(normalized[0][0].0, 5_000_000);
        assert_eq!(
            normalized[0][0].1.as_ref().unwrap().exposure_duration_s,
            Some(0.008)
        );
    }

    fn timing_camera(timestamp_reference: CameraTimestampReference) -> CameraConfig {
        CameraConfig {
            id: "cam0".to_owned(),
            topic: "/cam0/image".to_owned(),
            model: crate::config::CameraModelConfig::PinholeNone,
            pixel_sigma: 1.0,
            timing: crate::config::CameraTimingConfig {
                timestamp_reference,
                fixed_exposure_duration_s: Some(0.01),
                exposure_metadata_topic: None,
            },
        }
    }

    #[test]
    fn exposure_midpoint_headers_are_not_shifted() {
        let camera = timing_camera(CameraTimestampReference::ExposureMidpoint);
        assert_eq!(
            camera_midpoint_timestamp_ns(&camera, 123, Some(0.01)).unwrap(),
            123
        );
    }

    #[test]
    fn trigger_headers_are_shifted_by_half_the_fixed_exposure() {
        let camera = timing_camera(CameraTimestampReference::Trigger);
        assert_eq!(
            camera_midpoint_timestamp_ns(&camera, 123, Some(0.01)).unwrap(),
            5_000_123
        );
    }
}

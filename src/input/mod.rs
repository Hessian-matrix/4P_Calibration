use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, Mutex, mpsc};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, ensure};
use ccl_calib::camera::inertial::ImuSample;
use ccl_calib::camera::{CalibrationImageSize, SampleTime};
use ccl_image::{Gray8View, detection};
use ccl_rosbag::schema::Projection;
use ccl_rosbag::{
    CompiledSchema, ImageAdapter, MessageFilter, MessageSchema, OpenOptions, SchemaEncoding,
    SchemaSource, StaticSchemaRegistry, TopicId, ValueRef, Visit,
};
use serde::Serialize;

use crate::config::{BagSourceConfig, CalibrationConfig, RunSettings};
use crate::detection::{
    DetectedDataset, DetectedFrame, DetectedView, DetectionStats, DetectionSummary, FrameDetection,
    FrameDetector,
};

const RAW_IMAGE_TYPE: &str = "sensor_msgs/Image";
const COMPRESSED_IMAGE_TYPE: &str = "sensor_msgs/CompressedImage";
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

pub fn inspect(source: &BagSourceConfig) -> Result<BagInspection> {
    let bag = ccl_rosbag::open(&source.path, OpenOptions::new(source.format.core()))
        .with_context(|| format!("open bag {}", source.path.display()))?;
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

struct RawImuSample {
    timestamp_ns: i64,
    gyro_rad_s: [f64; 3],
    specific_force_m_s2: [f64; 3],
}

enum DetectionImage {
    Gray8 {
        width: u32,
        height: u32,
        pixels: Vec<u8>,
    },
    Compressed(Vec<u8>),
}

struct DetectionTask {
    camera_index: usize,
    timestamp_ns: i64,
    ordinal: u64,
    context: String,
    image: DetectionImage,
}

struct DetectionCompletion {
    camera_index: usize,
    timestamp_ns: i64,
    ordinal: u64,
    result: Result<(CalibrationImageSize, FrameDetection)>,
}

struct DetectionPool {
    sender: Option<mpsc::SyncSender<DetectionTask>>,
    receiver: mpsc::Receiver<DetectionCompletion>,
    handles: Vec<JoinHandle<()>>,
    workers: usize,
}

impl DetectionPool {
    fn new(detector: &FrameDetector) -> Self {
        let workers = thread::available_parallelism().map_or(1, std::num::NonZero::get);
        let (task_sender, task_receiver) = mpsc::sync_channel(workers.saturating_mul(2));
        let (result_sender, result_receiver) = mpsc::channel();
        let task_receiver = Arc::new(Mutex::new(task_receiver));
        let handles = (0..workers)
            .map(|_| {
                let tasks = Arc::clone(&task_receiver);
                let results = result_sender.clone();
                let detector = detector.clone();
                thread::spawn(move || {
                    loop {
                        let task = {
                            let receiver = tasks
                                .lock()
                                .unwrap_or_else(std::sync::PoisonError::into_inner);
                            receiver.recv()
                        };
                        let Ok(task) = task else { break };
                        let completion = process_detection_task(&detector, task);
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
            workers,
        }
    }

    fn submit(&self, task: DetectionTask) -> Result<()> {
        self.sender
            .as_ref()
            .context("detection pool is closed")?
            .send(task)
            .map_err(|_| anyhow!("detection worker pool stopped"))
    }

    fn try_recv(&self) -> Option<DetectionCompletion> {
        self.receiver.try_recv().ok()
    }

    fn finish(mut self) -> Result<Vec<DetectionCompletion>> {
        self.sender.take();
        for handle in self.handles.drain(..) {
            handle
                .join()
                .map_err(|_| anyhow!("detection worker panicked"))?;
        }
        Ok(self.receiver.into_iter().collect())
    }
}

fn process_detection_task(detector: &FrameDetector, task: DetectionTask) -> DetectionCompletion {
    let DetectionTask {
        camera_index,
        timestamp_ns,
        ordinal,
        context,
        image,
    } = task;
    let result = (|| {
        let image = match image {
            DetectionImage::Gray8 {
                width,
                height,
                pixels,
            } => image::GrayImage::from_raw(width, height, pixels)
                .context("invalid owned gray image")?,
            DetectionImage::Compressed(bytes) => image::load_from_memory(&bytes)
                .with_context(|| format!("decode compressed image {context}"))?
                .into_luma8(),
        };
        let size = CalibrationImageSize::new(image.width(), image.height())?;
        let view = Gray8View::new(
            image.width(),
            image.height(),
            image.width() as usize,
            image.as_raw(),
        )?;
        let detected = detector.detect(
            camera_index,
            size,
            detection::ImageView::Gray8(view),
            &context,
        )?;
        Ok((size, detected))
    })();
    DetectionCompletion {
        camera_index,
        timestamp_ns,
        ordinal,
        result,
    }
}

fn owned_gray(image: ccl_rosbag::ImageView<'_>) -> DetectionImage {
    let (width, height, pixels) = match image {
        ccl_rosbag::ImageView::Gray8(view) => {
            let (width, height) = (view.width(), view.height());
            let mut pixels = Vec::with_capacity(width as usize * height as usize);
            for y in 0..height {
                pixels.extend_from_slice(view.row(y).expect("validated image row"));
            }
            (width, height, pixels)
        }
        ccl_rosbag::ImageView::Gray16Le(view) => {
            let (width, height) = (view.width(), view.height());
            let mut pixels = Vec::with_capacity(width as usize * height as usize);
            for y in 0..height {
                for x in 0..width {
                    let value = view.sample(x, y).expect("validated image coordinate");
                    pixels.push(
                        u8::try_from((u32::from(value) + 128) / 257)
                            .expect("normalized gray16 is bounded"),
                    );
                }
            }
            (width, height, pixels)
        }
        ccl_rosbag::ImageView::Rgba8(view) => {
            let (width, height) = (view.width(), view.height());
            let mut pixels = Vec::with_capacity(width as usize * height as usize);
            for y in 0..height {
                for x in 0..width {
                    let [r, g, b, _] = view.pixel(x, y).expect("validated image coordinate");
                    pixels.push(
                        u8::try_from(
                            (54 * u32::from(r) + 183 * u32::from(g) + 19 * u32::from(b) + 128)
                                / 256,
                        )
                        .expect("normalized RGBA luminance is bounded"),
                    );
                }
            }
            (width, height, pixels)
        }
    };
    DetectionImage::Gray8 {
        width,
        height,
        pixels,
    }
}

struct DetectionRate {
    started: Instant,
    window_started: Instant,
    window_frames: usize,
    total_frames: usize,
    current_fps: f64,
    frames_by_camera: Vec<usize>,
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
    event: &mut dyn FnMut(DetectionProgress),
) -> Result<()> {
    let (size, detected) = completion.result?;
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
    settings.validate()?;
    let bag = ccl_rosbag::open(
        &settings.source.path,
        OpenOptions::new(settings.source.format.core()),
    )
    .with_context(|| format!("open bag {}", settings.source.path.display()))?;
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
    let pool = DetectionPool::new(&detector);
    let workers = pool.workers;
    let mut stats = DetectionStats::new(settings);
    let mut rate = DetectionRate::new(settings.cameras.len());
    let mut frames_by_camera = (0..settings.cameras.len())
        .map(|_| Vec::new())
        .collect::<Vec<Vec<RawDetectedFrame>>>();
    let mut image_sizes = vec![None; settings.cameras.len()];
    let mut imu_rows = Vec::new();
    let mut processing_error = None;
    let mut ordinal = 0_u64;

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
                    "{} at {}",
                    settings.cameras[camera_index].topic, timestamp_ns
                );
                let image = match &image_bindings[camera_index] {
                    ImageBinding::Raw(adapter) => owned_gray(adapter.to_view(&message)?),
                    ImageBinding::Compressed(projection) => {
                        DetectionImage::Compressed(projected_bytes(&message, projection)?.to_vec())
                    }
                };
                pool.submit(DetectionTask {
                    camera_index,
                    timestamp_ns,
                    ordinal,
                    context,
                    image,
                })?;
                ordinal = ordinal.checked_add(1).context("frame ordinal overflow")?;
                while let Some(completion) = pool.try_recv() {
                    collect_detection(
                        completion,
                        settings,
                        &mut stats,
                        &mut frames_by_camera,
                        &mut image_sizes,
                        &mut rate,
                        workers,
                        event,
                    )?;
                }
            } else {
                let binding = imu_binding.as_ref().context("missing IMU binding")?;
                let (gyro_rad_s, specific_force_m_s2) = binding.read(&message)?;
                imu_rows.push(RawImuSample {
                    timestamp_ns,
                    gyro_rad_s,
                    specific_force_m_s2,
                });
            }
            Ok(())
        })();
        if let Err(error) = result {
            processing_error = Some(error);
            return Ok(Visit::Stop);
        }
        Ok(Visit::Continue)
    });
    let remaining = pool.finish()?;
    visit_result?;
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
    let per_camera = detected_frames_by_camera(settings, &mut frames_by_camera)?;
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

/// Preserve missing detections while matching source timestamps: a detector miss must
/// not shift a camera to a different exposure. Duplicate timestamps are source errors.
fn detected_frames_by_camera(
    settings: &RunSettings,
    frames_by_camera: &mut [Vec<RawDetectedFrame>],
) -> Result<Vec<Vec<(i64, Option<Arc<DetectedFrame>>)>>> {
    frames_by_camera
        .iter_mut()
        .enumerate()
        .map(|(camera_index, frames)| {
            frames.sort_by_key(|frame| (frame.timestamp_ns, frame.ordinal));
            if let Some(pair) = frames
                .windows(2)
                .find(|pair| pair[0].timestamp_ns == pair[1].timestamp_ns)
            {
                return Err(anyhow!(
                    "camera {} has duplicate header timestamp {} ns",
                    settings.cameras[camera_index].id,
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
    frames_by_camera: &[Vec<(i64, Option<Arc<DetectedFrame>>)>],
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
/// frame that has a usable AprilGrid observation, take the unique nearest frame of every other
/// camera within `max_delta_ns`. An equal nearest distance is ambiguous and rejected rather
/// than broken arbitrarily. One message belongs to at most one shared view. With `require_all`,
/// a group missing any camera is dropped; without it, a partial group is kept.
fn synchronized_views(
    frames_by_camera: &[Vec<(i64, Option<Arc<DetectedFrame>>)>],
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
                            "ambiguous nearest frame for camera {} at anchor timestamp {anchor_time} ns: two frames are {delta} ns away; adjust source.synchronization.max_delta_ns",
                            camera_index
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

fn combine_timestamp(sec: i64, nanosec: u32) -> Option<i64> {
    sec.checked_mul(1_000_000_000)?
        .checked_add(i64::from(nanosec))
}

fn normalize_type(type_name: &str) -> &str {
    match type_name {
        "sensor_msgs/msg/Image" => RAW_IMAGE_TYPE,
        "sensor_msgs/msg/CompressedImage" => COMPRESSED_IMAGE_TYPE,
        "sensor_msgs/msg/Imu" => IMU_TYPE,
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
        })
    }

    fn timeline(entries: &[(i64, usize)]) -> Vec<(i64, Option<Arc<DetectedFrame>>)> {
        entries
            .iter()
            .map(|&(timestamp_ns, camera_index)| (timestamp_ns, Some(frame(camera_index))))
            .collect()
    }

    fn camera_indices(frames: &[Arc<DetectedFrame>]) -> Vec<usize> {
        frames.iter().map(|frame| frame.camera_index).collect()
    }

    #[test]
    fn forwards_every_detected_view_without_solver_side_thinning() {
        let groups = (0..100)
            .map(|index| (i64::try_from(index).unwrap(), vec![frame(0)]))
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
}

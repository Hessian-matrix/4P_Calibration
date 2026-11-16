use std::cell::RefCell;
use std::io;
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result, ensure};
use ccl_rosbag::{
    BagError, BagReader, MessageFilter, OpenOptions, RandomAccessSource, RawMessage,
    SourceCollection, SourceFile, TopicInfo, Visit,
};
use js_sys::{Function, Uint8Array};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use wasm_bindgen::prelude::*;

use crate::config::{BagSourceConfig, RunSettings};
use crate::run_record::{RunContext, RunStatus};
use crate::runner::Event;

#[cfg(all(feature = "wasm-threads", not(target_feature = "atomics")))]
compile_error!("wasm-threads requires an atomics-enabled rebuilt standard library");

#[cfg(feature = "wasm-threads")]
pub use wasm_bindgen_rayon::init_thread_pool;

static CANCELLED: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// Byte offset into the shared WASM memory; the host resets it before dispatching a new run.
#[cfg(feature = "wasm-threads")]
#[wasm_bindgen]
pub fn cancellation_flag_ptr() -> u32 {
    std::ptr::addr_of!(CANCELLED) as u32
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(catch, js_namespace = ["globalThis", "fourPHost"], js_name = readRange)]
    fn read_range(handle: u32, offset: u64, length: u32) -> std::result::Result<Uint8Array, JsValue>;
    #[wasm_bindgen(js_namespace = ["globalThis", "fourPHost"], js_name = nowMillis)]
    fn now_millis() -> f64;
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct BrowserFile {
    handle: u32,
    relative_path: String,
    size: String,
    #[serde(default)]
    role: Option<String>,
}

const fn preview_default() -> bool {
    true
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct BrowserSource {
    format: crate::config::BagFormatConfig,
    source_id: String,
    source_revision: u64,
    files: Vec<BrowserFile>,
    #[serde(default = "preview_default")]
    preview_enabled: bool,
}

#[derive(PartialEq, Eq)]
struct SourceKey {
    source_id: String,
    source_revision: u64,
    format: u8,
    files: Vec<(u32, String, String, Option<String>)>,
}

impl SourceKey {
    fn format(source: &BrowserSource) -> u8 {
        match source.format {
            crate::config::BagFormatConfig::Ros1V2 => 0,
            crate::config::BagFormatConfig::Ros2Mcap => 1,
            crate::config::BagFormatConfig::Ros2Sqlite3 => 2,
        }
    }

    fn matches(&self, source: &BrowserSource) -> bool {
        self.source_id == source.source_id
            && self.source_revision == source.source_revision
            && self.format == Self::format(source)
            && self.files.len() == source.files.len()
            && self.files.iter().zip(&source.files).all(|(cached, file)| {
                cached.0 == file.handle
                    && cached.1 == file.relative_path
                    && cached.2 == file.size
                    && cached.3 == file.role
            })
    }

    fn from_source(source: &BrowserSource) -> Self {
        Self {
            source_id: source.source_id.clone(),
            source_revision: source.source_revision,
            format: Self::format(source),
            files: source
                .files
                .iter()
                .map(|file| {
                    (
                        file.handle,
                        file.relative_path.clone(),
                        file.size.clone(),
                        file.role.clone(),
                    )
                })
                .collect(),
        }
    }
}

struct CachedBag {
    key: SourceKey,
    reader: Arc<dyn BagReader>,
}

thread_local! {
    static ACTIVE_SOURCE: RefCell<Option<BrowserSource>> = const { RefCell::new(None) };
    static ACTIVE_EVENT_EMITTER: RefCell<Option<Function>> = const { RefCell::new(None) };
    static CACHED_BAG: RefCell<Option<CachedBag>> = const { RefCell::new(None) };
}

struct SourceGuard;
impl Drop for SourceGuard {
    fn drop(&mut self) { ACTIVE_SOURCE.with(|source| source.borrow_mut().take()); }
}

fn parse_source(text: &str) -> Result<BrowserSource> {
    let source: BrowserSource = serde_json::from_str(text).context("parse authorized source descriptor")?;
    ensure!(!source.source_id.is_empty(), "authorized source identity is empty");
    ensure!(source.source_revision > 0, "authorized source revision is invalid");
    ensure!(!source.files.is_empty(), "no authorized files");
    let mut names = std::collections::HashSet::new();
    let mut handles = std::collections::HashSet::new();
    for file in &source.files {
        ensure!(!file.relative_path.is_empty() && !file.relative_path.contains('\\') && !file.relative_path.starts_with('/') && file.relative_path.split('/').all(|part| !part.is_empty() && part != "." && part != ".."), "invalid authorized relative path");
        ensure!(names.insert(&file.relative_path), "duplicate authorized file path");
        ensure!(handles.insert(file.handle), "duplicate authorized file handle");
        file.size.parse::<u64>().context("invalid file size")?;
    }
    Ok(source)
}

fn bind_source(source: BrowserSource) -> Result<SourceGuard> {
    ACTIVE_SOURCE.with(|slot| {
        ensure!(slot.borrow().is_none(), "engine already has an active source");
        *slot.borrow_mut() = Some(source);
        Ok(SourceGuard)
    })
}

fn clear_cached_bag() {
    CACHED_BAG.with(|slot| {
        slot.borrow_mut().take();
    });
}

#[wasm_bindgen]
pub fn invalidate_source_cache() {
    ACTIVE_SOURCE.with(|slot| {
        slot.borrow_mut().take();
    });
    clear_cached_bag();
}

pub(crate) fn preview_enabled() -> bool {
    ACTIVE_SOURCE.with(|slot| {
        slot.borrow()
            .as_ref()
            .is_none_or(|source| source.preview_enabled)
    })
}

struct EventEmitterGuard;
impl Drop for EventEmitterGuard {
    fn drop(&mut self) {
        ACTIVE_EVENT_EMITTER.with(|emitter| emitter.borrow_mut().take());
    }
}

fn bind_event_emitter(emit: &Function) -> Result<EventEmitterGuard> {
    ACTIVE_EVENT_EMITTER.with(|slot| {
        ensure!(slot.borrow().is_none(), "engine already has an active event emitter");
        *slot.borrow_mut() = Some(emit.clone());
        Ok(EventEmitterGuard)
    })
}
fn deliver_event(emitter: &Function, mut payload: Value, pixels: Option<Vec<u8>>) -> Result<()> {
    precise_json(&mut payload);
    let text = serde_json::to_string(&payload)?;
    let result = if let Some(pixels) = pixels {
        // `from` copies into a standalone JavaScript ArrayBuffer. In particular, this must
        // never expose a view of shared WASM memory, which cannot be transferred.
        let pixels = Uint8Array::from(pixels.as_slice());
        emitter.call2(&JsValue::NULL, &JsValue::from_str(&text), pixels.as_ref())
    } else {
        emitter.call2(
            &JsValue::NULL,
            &JsValue::from_str(&text),
            &JsValue::UNDEFINED,
        )
    };
    result
        .map(|_| ())
        .map_err(|error| anyhow::anyhow!("event delivery failed: {error:?}"))
}


pub(crate) fn emit_imu_progress(progress: crate::input::LatestImu) -> Result<()> {
    let payload = json!({"kind":"imuProgress", "payload":progress});
    ACTIVE_EVENT_EMITTER.with(|slot| {
        let emitter = slot.borrow();
        let emitter = emitter
            .as_ref()
            .context("browser event emitter is not active")?;
        deliver_event(emitter, payload, None)
    })
}

struct BrowserReadSource { handle: u32, length: u64 }
impl RandomAccessSource for BrowserReadSource {
    fn len(&self) -> u64 { self.length }
    fn read_at(&self, offset: u64, target: &mut [u8]) -> io::Result<()> {
        let end = offset.checked_add(target.len() as u64).ok_or_else(|| io::Error::other("file range overflow"))?;
        if end > self.length { return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "file range exceeds authorized length")); }
        // Bound JS temporary allocations while preserving the parser's caller-owned buffer.
        for (index, chunk) in target.chunks_mut(1024 * 1024).enumerate() {
            let at = offset + (index as u64) * 1024 * 1024;
            let bytes = read_range(self.handle, at, chunk.len() as u32)
                .map_err(|error| io::Error::other(format!("browser file read failed: {error:?}")))?;
            if bytes.length() as usize != chunk.len() { return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "browser file read was short")); }
            bytes.copy_to(chunk);
        }
        Ok(())
    }
}

struct SharedBagReader {
    inner: Arc<dyn BagReader>,
}

impl BagReader for SharedBagReader {
    fn topics(&self) -> &[TopicInfo] {
        self.inner.topics()
    }

    fn visit(
        &self,
        filter: &MessageFilter,
        visitor: &mut dyn FnMut(RawMessage<'_>) -> std::result::Result<Visit, BagError>,
    ) -> std::result::Result<(), BagError> {
        self.inner.visit(filter, visitor)
    }
}

pub(crate) fn open_bag(config: &BagSourceConfig) -> std::result::Result<Box<dyn BagReader>, BagError> {
    ACTIVE_SOURCE.with(|slot| -> std::result::Result<Box<dyn BagReader>, BagError> {
        let source = slot.borrow();
        let source = source.as_ref().ok_or_else(|| BagError::InvalidPayload("no authorized source bound".to_owned()))?;
        if let Some(reader) = CACHED_BAG.with(|cache| {
            cache
                .borrow()
                .as_ref()
                .filter(|cached| cached.key.matches(source))
                .map(|cached| Arc::clone(&cached.reader))
        }) {
            return Ok(Box::new(SharedBagReader { inner: reader }));
        }
        clear_cached_bag();
        let key = SourceKey::from_source(source);
        let mut files = Vec::new();
        for file in &source.files {
            if file.role.as_deref().is_some_and(|role| role != "data") { continue; }
            let length = file.size.parse::<u64>().map_err(|error| BagError::InvalidPayload(error.to_string()))?;
            files.push(SourceFile::new(file.relative_path.clone(), Arc::new(BrowserReadSource { handle: file.handle, length }))?);
        }
        let reader: Arc<dyn BagReader> = Arc::from(ccl_rosbag::open_source(
            SourceCollection::new(files)?,
            OpenOptions::new(config.format.core()),
        )?);
        CACHED_BAG.with(|cache| {
            *cache.borrow_mut() = Some(CachedBag { key, reader: Arc::clone(&reader) });
        });
        Ok(Box::new(SharedBagReader { inner: reader }))
    })
}

pub(crate) fn read_authorized_file(path: &Path) -> Result<Vec<u8>> {
    ACTIVE_SOURCE.with(|slot| {
        let source = slot.borrow();
        let source = source.as_ref().context("no authorized source bound")?;
        let name = path.to_str().context("file path is not UTF-8")?;
        let file = source.files.iter().find(|file| file.relative_path == name).context("auxiliary file is not authorized")?;
        let length = file.size.parse::<u64>()?;
        ensure!(length <= 16 * 1024 * 1024, "auxiliary configuration exceeds 16 MiB budget");
        let mut bytes = vec![0; length as usize];
        BrowserReadSource { handle: file.handle, length }.read_at(0, &mut bytes)?;
        Ok(bytes)
    })
}

fn parse_config(text: &str) -> Result<RunSettings> {
    let config: RunSettings = serde_yaml::from_str(text).context("parse RunSettings YAML")?;
    config.validate()?;
    Ok(config)
}

fn js_error(error: anyhow::Error) -> JsValue { JsValue::from_str(&format!("{error:#}")) }

// Preserve all integer values outside JavaScript's exact range before crossing the JSON ABI.
fn precise_json(value: &mut Value) {
    match value {
        Value::Number(number) if number.as_i64().is_some_and(|n| n.unsigned_abs() > 9_007_199_254_740_991) || number.as_u64().is_some_and(|n| n > 9_007_199_254_740_991) => { *value = Value::String(number.to_string()); }
        Value::Object(map) => for (key, value) in map {
            if (key.ends_with("timestamp_ns") || key.ends_with("time_ns")) && value.is_number() { *value = Value::String(value.to_string()); }
            else { precise_json(value); }
        },
        Value::Array(values) => for value in values { precise_json(value); },
        _ => {}
    }
}

fn serialize<T: Serialize>(value: &T) -> Result<String> {
    let mut value = serde_json::to_value(value)?;
    precise_json(&mut value);
    Ok(serde_json::to_string(&value)?)
}

#[wasm_bindgen]
pub fn engine_version() -> String {
    let profile = if cfg!(feature = "wasm-threads") { "cpu-rayon" } else { "cpu-single" };
    format!("4p/{};{profile}", env!("CARGO_PKG_VERSION"))
}

#[wasm_bindgen]
pub fn validate_config(text: &str) -> std::result::Result<String, JsValue> {
    (|| { let config = parse_config(text)?; serialize(&json!({"config": config, "yaml": serde_yaml::to_string(&config)?})) })().map_err(js_error)
}

#[wasm_bindgen]
pub fn inspect_source(descriptor: &str) -> std::result::Result<String, JsValue> {
    let result = (|| {
        let source = parse_source(descriptor)?;
        let format = source.format;
        let _guard = bind_source(source)?;
        let config = BagSourceConfig { path: "authorized-source".into(), format, imu_topic: None, synchronization: Default::default() };
        serialize(&crate::input::inspect(&config)?)
    })();
    if result.is_err() { clear_cached_bag(); }
    result.map_err(js_error)
}

#[wasm_bindgen]
pub fn run_calibration(text: &str, descriptor: &str, emit: &Function) -> std::result::Result<String, JsValue> {
    let result = (|| {
        let source = parse_source(descriptor)?;
        let format = source.format;
        let _guard = bind_source(source)?;
        let _event_guard = bind_event_emitter(emit)?;
        let config = parse_config(text)?;
        ensure!(std::mem::discriminant(&config.source.format) == std::mem::discriminant(&format), "configuration format differs from authorized source");
        let now = now_millis();
        ensure!(now.is_finite(), "browser clock is invalid");
        let started = chrono::DateTime::from_timestamp_millis(now as i64).context("browser clock outside supported range")?;
        let run = RunContext::memory(started)?;
        let callback_error = RefCell::new(None);
        let delivery_failed = std::sync::atomic::AtomicBool::new(false);
        let mut event = |event| {
            let result = (|| -> Result<()> {
                let (payload, pixels) = match event {
                    Event::Stage(value) => (json!({"kind":"stage", "payload":value}), None),
                    Event::DetectionProgress(mut value) => {
                        // Move the byte vector out before serialization. The JSON callback argument
                        // retains preview metadata, while the independent Uint8Array carries pixels.
                        let pixels = value
                            .preview
                            .as_mut()
                            .map(|preview| std::mem::take(&mut preview.pixels));
                        let mut value = serde_json::to_value(value)?;
                        if let Some(preview) = value
                            .get_mut("preview")
                            .and_then(Value::as_object_mut)
                        {
                            preview.remove("pixels");
                        }
                        let mut payload = json!({"kind":"detectionProgress", "payload":null});
                        payload["payload"] = value;
                        (payload, pixels)
                    }
                    Event::Dataset(value) => (json!({"kind":"dataset", "payload":value}), None),
                    Event::SolveProgress(value) => (json!({"kind":"solveProgress", "payload":{
                        "iteration":value.iteration,"max_iterations":value.max_iterations,"cost":value.cost,
                        "damping":value.damping,"scaled_gradient_inf_norm":value.scaled_gradient_inf_norm,
                        "accepted_steps":value.accepted_steps,"rejected_steps":value.rejected_steps
                    }}), None),
                    Event::SolveActivity(value) => (json!({"kind":"solveActivity", "payload":{
                        "phase":format!("{:?}",value.phase),"completed":value.completed,"total":value.total,
                        "iteration":value.iteration,"cancellable":value.cancellable
                    }}), None),
                    Event::SolveSummary(value) => (json!({"kind":"solveSummary", "payload":value}), None),
                    Event::Message(value) => (json!({"kind":"message", "payload":value}), None),
                };
                deliver_event(emit, payload, pixels)
            })();
            if let Err(error) = result {
                delivery_failed.store(true, std::sync::atomic::Ordering::Relaxed);
                *callback_error.borrow_mut() = Some(error);
            }
        };
        let output = crate::runner::execute(&config, &run, &|| delivery_failed.load(std::sync::atomic::Ordering::Relaxed) || CANCELLED.load(std::sync::atomic::Ordering::Relaxed) != 0, &mut event);
        if let Some(error) = callback_error.into_inner() { return Err(error); }
        let output = output?;
        let ended = chrono::DateTime::from_timestamp_millis(now_millis() as i64).context("browser end time outside supported range")?;
        run.finish_at(RunStatus::Complete, None, ended)?;
        let artifacts = run.take_artifacts()?.into_iter().map(|(path, bytes)| {
            let media_type = match path.extension().and_then(|ext| ext.to_str()) { Some("json" | "jsonl") => "application/json", Some("html") => "text/html", _ => "text/plain" };
            Ok(json!({"path":path,"mediaType":media_type,"text":String::from_utf8(bytes)?}))
        }).collect::<Result<Vec<_>>>()?;
        serialize(&json!({"result":output,"artifacts":artifacts}))
    })();
    if result.is_err() { clear_cached_bag(); }
    result.map_err(js_error)
}

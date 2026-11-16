use std::collections::BTreeMap;
use std::fs::{self, File, OpenOptions};
use std::io::{Error, ErrorKind, Write};
use std::path::{Component, Path, PathBuf};
use std::sync::{Arc, Mutex};

use anyhow::{Context, Result};
use chrono::{DateTime, SecondsFormat, Utc};
use serde::{Serialize, de::DeserializeOwned};

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RunStatus {
    InProgress,
    Complete,
    Failed,
    Cancelled,
    ClosedWithoutCalibration,
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct RunMetadata {
    schema_version: u32,
    started_at: DateTime<Utc>,
    ended_at: Option<DateTime<Utc>>,
    status: RunStatus,
    mode: Option<String>,
    application_version: String,
    corelib: CorelibProvenance,
    error_category: Option<String>,
    stage_duration_ms: BTreeMap<String, u64>,
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct CorelibProvenance {
    distribution: &'static str,
    workspace_version: &'static str,
    git_commit: &'static str,
    target: &'static str,
    rustc_release: &'static str,
    artifact_sha256: &'static str,
    profile: &'static str,
}

impl CorelibProvenance {
    const fn current() -> Self {
        Self {
            distribution: "corp_registry_with_local_ccl_image_patch",
            workspace_version: "ccl_calib=0.6.0,ccl_geometry=0.1.0,ccl_image=0.3.0+unpublished-workspace,ccl_rosbag=0.5.0",
            git_commit: "unpublished_worktree",
            target: if cfg!(target_arch = "wasm32") {
                "wasm32_browser"
            } else {
                "cargo_host"
            },
            rustc_release: "cargo_toolchain",
            artifact_sha256: "unrecorded; browser artifact hashes are in engine manifest",
            profile: "cargo_local_patch",
        }
    }
}

struct RunDirectory {
    path: PathBuf,
    sequence: u32,
}

enum RunStorage {
    Native {
        root: Mutex<RunDirectory>,
        log: Mutex<Option<File>>,
    },
    Memory {
        artifacts: Mutex<MemoryArtifacts>,
    },
}

struct MemoryArtifacts {
    files: BTreeMap<PathBuf, Vec<u8>>,
    total_bytes: usize,
}

const MAX_MEMORY_ARTIFACT_BYTES: usize = 256 * 1024 * 1024;
const MAX_MEMORY_ARTIFACTS: usize = 128;

#[derive(Clone)]
pub struct RunContext {
    stamp: String,
    storage: Arc<RunStorage>,
    metadata: Arc<Mutex<RunMetadata>>,
}

impl RunContext {
    pub fn create() -> Result<Self> {
        Self::create_in(Path::new("runs"), platform_now())
    }

    /// Creates an in-memory run without consulting a platform clock or filesystem.
    ///
    /// The timestamp is supplied by the browser host and is also used for the initial
    /// log entry. Artifacts are retained up to a conservative per-run output bound.
    pub fn memory(started_at: DateTime<Utc>) -> Result<Self> {
        let context = Self {
            stamp: format_run_stamp(started_at),
            storage: Arc::new(RunStorage::Memory {
                artifacts: Mutex::new(MemoryArtifacts {
                    files: BTreeMap::new(),
                    total_bytes: 0,
                }),
            }),
            metadata: Arc::new(Mutex::new(new_metadata(started_at))),
        };
        context.persist_metadata()?;
        context.log_at(started_at, "INFO", "run record created in memory")?;
        Ok(context)
    }

    /// Create a fresh, unconfigured run beside this one without touching its
    /// results. The new run starts in the same parent directory, with its own
    /// timestamp and log; same-second runs get a numeric sequence suffix.
    pub fn next_run(&self) -> Result<Self> {
        let RunStorage::Native { .. } = self.storage.as_ref() else {
            anyhow::bail!("an in-memory run requires an explicit timestamp")
        };
        let current = self.root();
        let parent = current
            .parent()
            .with_context(|| format!("{} has no parent directory", current.display()))?;
        Self::create_in(parent, platform_now())
    }

    fn create_in(parent: &Path, started_at: DateTime<Utc>) -> Result<Self> {
        fs::create_dir_all(parent).context("create runs directory")?;
        let stamp = format_run_stamp(started_at);
        let (sequence, path) = create_unconfigured_dir(parent, &stamp)?;
        let log = OpenOptions::new()
            .create_new(true)
            .append(true)
            .open(path.join("run.log"))
            .with_context(|| format!("create {}/run.log", path.display()))?;
        let context = Self {
            stamp,
            storage: Arc::new(RunStorage::Native {
                root: Mutex::new(RunDirectory { path, sequence }),
                log: Mutex::new(Some(log)),
            }),
            metadata: Arc::new(Mutex::new(new_metadata(started_at))),
        };
        context.persist_metadata()?;
        context.log("INFO", "run directory created")?;
        Ok(context)
    }

    /// Returns the native run directory.
    ///
    /// In-memory callers must use [`Self::read`], [`Self::write`], and
    /// [`Self::take_artifacts`] instead of manufacturing filesystem paths.
    #[must_use]
    pub fn root(&self) -> PathBuf {
        let RunStorage::Native { root, .. } = self.storage.as_ref() else {
            panic!("in-memory run has no filesystem root")
        };
        root.lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .path
            .clone()
    }

    pub fn set_mode(&self, mode: &str) -> Result<()> {
        ensure_mode_suffix(mode)?;
        let same_mode = self.lock_metadata()?.mode.as_deref() == Some(mode);
        if same_mode {
            return self.persist_metadata();
        }
        let RunStorage::Native { root, log } = self.storage.as_ref() else {
            self.lock_metadata()?.mode = Some(mode.to_owned());
            return self.persist_metadata();
        };
        let (current, mut sequence) = {
            let directory = root
                .lock()
                .map_err(|_| anyhow::anyhow!("run root lock poisoned"))?;
            (directory.path.clone(), directory.sequence)
        };
        let parent = current
            .parent()
            .with_context(|| format!("{} has no parent directory", current.display()))?;
        let mut log = log
            .lock()
            .map_err(|_| anyhow::anyhow!("run log lock poisoned"))?;
        let file = log
            .take()
            .context("run log file is unavailable during rename")?;
        drop(file);
        // Never replace an existing directory: take the next free sequence
        // number instead, so a run created or renamed in the same second keeps
        // its results. rename itself still refuses a non-empty destination on
        // both Unix and Windows, which covers a destination that appears
        // between the check and the rename.
        let desired = loop {
            let candidate = run_dir_path(parent, &self.stamp, sequence, mode);
            if candidate == current {
                break candidate;
            }
            match fs::symlink_metadata(&candidate) {
                Ok(_) => {}
                Err(error) if error.kind() == ErrorKind::NotFound => {
                    match fs::rename(&current, &candidate) {
                        Ok(()) => break candidate,
                        Err(error)
                            if error.kind() == ErrorKind::AlreadyExists
                                || error.kind() == ErrorKind::DirectoryNotEmpty => {}
                        Err(error) => {
                            reopen_log(&mut log, &current)?;
                            return Err(error).with_context(|| {
                                format!("rename {} to {}", current.display(), candidate.display())
                            });
                        }
                    }
                }
                Err(error) => {
                    reopen_log(&mut log, &current)?;
                    return Err(error).with_context(|| format!("inspect {}", candidate.display()));
                }
            }
            sequence = if let Some(next) = sequence.checked_add(1) {
                next
            } else {
                reopen_log(&mut log, &current)?;
                anyhow::bail!("run directory sequence overflow for {}-{mode}", self.stamp);
            };
        };
        {
            let mut directory = root
                .lock()
                .map_err(|_| anyhow::anyhow!("run root lock poisoned"))?;
            directory.path.clone_from(&desired);
            directory.sequence = sequence;
        }
        *log = Some(
            OpenOptions::new()
                .append(true)
                .open(desired.join("run.log"))
                .context("reopen renamed run log")?,
        );
        self.lock_metadata()?.mode = Some(mode.to_owned());
        self.persist_metadata()
    }

    pub fn record_stage_duration(&self, stage: &str, milliseconds: u64) -> Result<()> {
        self.lock_metadata()?
            .stage_duration_ms
            .insert(stage.to_owned(), milliseconds);
        self.persist_metadata()
    }

    pub fn snapshot_config(&self, source: &Path) -> Result<()> {
        anyhow::ensure!(
            matches!(self.storage.as_ref(), RunStorage::Native { .. }),
            "snapshot_config requires native filesystem storage"
        );
        let bytes = fs::read(source).with_context(|| format!("read {}", source.display()))?;
        self.write(Path::new("config.yaml"), &bytes)
    }

    pub fn write_yaml<T: Serialize>(&self, relative: &Path, value: &T) -> Result<()> {
        let bytes = serde_yaml::to_string(value).context("serialize yaml")?;
        self.write(relative, bytes.as_bytes())
    }

    /// Opens a new artifact for streaming output, truncating an existing artifact.
    pub fn create_writer(&self, relative: &Path) -> Result<Box<dyn Write>> {
        validate_relative(relative)?;
        match self.storage.as_ref() {
            RunStorage::Native { .. } => {
                let path = self.root().join(relative);
                let parent = path
                    .parent()
                    .with_context(|| format!("{} has no parent", path.display()))?;
                fs::create_dir_all(parent)
                    .with_context(|| format!("create {}", parent.display()))?;
                let file = File::create(&path)
                    .with_context(|| format!("create {}", path.display()))?;
                Ok(Box::new(file))
            }
            RunStorage::Memory { artifacts } => {
                let relative = relative.to_path_buf();
                artifacts
                    .lock()
                    .map_err(|_| anyhow::anyhow!("memory artifact lock poisoned"))?
                    .insert(relative.clone(), Vec::new())?;
                Ok(Box::new(MemoryArtifactWriter {
                    storage: Arc::clone(&self.storage),
                    relative,
                }))
            }
        }
    }

    /// Reads one run-relative artifact.
    pub fn read(&self, relative: &Path) -> Result<Vec<u8>> {
        validate_relative(relative)?;
        match self.storage.as_ref() {
            RunStorage::Native { .. } => {
                let path = self.root().join(relative);
                fs::read(&path).with_context(|| format!("read {}", path.display()))
            }
            RunStorage::Memory { artifacts } => artifacts
                .lock()
                .map_err(|_| anyhow::anyhow!("memory artifact lock poisoned"))?
                .files
                .get(relative)
                .cloned()
                .with_context(|| format!("read missing artifact {}", relative.display())),
        }
    }

    /// Atomically replaces one run-relative artifact.
    pub fn write(&self, relative: &Path, bytes: &[u8]) -> Result<()> {
        validate_relative(relative)?;
        match self.storage.as_ref() {
            RunStorage::Native { .. } => atomic_write(&self.root().join(relative), bytes),
            RunStorage::Memory { artifacts } => artifacts
                .lock()
                .map_err(|_| anyhow::anyhow!("memory artifact lock poisoned"))?
                .replace_from_slice(relative, bytes),
        }
    }

    /// Publishes a complete artifact directory as one operation.
    ///
    /// Native storage keeps the existing temporary-directory rename protocol.
    /// Memory storage validates the complete batch before changing the artifact map.
    pub fn publish(
        &self,
        relative: &Path,
        artifacts: BTreeMap<PathBuf, Vec<u8>>,
    ) -> Result<()> {
        validate_relative(relative)?;
        anyhow::ensure!(!artifacts.is_empty(), "cannot publish an empty artifact set");
        for path in artifacts.keys() {
            validate_relative(path)?;
        }
        match self.storage.as_ref() {
            RunStorage::Native { .. } => self.publish_native(relative, artifacts),
            RunStorage::Memory { artifacts: stored } => {
                let mut stored = stored
                    .lock()
                    .map_err(|_| anyhow::anyhow!("memory artifact lock poisoned"))?;
                anyhow::ensure!(
                    !stored.files.keys().any(|path| path.starts_with(relative)),
                    "artifact directory {} already exists",
                    relative.display()
                );
                stored.insert_batch(relative, artifacts)
            }
        }
    }

    /// Removes and returns all artifacts from an in-memory run without copying bytes.
    pub fn take_artifacts(&self) -> Result<BTreeMap<PathBuf, Vec<u8>>> {
        let RunStorage::Memory { artifacts } = self.storage.as_ref() else {
            anyhow::bail!("native run artifacts remain in their run directory")
        };
        let mut artifacts = artifacts
            .lock()
            .map_err(|_| anyhow::anyhow!("memory artifact lock poisoned"))?;
        artifacts.total_bytes = 0;
        Ok(std::mem::take(&mut artifacts.files))
    }

    pub fn log(&self, level: &str, message: &str) -> Result<()> {
        self.log_at(platform_now(), level, message)
    }

    fn log_at(&self, at: DateTime<Utc>, level: &str, message: &str) -> Result<()> {
        let stamp = at.to_rfc3339_opts(SecondsFormat::Millis, true);
        let normalized = message.replace('\n', "\\n");
        let line = format!("{stamp} {level} {normalized}\n");
        match self.storage.as_ref() {
            RunStorage::Native { log, .. } => {
                let mut log = log
                    .lock()
                    .map_err(|_| anyhow::anyhow!("run log lock poisoned"))?;
                let file = log.as_mut().context("run log file is unavailable")?;
                file.write_all(line.as_bytes()).context("append run log")?;
                file.flush().context("flush run log")
            }
            RunStorage::Memory { artifacts } => artifacts
                .lock()
                .map_err(|_| anyhow::anyhow!("memory artifact lock poisoned"))?
                .append(Path::new("run.log"), line.as_bytes()),
        }
    }

    pub fn finish(&self, status: RunStatus, error_category: Option<&str>) -> Result<()> {
        self.finish_at(status, error_category, platform_now())
    }

    /// Finalizes a run at a host-supplied timestamp.
    pub fn finish_at(
        &self,
        status: RunStatus,
        error_category: Option<&str>,
        ended_at: DateTime<Utc>,
    ) -> Result<()> {
        {
            let mut metadata = self.lock_metadata()?;
            metadata.status = status;
            metadata.ended_at = Some(ended_at);
            metadata.error_category = error_category.map(str::to_owned);
        }
        self.persist_metadata()?;
        self.log_at(
            ended_at,
            "INFO",
            &format!("run finalized with status {status:?}"),
        )
    }

    fn publish_native(
        &self,
        relative: &Path,
        artifacts: BTreeMap<PathBuf, Vec<u8>>,
    ) -> Result<()> {
        let root = self.root();
        let final_path = root.join(relative);
        anyhow::ensure!(!final_path.exists(), "{} already exists", final_path.display());
        let file_name = relative
            .file_name()
            .and_then(|name| name.to_str())
            .context("non-UTF-8 artifact directory name")?;
        let temporary = final_path.with_file_name(format!(".{file_name}.tmp"));
        if temporary.exists() {
            fs::remove_dir_all(&temporary)
                .with_context(|| format!("remove stale {}", temporary.display()))?;
        }
        fs::create_dir(&temporary)
            .with_context(|| format!("create {}", temporary.display()))?;
        let result = (|| {
            for (path, bytes) in artifacts {
                atomic_write(&temporary.join(path), &bytes)?;
            }
            fs::rename(&temporary, &final_path)
                .with_context(|| format!("publish {}", final_path.display()))?;
            Ok(())
        })();
        if result.is_err() {
            let _ = fs::remove_dir_all(&temporary);
        }
        result
    }

    fn lock_metadata(&self) -> Result<std::sync::MutexGuard<'_, RunMetadata>> {
        self.metadata
            .lock()
            .map_err(|_| anyhow::anyhow!("run metadata lock poisoned"))
    }

    fn persist_metadata(&self) -> Result<()> {
        let yaml = {
            let metadata = self.lock_metadata()?;
            serde_yaml::to_string(&*metadata).context("serialize run metadata")?
        };
        self.write(Path::new("run.yaml"), yaml.as_bytes())
    }
}

impl MemoryArtifacts {
    fn insert(&mut self, path: PathBuf, bytes: Vec<u8>) -> Result<()> {
        let total_bytes = self.replacement_total(&path, bytes.len())?;
        self.files.insert(path, bytes);
        self.total_bytes = total_bytes;
        Ok(())
    }

    fn replace_from_slice(&mut self, path: &Path, bytes: &[u8]) -> Result<()> {
        let total_bytes = self.replacement_total(path, bytes.len())?;
        let bytes = bytes.to_vec();
        if let Some(previous) = self.files.get_mut(path) {
            *previous = bytes;
        } else {
            self.files.insert(path.to_path_buf(), bytes);
        }
        self.total_bytes = total_bytes;
        Ok(())
    }

    fn replacement_total(&self, path: &Path, byte_len: usize) -> Result<usize> {
        let previous_len = self.files.get(path).map_or(0, Vec::len);
        let artifact_count = if self.files.contains_key(path) {
            self.files.len()
        } else {
            self.files.len() + 1
        };
        anyhow::ensure!(
            artifact_count <= MAX_MEMORY_ARTIFACTS,
            "memory artifact count exceeds {MAX_MEMORY_ARTIFACTS}"
        );
        let total_bytes = self
            .total_bytes
            .checked_sub(previous_len)
            .and_then(|total| total.checked_add(byte_len))
            .context("memory artifact size overflow")?;
        anyhow::ensure!(
            total_bytes <= MAX_MEMORY_ARTIFACT_BYTES,
            "memory artifacts exceed {} bytes",
            MAX_MEMORY_ARTIFACT_BYTES
        );
        Ok(total_bytes)
    }

    fn append(&mut self, path: &Path, bytes: &[u8]) -> Result<()> {
        let total_bytes = self
            .total_bytes
            .checked_add(bytes.len())
            .context("memory artifact size overflow")?;
        anyhow::ensure!(
            total_bytes <= MAX_MEMORY_ARTIFACT_BYTES,
            "memory artifacts exceed {} bytes",
            MAX_MEMORY_ARTIFACT_BYTES
        );
        if !self.files.contains_key(path) {
            anyhow::ensure!(
                self.files.len() < MAX_MEMORY_ARTIFACTS,
                "memory artifact count exceeds {MAX_MEMORY_ARTIFACTS}"
            );
        }
        self.files.entry(path.to_path_buf()).or_default().extend_from_slice(bytes);
        self.total_bytes = total_bytes;
        Ok(())
    }

    fn insert_batch(
        &mut self,
        directory: &Path,
        artifacts: BTreeMap<PathBuf, Vec<u8>>,
    ) -> Result<()> {
        let added_bytes = artifacts.values().try_fold(0usize, |total, bytes| {
            total.checked_add(bytes.len()).context("memory artifact size overflow")
        })?;
        let total_bytes = self
            .total_bytes
            .checked_add(added_bytes)
            .context("memory artifact size overflow")?;
        anyhow::ensure!(
            self.files.len() + artifacts.len() <= MAX_MEMORY_ARTIFACTS,
            "memory artifact count exceeds {MAX_MEMORY_ARTIFACTS}"
        );
        anyhow::ensure!(
            total_bytes <= MAX_MEMORY_ARTIFACT_BYTES,
            "memory artifacts exceed {} bytes",
            MAX_MEMORY_ARTIFACT_BYTES
        );
        self.files.extend(
            artifacts
                .into_iter()
                .map(|(path, bytes)| (directory.join(path), bytes)),
        );
        self.total_bytes = total_bytes;
        Ok(())
    }
}

struct MemoryArtifactWriter {
    storage: Arc<RunStorage>,
    relative: PathBuf,
}

impl Write for MemoryArtifactWriter {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        let RunStorage::Memory { artifacts } = self.storage.as_ref() else {
            return Err(Error::other("memory artifact writer has native storage"));
        };
        let mut artifacts = artifacts
            .lock()
            .map_err(|_| Error::other("memory artifact lock poisoned"))?;
        artifacts
            .append(&self.relative, bytes)
            .map_err(|error| Error::other(format!("{error:#}")))?;
        Ok(bytes.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

fn validate_relative(path: &Path) -> Result<()> {
    anyhow::ensure!(!path.as_os_str().is_empty(), "artifact path is empty");
    anyhow::ensure!(
        path.components().all(|component| matches!(component, Component::Normal(_))),
        "artifact path must be normalized and relative: {}",
        path.display()
    );
    Ok(())
}

fn new_metadata(started_at: DateTime<Utc>) -> RunMetadata {
    RunMetadata {
        schema_version: 1,
        started_at,
        ended_at: None,
        status: RunStatus::InProgress,
        mode: None,
        application_version: env!("CARGO_PKG_VERSION").to_owned(),
        corelib: CorelibProvenance::current(),
        error_category: None,
        stage_duration_ms: BTreeMap::new(),
    }
}

fn platform_now() -> DateTime<Utc> {
    Utc::now()
}

fn format_run_stamp(started_at: DateTime<Utc>) -> String {
    started_at.format("%Y%m%d-%H%M%S").to_string()
}

/// Name of the directory for the run with `sequence` (1-based) and `label`:
/// `<stamp>-<label>` for sequence 1, otherwise `<stamp>-<sequence>-<label>`.
fn run_dir_path(parent: &Path, stamp: &str, sequence: u32, label: &str) -> PathBuf {
    if sequence == 1 {
        parent.join(format!("{stamp}-{label}"))
    } else {
        parent.join(format!("{stamp}-{sequence}-{label}"))
    }
}

/// Exclusively create the initial unconfigured directory. The sequence is
/// retained by the run and reused for mode renames, so same-second runs never
/// share a name. Allocation starts above every directory already using the
/// stamp (configured or not); only `AlreadyExists` advances the sequence, and
/// any other failure is reported immediately.
fn create_unconfigured_dir(parent: &Path, stamp: &str) -> Result<(u32, PathBuf)> {
    let mut sequence = next_sequence(parent, stamp)?;
    loop {
        let path = run_dir_path(parent, stamp, sequence, "unconfigured");
        match fs::create_dir(&path) {
            Ok(()) => return Ok((sequence, path)),
            Err(error) if error.kind() == ErrorKind::AlreadyExists => {
                sequence = sequence
                    .checked_add(1)
                    .context("run directory suffix overflow")?;
            }
            Err(error) => {
                return Err(error)
                    .with_context(|| format!("create exclusive run directory {}", path.display()));
            }
        }
    }
}

/// One past the highest sequence visible among `parent` entries named with
/// `stamp`. A name carries a sequence when the leading component after the
/// stamp parses as a number and a label follows it
/// (`<stamp>-2-unconfigured`, `<stamp>-2-single_camera`); every other name is
/// sequence 1.
fn next_sequence(parent: &Path, stamp: &str) -> Result<u32> {
    let prefix = format!("{stamp}-");
    let mut highest = 0u32;
    for entry in fs::read_dir(parent).with_context(|| format!("read {}", parent.display()))? {
        let entry = entry.with_context(|| format!("read entry in {}", parent.display()))?;
        let file_name = entry.file_name();
        let Some(name) = file_name.to_str() else {
            continue;
        };
        let Some(rest) = name.strip_prefix(&prefix) else {
            continue;
        };
        let sequence = rest
            .split_once('-')
            .and_then(|(head, _)| head.parse::<u32>().ok())
            .unwrap_or(1);
        highest = highest.max(sequence);
    }
    highest
        .checked_add(1)
        .context("run directory sequence overflow")
}

fn reopen_log(log: &mut Option<File>, root: &Path) -> Result<()> {
    *log = Some(
        OpenOptions::new()
            .append(true)
            .open(root.join("run.log"))
            .with_context(|| format!("reopen run log in {}", root.display()))?,
    );
    Ok(())
}

fn ensure_mode_suffix(mode: &str) -> Result<()> {
    anyhow::ensure!(
        !mode.is_empty()
            && mode
                .bytes()
                .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_'),
        "invalid calibration mode for run directory: {mode}"
    );
    Ok(())
}

pub fn atomic_write(path: &Path, bytes: &[u8]) -> Result<()> {
    let parent = path
        .parent()
        .with_context(|| format!("{} has no parent", path.display()))?;
    fs::create_dir_all(parent).with_context(|| format!("create {}", parent.display()))?;
    let file_name = path
        .file_name()
        .and_then(|name| name.to_str())
        .context("non-UTF-8 output file name")?;
    let temporary = parent.join(format!(".{file_name}.tmp"));
    let mut file = OpenOptions::new()
        .create(true)
        .truncate(true)
        .write(true)
        .open(&temporary)
        .with_context(|| format!("create {}", temporary.display()))?;
    file.write_all(bytes)
        .with_context(|| format!("write {}", temporary.display()))?;
    file.sync_all()
        .with_context(|| format!("sync {}", temporary.display()))?;
    drop(file);
    fs::rename(&temporary, path)
        .with_context(|| format!("rename {} to {}", temporary.display(), path.display()))?;
    sync_directory(parent)?;
    Ok(())
}

#[cfg(unix)]
fn sync_directory(path: &Path) -> Result<()> {
    File::open(path)
        .with_context(|| format!("open directory {}", path.display()))?
        .sync_all()
        .with_context(|| format!("sync directory {}", path.display()))
}

#[cfg(not(unix))]
fn sync_directory(_path: &Path) -> Result<()> {
    Ok(())
}

pub fn read_yaml<T: DeserializeOwned>(path: &Path) -> Result<T> {
    let bytes = fs::read(path).with_context(|| format!("read {}", path.display()))?;
    serde_yaml::from_slice(&bytes).with_context(|| format!("parse {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    fn fixed_started_at() -> DateTime<Utc> {
        DateTime::parse_from_rfc3339("2026-10-16T09:28:58Z")
            .unwrap()
            .with_timezone(&Utc)
    }

    fn create_at(parent: &Path) -> RunContext {
        RunContext::create_in(parent, fixed_started_at()).expect("create run")
    }

    #[test]
    fn run_stamp_uses_full_date_and_hour_minute_second() {
        assert_eq!(format_run_stamp(fixed_started_at()), "20261016-092858");
    }

    #[test]
    fn same_stamp_creates_get_distinct_unconfigured_directories() {
        let parent = tempdir().unwrap();
        let first = create_at(parent.path());
        let second = create_at(parent.path());

        assert_eq!(
            first.root(),
            parent.path().join("20261016-092858-unconfigured")
        );
        assert_eq!(
            second.root(),
            parent.path().join("20261016-092858-2-unconfigured")
        );
        assert!(first.root().join("run.yaml").is_file());
        assert!(second.root().join("run.yaml").is_file());
    }

    #[test]
    fn same_stamp_mode_renames_keep_each_others_results() {
        let parent = tempdir().unwrap();
        let first = create_at(parent.path());
        first.set_mode("single_camera").unwrap();
        atomic_write(&first.root().join("result/calibration.yaml"), b"first").unwrap();

        let second = create_at(parent.path());
        second.set_mode("single_camera").unwrap();

        assert_eq!(
            first.root(),
            parent.path().join("20261016-092858-single_camera")
        );
        assert_eq!(
            second.root(),
            parent.path().join("20261016-092858-2-single_camera")
        );
        assert_eq!(
            fs::read(first.root().join("result/calibration.yaml")).unwrap(),
            b"first"
        );
        assert!(second.root().join("run.yaml").is_file());
    }

    #[test]
    fn set_mode_never_replaces_an_existing_directory() {
        let parent = tempdir().unwrap();
        let run = create_at(parent.path());
        let occupied = parent.path().join("20261016-092858-single_camera");
        fs::create_dir(&occupied).unwrap();
        atomic_write(&occupied.join("keep.txt"), b"keep").unwrap();

        run.set_mode("single_camera").unwrap();

        assert_eq!(
            run.root(),
            parent.path().join("20261016-092858-2-single_camera")
        );
        assert_eq!(fs::read(occupied.join("keep.txt")).unwrap(), b"keep");
    }

    #[test]
    fn next_run_creates_a_distinct_sibling_and_keeps_finished_results() {
        let parent = tempdir().unwrap();
        let first = create_at(parent.path());
        first.set_mode("multi_camera").unwrap();
        first.finish(RunStatus::Complete, None).unwrap();
        let original_root = first.root();
        atomic_write(&original_root.join("result/calibration.yaml"), b"original").unwrap();

        let second = first.next_run().unwrap();
        let second_root = second.root();

        assert_eq!(first.root(), original_root);
        assert_eq!(second_root.parent(), Some(parent.path()));
        assert_ne!(second_root, original_root);
        assert_eq!(
            fs::read(original_root.join("result/calibration.yaml")).unwrap(),
            b"original"
        );
        assert!(!second_root.join("result/calibration.yaml").exists());
        let metadata = read_yaml::<serde_yaml::Value>(&second_root.join("run.yaml")).unwrap();
        assert_eq!(metadata["mode"].as_str(), None);
        assert_eq!(metadata["status"].as_str(), Some("in_progress"));
    }
    #[test]
    fn memory_run_preserves_streamed_and_published_artifact_contents() {
        let started_at = fixed_started_at();
        let run = RunContext::memory(started_at).unwrap();
        run.set_mode("single_camera").unwrap();
        {
            let mut writer = run
                .create_writer(Path::new("detection_progress.jsonl"))
                .unwrap();
            writer.write_all(b"{\"frames\":1}\n").unwrap();
            writer.write_all(b"{\"frames\":2}\n").unwrap();
            writer.flush().unwrap();
        }
        run.write(Path::new("trajectory.json"), br#"{"samples":[1,2]}"#)
            .unwrap();
        run.publish(
            Path::new("result"),
            BTreeMap::from([
                (PathBuf::from("calibration.yaml"), b"mode: single_camera\n".to_vec()),
                (PathBuf::from("trajectory.json"), br#"{"samples":[1,2]}"#.to_vec()),
            ]),
        )
        .unwrap();
        run.finish_at(RunStatus::Complete, None, started_at).unwrap();

        let artifacts = run.take_artifacts().unwrap();
        assert_eq!(
            artifacts[Path::new("detection_progress.jsonl")],
            b"{\"frames\":1}\n{\"frames\":2}\n"
        );
        assert_eq!(
            artifacts[Path::new("result/calibration.yaml")],
            b"mode: single_camera\n"
        );
        assert_eq!(
            artifacts[Path::new("result/trajectory.json")],
            br#"{"samples":[1,2]}"#
        );
        let metadata: serde_yaml::Value =
            serde_yaml::from_slice(&artifacts[Path::new("run.yaml")]).unwrap();
        assert_eq!(metadata["mode"].as_str(), Some("single_camera"));
        assert_eq!(metadata["status"].as_str(), Some("complete"));
    }

    #[test]
    fn failed_memory_publication_leaves_the_complete_result_unchanged() {
        let started_at = fixed_started_at();
        let run = RunContext::memory(started_at).unwrap();
        run.publish(
            Path::new("result"),
            BTreeMap::from([
                (PathBuf::from("calibration.yaml"), b"complete-result".to_vec()),
                (PathBuf::from("summary.json"), b"complete-summary".to_vec()),
            ]),
        )
        .unwrap();

        let replacement = BTreeMap::from([
            (PathBuf::from("calibration.yaml"), b"partial-replacement".to_vec()),
            (PathBuf::from("late.json"), b"must-not-appear".to_vec()),
        ]);
        assert!(run.publish(Path::new("result"), replacement).is_err());
        run.finish_at(RunStatus::Failed, Some("export"), started_at)
            .unwrap();

        let artifacts = run.take_artifacts().unwrap();
        assert_eq!(
            artifacts[Path::new("result/calibration.yaml")],
            b"complete-result"
        );
        assert_eq!(
            artifacts[Path::new("result/summary.json")],
            b"complete-summary"
        );
        assert!(!artifacts.contains_key(Path::new("result/late.json")));
        let metadata: serde_yaml::Value =
            serde_yaml::from_slice(&artifacts[Path::new("run.yaml")]).unwrap();
        assert_eq!(metadata["status"].as_str(), Some("failed"));
        assert_eq!(metadata["error_category"].as_str(), Some("export"));
    }
}

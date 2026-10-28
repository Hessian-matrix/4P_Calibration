use std::collections::BTreeMap;
use std::fs::{self, File, OpenOptions};
use std::io::{ErrorKind, Write};
use std::path::{Path, PathBuf};
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
    const fn bundled() -> Self {
        Self {
            distribution: "rlib_bundle",
            workspace_version: env!("FOUR_P_CORELIB_VERSION"),
            git_commit: env!("FOUR_P_CORELIB_GIT_COMMIT"),
            target: env!("FOUR_P_CORELIB_TARGET"),
            rustc_release: env!("FOUR_P_CORELIB_RUSTC_RELEASE"),
            artifact_sha256: env!("FOUR_P_CORELIB_ARTIFACT_SHA256"),
            profile: env!("FOUR_P_CORELIB_WORKSPACE_FEATURES"),
        }
    }
}

struct RunDirectory {
    path: PathBuf,
    sequence: u32,
}

#[derive(Clone)]
pub struct RunContext {
    stamp: String,
    root: Arc<Mutex<RunDirectory>>,
    log: Arc<Mutex<Option<File>>>,
    metadata: Arc<Mutex<RunMetadata>>,
}

impl RunContext {
    pub fn create() -> Result<Self> {
        Self::create_in(Path::new("runs"), Utc::now())
    }

    /// Create a fresh, unconfigured run beside this one without touching its
    /// results. The new run starts in the same parent directory, with its own
    /// timestamp and log; same-second runs get a numeric sequence suffix.
    pub fn next_run(&self) -> Result<Self> {
        let current = self.root();
        let parent = current
            .parent()
            .with_context(|| format!("{} has no parent directory", current.display()))?;
        Self::create_in(parent, Utc::now())
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
            root: Arc::new(Mutex::new(RunDirectory { path, sequence })),
            log: Arc::new(Mutex::new(Some(log))),
            metadata: Arc::new(Mutex::new(RunMetadata {
                schema_version: 1,
                started_at,
                ended_at: None,
                status: RunStatus::InProgress,
                mode: None,
                application_version: env!("CARGO_PKG_VERSION").to_owned(),
                corelib: CorelibProvenance::bundled(),
                error_category: None,
                stage_duration_ms: BTreeMap::new(),
            })),
        };
        context.persist_metadata()?;
        context.log("INFO", "run directory created")?;
        Ok(context)
    }

    #[must_use]
    pub fn root(&self) -> PathBuf {
        self.root
            .lock()
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
        let (current, mut sequence) = {
            let directory = self
                .root
                .lock()
                .map_err(|_| anyhow::anyhow!("run root lock poisoned"))?;
            (directory.path.clone(), directory.sequence)
        };
        let parent = current
            .parent()
            .with_context(|| format!("{} has no parent directory", current.display()))?;
        let mut log = self
            .log
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
            let mut directory = self
                .root
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
        let bytes = fs::read(source).with_context(|| format!("read {}", source.display()))?;
        atomic_write(&self.root().join("config.yaml"), &bytes)
    }

    pub fn write_yaml<T: Serialize>(&self, relative: &Path, value: &T) -> Result<()> {
        let bytes = serde_yaml::to_string(value).context("serialize yaml")?;
        atomic_write(&self.root().join(relative), bytes.as_bytes())
    }

    pub fn log(&self, level: &str, message: &str) -> Result<()> {
        let stamp = Utc::now().to_rfc3339_opts(SecondsFormat::Millis, true);
        let normalized = message.replace('\n', "\\n");
        let mut log = self
            .log
            .lock()
            .map_err(|_| anyhow::anyhow!("run log lock poisoned"))?;
        let file = log.as_mut().context("run log file is unavailable")?;
        writeln!(file, "{stamp} {level} {normalized}").context("append run log")?;
        file.flush().context("flush run log")
    }

    pub fn finish(&self, status: RunStatus, error_category: Option<&str>) -> Result<()> {
        {
            let mut metadata = self.lock_metadata()?;
            metadata.status = status;
            metadata.ended_at = Some(Utc::now());
            metadata.error_category = error_category.map(str::to_owned);
        }
        self.persist_metadata()?;
        self.log("INFO", &format!("run finalized with status {status:?}"))
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
        atomic_write(&self.root().join("run.yaml"), yaml.as_bytes())
    }
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
}

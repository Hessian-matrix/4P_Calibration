//! 角点观测的会话留存与离线重放（本 crate 唯一的观测日志实现）。
//!
//! 一次采集 = 一个会话目录 `root/sessions/<unique-id>/`：
//!
//! - `config.yaml`：`create` 时写入的**已解析实际 `Config`**（不是原始输入文本），重放时按同一
//!   schema 加载并校验；
//! - `observations.jsonl`：一行为**一个已审核帧组**的完整 JSON，含组身份（`version` /
//!   `group_id` / `group_timestamp_ns`）与逐路角点观测（相机/帧身份 + `object_points` /
//!   `image_points`）。
//!
//! 只保存**角点与身份**——不保存原始像素，不做压缩/数据库/后台线程：数值重算只需要这些，
//! 现场也不再需要连回设备。
//!
//! 写盘路径**不静默**：`append` 先做全部校验（单调性/身份/相机集合/角点有效性），再把整行
//! 一次性写入并 `sync_data`；任一失败都返回 `Err`，且**不推进**日志的单调性游标——调用方只有
//! 在 `Ok` 之后才提交内存入库。`read_observations` 同样**不假装全量恢复**：截断（缺尾换行）、
//! 空行、坏 JSON、语义非法都明确报错，绝不跳过最后半行再返回「成功」。

use std::collections::BTreeSet;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

use rigcal_core::config::{Config, ConfigError};
use rigcal_core::estimator::Observation;
use serde::{Deserialize, Serialize};

/// 会话根目录名（`root/sessions/<unique-id>/`）。
pub const SESSIONS_DIR: &str = "sessions";
/// 会话内已解析配置文件名。
pub const CONFIG_FILE: &str = "config.yaml";
/// 会话内观测日志文件名（JSON Lines）。
pub const OBSERVATIONS_FILE: &str = "observations.jsonl";

/// 唯一会话目录的重试上限（仅在同名目录已被占用时递增后缀）。
const MAX_SESSION_ATTEMPTS: u32 = 10_000;

/// 一路视图的留存：身份 + 角点观测。
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedView {
    pub camera_id: String,
    pub frame_id: u64,
    pub camera_timestamp_ns: u64,
    pub observation: Observation,
}

/// 一个已审核帧组的留存：组身份 + 逐路视图。
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedGroup {
    pub version: usize,
    pub group_id: u64,
    pub group_timestamp_ns: u64,
    pub views: Vec<RecordedView>,
}

#[derive(Debug, thiserror::Error)]
pub enum JournalError {
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("configuration: {0}")]
    Config(#[from] ConfigError),
    #[error("yaml: {0}")]
    Yaml(#[from] serde_yaml::Error),
    #[error("json: {0}")]
    Json(#[from] serde_json::Error),
    #[error("could not allocate a unique session directory under {0}")]
    SessionIdExhausted(PathBuf),
    #[error("journal {path} has no adjacent {CONFIG_FILE}")]
    MissingConfig { path: PathBuf },
    #[error("journal {path} is truncated: line {line} has no terminating newline")]
    Truncated { path: PathBuf, line: usize },
    #[error("journal {path} line {line} is blank")]
    BlankLine { path: PathBuf, line: usize },
    #[error("journal {path} line {line}: {error}")]
    Line {
        path: PathBuf,
        line: usize,
        error: String,
    },
    #[error("group V{version} has no views")]
    EmptyGroup { version: usize },
    #[error("group V{version} repeats camera {camera_id}")]
    DuplicateCamera { version: usize, camera_id: String },
    #[error("group V{version} has camera {camera_id} absent from the session config")]
    UnknownCamera { version: usize, camera_id: String },
    #[error(
        "group V{version} camera {camera_id}: {objects} object points vs {images} image points"
    )]
    CornerCountMismatch {
        version: usize,
        camera_id: String,
        objects: usize,
        images: usize,
    },
    #[error("group V{version} camera {camera_id}: non-finite corner coordinate")]
    NonFiniteCorner { version: usize, camera_id: String },
    #[error("group version {version} is not greater than last recorded {last}")]
    VersionNotMonotonic { version: usize, last: usize },
    #[error("group id {group_id} is not greater than last recorded {last}")]
    GroupIdNotMonotonic { group_id: u64, last: u64 },
}

/// 一次会话的观测日志：独占一个会话目录并持有 `observations.jsonl` 的追加句柄。
pub struct ObservationJournal {
    journal_path: PathBuf,
    file: File,
    allowed_cameras: BTreeSet<String>,
    /// 已成功写入的最后一组 `(version, group_id)`；只有写成功才推进。
    previous: Option<(usize, u64)>,
}

/// 新建一个会话目录 `root/sessions/<unique-id>/`，写入已解析配置并开启空日志。
///
/// 目录用 `create_dir`（`create_new` 语义）申请，**不覆盖任何历史会话**；同名占用时在同一次
/// 调用内递增后缀重试。配置或日志创建失败会回滚掉刚申请的目录再返回 `Err`。
pub fn create(root: &Path, config: &Config) -> Result<ObservationJournal, JournalError> {
    let sessions = root.join(SESSIONS_DIR);
    std::fs::create_dir_all(&sessions)?;

    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|elapsed| elapsed.as_nanos())
        .unwrap_or(0);
    let mut claimed = None;
    for attempt in 0..MAX_SESSION_ATTEMPTS {
        let id = if attempt == 0 {
            format!("s-{stamp:020}")
        } else {
            format!("s-{stamp:020}-{attempt}")
        };
        let candidate = sessions.join(id);
        match std::fs::create_dir(&candidate) {
            Ok(()) => {
                claimed = Some(candidate);
                break;
            }
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(error) => return Err(error.into()),
        }
    }
    let directory = claimed.ok_or_else(|| JournalError::SessionIdExhausted(sessions.clone()))?;

    let prepared = (|| -> Result<(PathBuf, File), JournalError> {
        let config_path = directory.join(CONFIG_FILE);
        write_synced(&config_path, serde_yaml::to_string(config)?.as_bytes())?;
        let journal_path = directory.join(OBSERVATIONS_FILE);
        let file = std::fs::OpenOptions::new()
            .create_new(true)
            .append(true)
            .open(&journal_path)?;
        file.sync_all()?;
        Ok((journal_path, file))
    })();

    match prepared {
        Ok((journal_path, file)) => Ok(ObservationJournal {
            journal_path,
            file,
            allowed_cameras: allowed_cameras(config),
            previous: None,
        }),
        Err(error) => {
            let _ = std::fs::remove_dir_all(&directory);
            Err(error)
        }
    }
}

impl ObservationJournal {
    /// 已写入（或待写）的 `observations.jsonl` 路径，可直接交给 [`read_observations`]。
    pub fn path(&self) -> &Path {
        &self.journal_path
    }

    /// 追加一个已审核帧组：先校验，再整行写入并 `sync_data`。
    ///
    /// 只有本方法返回 `Ok` 后调用方才应把该组提交到内存；失败时日志与单调性游标都保持不变，
    /// 后续可用正确的一组继续写。
    pub fn append(&mut self, group: &RecordedGroup) -> Result<(), JournalError> {
        validate_group(&self.allowed_cameras, self.previous, group)?;
        let mut line = serde_json::to_vec(group)?;
        line.push(b'\n');
        self.file.write_all(&line)?;
        self.file.sync_data()?;
        self.previous = Some((group.version, group.group_id));
        Ok(())
    }
}

/// 加载 `path` 相邻的 `config.yaml` 并回放整份日志。
///
/// 日志必须**完整**：截断、空行、坏 JSON 或语义非法的组都以 `Err` 结束，不会返回「已恢复的
/// 前缀」冒充全量恢复。
pub fn read_observations(path: &Path) -> Result<(Config, Vec<RecordedGroup>), JournalError> {
    let config_path = path
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .join(CONFIG_FILE);
    let config_text = match std::fs::read_to_string(&config_path) {
        Ok(text) => text,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            return Err(JournalError::MissingConfig { path: config_path });
        }
        Err(error) => return Err(error.into()),
    };
    let config = Config::from_yaml(&config_text)?;

    let text = std::fs::read_to_string(path)?;
    if !text.is_empty() && !text.ends_with('\n') {
        return Err(JournalError::Truncated {
            path: path.to_path_buf(),
            line: text.split('\n').count(),
        });
    }

    let allowed = allowed_cameras(&config);
    let mut groups = Vec::new();
    let mut previous: Option<(usize, u64)> = None;
    for (index, line) in text.lines().enumerate() {
        let number = index + 1;
        if line.trim().is_empty() {
            return Err(JournalError::BlankLine {
                path: path.to_path_buf(),
                line: number,
            });
        }
        let group: RecordedGroup =
            serde_json::from_str(line).map_err(|error| JournalError::Line {
                path: path.to_path_buf(),
                line: number,
                error: error.to_string(),
            })?;
        validate_group(&allowed, previous, &group)?;
        previous = Some((group.version, group.group_id));
        groups.push(group);
    }
    Ok((config, groups))
}

/// 配置允许出现的相机集合：就是 `cameras` 列出的那些。
fn allowed_cameras(config: &Config) -> BTreeSet<String> {
    config.cameras.iter().map(|camera| camera.id.clone()).collect()
}

/// 组级校验：版本/组身份单调、非空、相机唯一且属于配置、角点非空等长有限。
///
/// `previous` 是上一组的 `(version, group_id)`；`append` 与 `read_observations` 共用这一条路径，
/// 保证「写进去的」和「读回来愿意接受的」是同一套契约。
fn validate_group(
    allowed: &BTreeSet<String>,
    previous: Option<(usize, u64)>,
    group: &RecordedGroup,
) -> Result<(), JournalError> {
    if let Some((last_version, last_group_id)) = previous {
        if group.version <= last_version {
            return Err(JournalError::VersionNotMonotonic {
                version: group.version,
                last: last_version,
            });
        }
        if group.group_id <= last_group_id {
            return Err(JournalError::GroupIdNotMonotonic {
                group_id: group.group_id,
                last: last_group_id,
            });
        }
    }
    if group.views.is_empty() {
        return Err(JournalError::EmptyGroup {
            version: group.version,
        });
    }

    let mut seen = BTreeSet::new();
    for view in &group.views {
        if !seen.insert(view.camera_id.as_str()) {
            return Err(JournalError::DuplicateCamera {
                version: group.version,
                camera_id: view.camera_id.clone(),
            });
        }
        if !allowed.contains(&view.camera_id) {
            return Err(JournalError::UnknownCamera {
                version: group.version,
                camera_id: view.camera_id.clone(),
            });
        }
        let object_points = &view.observation.object_points;
        let image_points = &view.observation.image_points;
        if object_points.is_empty() || object_points.len() != image_points.len() {
            return Err(JournalError::CornerCountMismatch {
                version: group.version,
                camera_id: view.camera_id.clone(),
                objects: object_points.len(),
                images: image_points.len(),
            });
        }
        if object_points
            .iter()
            .flatten()
            .any(|value| !value.is_finite())
            || image_points
                .iter()
                .flatten()
                .any(|value| !value.is_finite())
        {
            return Err(JournalError::NonFiniteCorner {
                version: group.version,
                camera_id: view.camera_id.clone(),
            });
        }
    }
    Ok(())
}

fn write_synced(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let mut file = File::create(path)?;
    file.write_all(bytes)?;
    file.sync_all()
}

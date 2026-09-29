//! 多路帧组捕获（四路 rig 用）：锚点 → 并发扇出 → 组一致性校验 → 两级重试。
//!
//! 默认设备合同：同 `group_id` 且同 `group_timestamp_ns` 代表相同曝光时刻。
//! 四路必须同时满足这两项，不返回混合组；`max_skew_ns` 仅为诊断。
//! 已知目标时刻时四路同时 GET，不等待锚点下载；未指定时刻的探针先取 LATEST
//! 锚点再并发扇出。生产同刻采集失败不会退回另一个时刻的 LATEST。

use std::collections::HashMap;
use std::time::Duration;

use crate::raw_tcp::{RawFrame, RawFrameError, RawTcpFrameSource};

/// 同一锚点下失败路的重取次数。
pub const SAME_ANCHOR_RETRIES: usize = 1;
pub const DEFAULT_MAX_REANCHOR: usize = 3;

#[derive(Clone, Debug)]
pub struct CameraGroupSpec {
    pub camera_id: String,
    pub raw_camera_id: i32,
}

#[derive(Clone, Debug)]
pub struct GroupedFrame {
    pub camera_id: String,
    pub frame: RawFrame,
}

#[derive(Clone, Debug)]
pub struct FrameGroup {
    pub group_id: u64,
    pub group_timestamp_ns: u64,
    pub frames: Vec<GroupedFrame>,
    /// 组内各路 `camera_timestamp_ns` 的极差（只记录）。
    pub max_skew_ns: u64,
    /// 第几次尝试（含重新取锚点）才组成功。
    pub attempts: usize,
}

#[derive(Debug, thiserror::Error)]
pub enum GroupError {
    #[error("capture_group requires at least one camera")]
    NoCameras,
    #[error("trigger camera {0} is not part of the enabled set")]
    UnknownTrigger(String),
    #[error("raw link failure at {endpoint}: {source}")]
    Link {
        endpoint: String,
        #[source]
        source: RawFrameError,
    },
    #[error(
        "failed to capture a synchronized group from {endpoint} after {attempts} attempt(s): {reason}"
    )]
    Unsynchronized {
        endpoint: String,
        attempts: usize,
        reason: String,
    },
}

pub struct GroupCaptureOptions {
    pub max_reanchor: usize,
    pub same_anchor_retries: usize,
}

impl Default for GroupCaptureOptions {
    fn default() -> Self {
        Self {
            max_reanchor: DEFAULT_MAX_REANCHOR,
            same_anchor_retries: SAME_ANCHOR_RETRIES,
        }
    }
}

/// 帧组捕获器：持有端点与各路相机编号，`capture()` 每次取一组。
pub struct GroupCapture {
    host: String,
    port: u16,
    timeout: Duration,
    image_size: (u32, u32),
    trigger: CameraGroupSpec,
    others: Vec<CameraGroupSpec>,
    options: GroupCaptureOptions,
}

impl GroupCapture {
    pub fn new(
        host: &str,
        port: u16,
        cameras: &[CameraGroupSpec],
        trigger_camera_id: &str,
        image_size: (u32, u32),
        timeout_s: f64,
        options: GroupCaptureOptions,
    ) -> Result<Self, GroupError> {
        if cameras.is_empty() {
            return Err(GroupError::NoCameras);
        }
        let trigger = cameras
            .iter()
            .find(|camera| camera.camera_id == trigger_camera_id)
            .cloned()
            .ok_or_else(|| GroupError::UnknownTrigger(trigger_camera_id.to_owned()))?;
        let others = cameras
            .iter()
            .filter(|camera| camera.camera_id != trigger_camera_id)
            .cloned()
            .collect();
        Ok(Self {
            host: host.to_owned(),
            port,
            timeout: Duration::from_secs_f64(timeout_s.max(0.1)),
            image_size,
            trigger,
            others,
            options,
        })
    }

    pub fn endpoint(&self) -> String {
        format!(
            "raw-tcp://{}:{}/{}",
            self.host, self.port, self.trigger.camera_id
        )
    }

    fn source(&self, camera: &CameraGroupSpec) -> Result<RawTcpFrameSource, RawFrameError> {
        RawTcpFrameSource::new(
            &self.host,
            self.port,
            camera.raw_camera_id,
            self.image_size,
            self.timeout.as_secs_f64(),
        )
    }

    /// 并发取回多路；`Err`（链路层）直接上抛，`Ok(None)`（该刻没有帧）留给重试逻辑。
    fn fetch_many(
        &self,
        cameras: &[CameraGroupSpec],
        timestamp_ns: Option<u64>,
    ) -> Result<Vec<Option<RawFrame>>, GroupError> {
        if cameras.is_empty() {
            return Ok(Vec::new());
        }
        let endpoint = self.endpoint();
        // 先校验连接参数，再并发取图，避免串行传输耗尽设备 ring 保留窗口。
        let mut sources = Vec::with_capacity(cameras.len());
        for camera in cameras {
            sources.push(self.source(camera).map_err(|source| GroupError::Link {
                endpoint: endpoint.clone(),
                source,
            })?);
        }
        let results: Vec<Result<Option<RawFrame>, RawFrameError>> = std::thread::scope(|scope| {
            let handles: Vec<_> = sources
                .into_iter()
                .map(|mut source| scope.spawn(move || source.fetch(timestamp_ns)))
                .collect();
            handles
                .into_iter()
                .map(|handle| {
                    handle.join().unwrap_or_else(|_| {
                        Err(RawFrameError::Io(std::io::Error::other(
                            "fetch thread panicked",
                        )))
                    })
                })
                .collect()
        });
        results
            .into_iter()
            .map(|result| {
                result.map_err(|source| GroupError::Link {
                    endpoint: endpoint.clone(),
                    source,
                })
            })
            .collect()
    }

    /// 校验四路回包是否同组；返回 `(组, 失败原因)`，失败原因直接进错误信息便于现场定位。
    fn build_group(
        &self,
        anchor: RawFrame,
        fetched: &[Option<RawFrame>],
        attempts: usize,
    ) -> (Option<FrameGroup>, String) {
        let mut entries = Vec::with_capacity(self.others.len() + 1);
        let mut stamps = Vec::with_capacity(self.others.len() + 1);
        stamps.push(anchor.header.camera_timestamp_ns);
        entries.push(GroupedFrame {
            camera_id: self.trigger.camera_id.clone(),
            frame: anchor.clone(),
        });
        for (camera, raw) in self.others.iter().zip(fetched) {
            let Some(raw) = raw else {
                return (
                    None,
                    format!(
                        "camera {} returned no frame for anchor {}",
                        camera.camera_id, anchor.header.group_timestamp_ns
                    ),
                );
            };
            if raw.header.group_id != anchor.header.group_id
                || raw.header.group_timestamp_ns != anchor.header.group_timestamp_ns
            {
                return (
                    None,
                    format!(
                        "camera {} returned group {} instead of anchor {}",
                        camera.camera_id,
                        raw.header.group_timestamp_ns,
                        anchor.header.group_timestamp_ns
                    ),
                );
            }
            stamps.push(raw.header.camera_timestamp_ns);
            entries.push(GroupedFrame {
                camera_id: camera.camera_id.clone(),
                frame: raw.clone(),
            });
        }
        let max_skew_ns =
            stamps.iter().max().copied().unwrap_or(0) - stamps.iter().min().copied().unwrap_or(0);
        (
            Some(FrameGroup {
                group_id: anchor.header.group_id,
                group_timestamp_ns: anchor.header.group_timestamp_ns,
                frames: entries,
                max_skew_ns,
                attempts,
            }),
            String::new(),
        )
    }

    /// 按**指定时刻**取一组：四路（含锚点相机）都用 `GET <cam> <timestamp_ns>`。
    ///
    /// 这是时钟对齐后的主路径：`timestamp_ns` 由引导帧的 `pts + offset` 算得，
    /// 板端按**最近邻**（半帧容差）返回那一刻的组 ⇒ 拿到的就是"与引导帧同刻"的四帧。
    /// 不做重锚点或 LATEST 回退；目标由触发帧决定，失败直接返回。
    pub fn capture_at(&self, timestamp_ns: u64) -> Result<FrameGroup, GroupError> {
        let endpoint = self.endpoint();
        let mut attempts = 0usize;
        let mut last_error = "no attempt was made".to_owned();
        let cameras: Vec<_> = std::iter::once(self.trigger.clone())
            .chain(self.others.iter().cloned())
            .collect();
        for _ in 0..self.options.same_anchor_retries + 1 {
            attempts += 1;
            let mut fetched = self.fetch_many(&cameras, Some(timestamp_ns))?;
            let anchor = fetched.remove(0);
            let Some(anchor) = anchor else {
                last_error = format!(
                    "trigger camera {} has no frame near {timestamp_ns}",
                    self.trigger.camera_id
                );
                continue;
            };
            let (group, reason) = self.build_group(anchor.clone(), &fetched, attempts);
            if let Some(group) = group {
                return Ok(group);
            }
            // 失败路重取一次（同 target）
            let retry_cameras: Vec<CameraGroupSpec> = self
                .others
                .iter()
                .zip(fetched.iter())
                .filter(|(_, raw)| raw.is_none())
                .map(|(camera, _)| camera.clone())
                .collect();
            if !retry_cameras.is_empty() {
                let retried = self.fetch_many(&retry_cameras, Some(timestamp_ns))?;
                let mut by_id: HashMap<&str, Option<RawFrame>> = HashMap::new();
                for (camera, raw) in retry_cameras.iter().zip(retried) {
                    by_id.insert(camera.camera_id.as_str(), raw);
                }
                let merged: Vec<Option<RawFrame>> = self
                    .others
                    .iter()
                    .zip(fetched.iter())
                    .map(|(camera, raw)| match raw {
                        Some(frame) => Some(frame.clone()),
                        None => by_id.get(camera.camera_id.as_str()).cloned().flatten(),
                    })
                    .collect();
                let (group, retry_reason) = self.build_group(anchor, &merged, attempts);
                if let Some(group) = group {
                    return Ok(group);
                }
                last_error = format!("{retry_reason} (attempt {attempts}, target {timestamp_ns})");
            } else {
                last_error = format!("{reason} (attempt {attempts}, target {timestamp_ns})");
            }
        }
        Err(GroupError::Unsynchronized {
            endpoint,
            attempts,
            reason: last_error,
        })
    }

    /// 取一组：`LATEST` 锚点 → 并发扇出 → 校验 → 同锚点重取失败路 → 重新取锚点。
    ///
    /// 显式请求最新组的探针/演练入口；不能作为同刻触发失败的回退。
    pub fn capture(&self) -> Result<FrameGroup, GroupError> {
        let endpoint = self.endpoint();
        let mut attempts = 0usize;
        let mut last_error = "no attempt was made".to_owned();
        for _ in 0..self.options.max_reanchor + 1 {
            attempts += 1;
            let mut anchor_source =
                self.source(&self.trigger)
                    .map_err(|source| GroupError::Link {
                        endpoint: endpoint.clone(),
                        source,
                    })?;
            let anchor = anchor_source
                .fetch(None)
                .map_err(|source| GroupError::Link {
                    endpoint: endpoint.clone(),
                    source,
                })?;
            let Some(anchor) = anchor else {
                last_error = format!(
                    "trigger camera {} returned no LATEST frame",
                    self.trigger.camera_id
                );
                continue;
            };
            let mut fetched: Vec<Option<RawFrame>> = Vec::new();
            for _ in 0..self.options.same_anchor_retries + 1 {
                if fetched.is_empty() {
                    fetched =
                        self.fetch_many(&self.others, Some(anchor.header.group_timestamp_ns))?;
                } else {
                    let retry_cameras: Vec<CameraGroupSpec> = self
                        .others
                        .iter()
                        .zip(fetched.iter())
                        .filter(|(_, raw)| raw.is_none())
                        .map(|(camera, _)| camera.clone())
                        .collect();
                    if !retry_cameras.is_empty() {
                        let retried = self
                            .fetch_many(&retry_cameras, Some(anchor.header.group_timestamp_ns))?;
                        let mut by_id: HashMap<&str, Option<RawFrame>> = HashMap::new();
                        for (camera, raw) in retry_cameras.iter().zip(retried) {
                            by_id.insert(camera.camera_id.as_str(), raw);
                        }
                        fetched = self
                            .others
                            .iter()
                            .zip(fetched.iter())
                            .map(|(camera, raw)| match raw {
                                Some(frame) => Some(frame.clone()),
                                None => by_id.get(camera.camera_id.as_str()).cloned().flatten(),
                            })
                            .collect();
                    }
                }
                let (group, reason) = self.build_group(anchor.clone(), &fetched, attempts);
                if let Some(group) = group {
                    // skew 只记录不门禁（见模块文档）。
                    return Ok(group);
                }
                last_error = format!("{reason} (attempt {attempts})");
            }
        }
        Err(GroupError::Unsynchronized {
            endpoint,
            attempts,
            reason: last_error,
        })
    }
}

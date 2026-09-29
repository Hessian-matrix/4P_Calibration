//! 采集适配器：板端 raw 帧服务（RWF1/TCP）与 RTSP 解码。
//!
//! 这一层只做"把帧取回来"，不做检测、不做求解、不碰 UI —— 与 `rigcal-core`（纯算法）和产线
//! bin（会话/GUI）分开。**证据帧只来自 raw 服务**（未经 H.264 编码），RTSP 只用于预览与引导。

pub mod calibration;
pub mod clock;
pub mod group;
pub mod observations;
pub mod raw_tcp;
pub mod rtsp;

pub use calibration::{
    CALIBRATION_FILE, CAMCHAIN_FILE, ExportError, ExportReceipt, REFERENCE_CAMERA,
    export_calibration,
};
pub use clock::{
    ClockAligner, ClockError, ClockEstimate, ClockSample, MIN_CLOCK_SAMPLES,
    PHASE_UNCERTAINTY_LIMIT_NS, estimate_clock,
};
pub use group::{CameraGroupSpec, FrameGroup, GroupCapture, GroupCaptureOptions, GroupError};
pub use rtsp::{FrameSlot, FrameSource, MonoTile, SourceError};

pub use raw_tcp::{
    RAW_FRAME_HEADER_SIZE, RAW_FRAME_MAGIC, RAW_FRAME_VERSION, RawFrame, RawFrameError,
    RawFrameHeader, RawFrameStatus, RawTcpFrameSource, compact_nv12_to_gray, unpack_raw_header,
};

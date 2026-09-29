//! 板端 raw 帧服务客户端（RWF1 协议）。
//!
//! 协议事实（板端 `docs/raw-frame-server-guide.md`）：
//! - **每条 TCP 连接只应答一次**：发一行请求 `LATEST <cam>\n` 或 `GET <cam> <ts>\n`，读一个响应；
//! - 响应头 96 字节小端 `<IIiiQQQQIIIIQ3Q`（magic/version/status/camera_id/两个时间戳/frame_id/
//!   group_id/width/height/stride/vstride/payload_size/3×reserved），magic = `0x31574652`（"RWF1"）；
//! - `status == 0` 后接**紧凑 NV12**（长度 `w*h*3/2`），Y 平面是前 `w*h` 字节、逐行连续。
//!
//! `NO_MATCH`/`NO_FRAME` 返回 `Ok(None)`，由调用方决定是否重试；
//! `CAMERA_DISABLED`、未知状态、短读、magic/version 不符 → 错误，绝不把半个响应当有效帧。

use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

pub const RAW_FRAME_MAGIC: u32 = 0x3157_4652; // "RWF1"
pub const RAW_FRAME_VERSION: u32 = 1;
pub const RAW_FRAME_HEADER_SIZE: usize = 96;

#[derive(Debug, thiserror::Error)]
pub enum RawFrameError {
    #[error("raw link io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("short response header {got} != {RAW_FRAME_HEADER_SIZE}")]
    ShortHeader { got: usize },
    #[error("bad raw frame magic 0x{0:08x}")]
    BadMagic(u32),
    #[error("unsupported raw frame version {0}")]
    BadVersion(u32),
    #[error("camera {0} is disabled on the raw server")]
    CameraDisabled(i32),
    #[error("unexpected raw frame status {0}")]
    UnexpectedStatus(i32),
    #[error("payload size {got} != compact NV12 {expected}")]
    PayloadSize { got: usize, expected: usize },
    #[error("raw frame size {got:?} != expected {expected:?}")]
    SizeMismatch {
        got: (u32, u32),
        expected: (u32, u32),
    },
    #[error("camera_id must be 0..3, got {0}")]
    BadCameraId(i32),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RawFrameStatus {
    Ok,
    NoMatch,
    BadRequest,
    CameraDisabled,
    NoFrame,
    Unknown(i32),
}

impl RawFrameStatus {
    fn from_i32(value: i32) -> Self {
        match value {
            0 => RawFrameStatus::Ok,
            1 => RawFrameStatus::NoMatch,
            2 => RawFrameStatus::BadRequest,
            3 => RawFrameStatus::CameraDisabled,
            4 => RawFrameStatus::NoFrame,
            other => RawFrameStatus::Unknown(other),
        }
    }
}

#[derive(Clone, Debug)]
pub struct RawFrameHeader {
    pub magic: u32,
    pub version: u32,
    pub status: RawFrameStatus,
    pub camera_id: i32,
    pub group_timestamp_ns: u64,
    pub camera_timestamp_ns: u64,
    pub frame_id: u64,
    pub group_id: u64,
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub vstride: u32,
    pub payload_size: u64,
}

#[derive(Clone, Debug)]
pub struct RawFrame {
    pub header: RawFrameHeader,
    /// mono8 亮度（Y 平面），行优先。
    pub gray: Vec<u8>,
}

impl RawFrame {
    pub fn size(&self) -> (u32, u32) {
        (self.header.width, self.header.height)
    }

    pub fn group_timestamp_ns(&self) -> u64 {
        self.header.group_timestamp_ns
    }

    pub fn frame_id(&self) -> u64 {
        self.header.frame_id
    }

    /// Y 平面的 FNV-1a 摘要。
    pub fn gray_digest(&self) -> String {
        use std::fmt::Write as _;
        // 按像素字节累积 FNV-1a。
        let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
        for byte in &self.gray {
            hash ^= *byte as u64;
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
        let mut out = String::with_capacity(16);
        let _ = write!(out, "{hash:016x}");
        out
    }
}

fn read_exact(stream: &mut TcpStream, size: usize) -> Result<Vec<u8>, RawFrameError> {
    let mut buffer = vec![0u8; size];
    let mut filled = 0;
    while filled < size {
        let read = stream.read(&mut buffer[filled..])?;
        if read == 0 {
            return Err(RawFrameError::ShortHeader { got: filled });
        }
        filled += read;
    }
    Ok(buffer)
}

fn field_u32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

fn field_i32(bytes: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

fn field_u64(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
}

/// 按 RWF1 小端布局解析 96 字节响应头。
pub fn unpack_raw_header(bytes: &[u8]) -> Result<RawFrameHeader, RawFrameError> {
    if bytes.len() != RAW_FRAME_HEADER_SIZE {
        return Err(RawFrameError::ShortHeader { got: bytes.len() });
    }
    let magic = field_u32(bytes, 0);
    if magic != RAW_FRAME_MAGIC {
        return Err(RawFrameError::BadMagic(magic));
    }
    let version = field_u32(bytes, 4);
    if version != RAW_FRAME_VERSION {
        return Err(RawFrameError::BadVersion(version));
    }
    Ok(RawFrameHeader {
        magic,
        version,
        status: RawFrameStatus::from_i32(field_i32(bytes, 8)),
        camera_id: field_i32(bytes, 12),
        group_timestamp_ns: field_u64(bytes, 16),
        camera_timestamp_ns: field_u64(bytes, 24),
        frame_id: field_u64(bytes, 32),
        group_id: field_u64(bytes, 40),
        width: field_u32(bytes, 48),
        height: field_u32(bytes, 52),
        stride: field_u32(bytes, 56),
        vstride: field_u32(bytes, 60),
        payload_size: field_u64(bytes, 64),
    })
}

/// 紧凑 NV12 → Y 平面；长度不符一律报错（fail closed）。
pub fn compact_nv12_to_gray(
    payload: &[u8],
    width: u32,
    height: u32,
) -> Result<Vec<u8>, RawFrameError> {
    let y_bytes = (width as usize) * (height as usize);
    let expected = y_bytes + y_bytes / 2;
    if payload.len() != expected {
        return Err(RawFrameError::PayloadSize {
            got: payload.len(),
            expected,
        });
    }
    Ok(payload[..y_bytes].to_vec())
}

/// 板端 raw 帧源：每次 `fetch` 新建连接、发一行、读一个响应。
#[derive(Clone, Debug)]
pub struct RawTcpFrameSource {
    host: String,
    port: u16,
    camera_id: i32,
    expected_size: (u32, u32),
    timeout: Duration,
    index: u64,
}

impl RawTcpFrameSource {
    pub fn new(
        host: &str,
        port: u16,
        camera_id: i32,
        expected_size: (u32, u32),
        timeout_s: f64,
    ) -> Result<Self, RawFrameError> {
        if !(0..=3).contains(&camera_id) {
            return Err(RawFrameError::BadCameraId(camera_id));
        }
        Ok(Self {
            host: host.to_owned(),
            port,
            camera_id,
            expected_size,
            timeout: Duration::from_secs_f64(timeout_s.max(0.1)),
            index: 0,
        })
    }

    pub fn source_label(&self) -> String {
        format!(
            "raw-tcp://{}:{}/cam{}",
            self.host, self.port, self.camera_id
        )
    }

    fn open(&self) -> Result<TcpStream, RawFrameError> {
        let address = format!("{}:{}", self.host, self.port);
        let mut last_error = None;
        for candidate in address.to_socket_addrs()? {
            match TcpStream::connect_timeout(&candidate, self.timeout) {
                Ok(stream) => {
                    stream.set_read_timeout(Some(self.timeout))?;
                    stream.set_write_timeout(Some(self.timeout))?;
                    return Ok(stream);
                }
                Err(error) => last_error = Some(error),
            }
        }
        Err(RawFrameError::Io(last_error.unwrap_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::AddrNotAvailable, "no address resolved")
        })))
    }

    /// `timestamp_ns = None` 发 `LATEST`，否则发 `GET <cam> <ts>`。
    pub fn fetch(&mut self, timestamp_ns: Option<u64>) -> Result<Option<RawFrame>, RawFrameError> {
        let request = match timestamp_ns {
            None => format!("LATEST {}\n", self.camera_id),
            Some(stamp) => format!("GET {} {}\n", self.camera_id, stamp),
        };
        let mut stream = self.open()?;
        stream.write_all(request.as_bytes())?;
        stream.flush()?;
        let header_bytes = read_exact(&mut stream, RAW_FRAME_HEADER_SIZE)?;
        let header = unpack_raw_header(&header_bytes)?;
        match header.status {
            RawFrameStatus::Ok => {
                let payload = if header.payload_size > 0 {
                    read_exact(&mut stream, header.payload_size as usize)?
                } else {
                    Vec::new()
                };
                let gray = compact_nv12_to_gray(&payload, header.width, header.height)?;
                if (header.width, header.height) != self.expected_size {
                    return Err(RawFrameError::SizeMismatch {
                        got: (header.width, header.height),
                        expected: self.expected_size,
                    });
                }
                self.index += 1;
                Ok(Some(RawFrame { header, gray }))
            }
            RawFrameStatus::NoMatch | RawFrameStatus::NoFrame => Ok(None),
            RawFrameStatus::CameraDisabled => Err(RawFrameError::CameraDisabled(self.camera_id)),
            other => Err(RawFrameError::UnexpectedStatus(match other {
                RawFrameStatus::Ok
                | RawFrameStatus::NoMatch
                | RawFrameStatus::BadRequest
                | RawFrameStatus::CameraDisabled
                | RawFrameStatus::NoFrame => unreachable!(),
                RawFrameStatus::Unknown(value) => value,
            })),
        }
    }

    pub fn expected_size(&self) -> (u32, u32) {
        self.expected_size
    }
}

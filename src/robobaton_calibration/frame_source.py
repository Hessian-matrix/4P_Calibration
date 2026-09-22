"""Read lossless image directories or the latest decoded RTSP mono8 frame."""

import os
import select
import socket
import struct
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Generator, Iterable, Optional, Tuple
from urllib.parse import urlsplit, urlunsplit

import cv2
import numpy as np
import subprocess
import threading
import time
import json


@dataclass(frozen=True)
class MonoFrame:
    index: int
    gray: np.ndarray
    source: str
    decoded_encoding: str = "mono8"
    monotonic_ns: int = 0


@dataclass(frozen=True)
class StreamInfo:
    codec_name: str
    width: int
    height: int


class _LatestFrameBuffer:
    """RTSP解码必须持续泄压，慢检测线程只消费最新帧，避免stdout和解码队列累积延迟。"""

    def __init__(self, timeout_s: float, timeout_message: str) -> None:
        self.timeout_s = timeout_s
        self.timeout_message = timeout_message
        self._condition = threading.Condition()
        self._latest_frame: Optional[MonoFrame] = None
        self._latest_generation = 0
        self._closed = False
        self._error: Optional[BaseException] = None

    def push(self, frame: MonoFrame) -> None:
        with self._condition:
            if self._closed:
                return
            # 队列深度固定为1；慢消费者醒来时只拿最新帧，旧帧直接丢弃。
            self._latest_frame = frame
            self._latest_generation += 1
            self._condition.notify_all()

    def fail(self, exc: BaseException) -> None:
        with self._condition:
            self._error = exc
            self._closed = True
            self._condition.notify_all()

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def frames(self) -> Generator[MonoFrame, None, None]:
        delivered_generation = 0
        while True:
            with self._condition:
                deadline = time.monotonic() + self.timeout_s
                # 等待新generation而不是等待每一帧，保证检测耗时超过帧周期时不会回放积压旧帧。
                while self._latest_generation == delivered_generation and self._error is None and not self._closed:
                    remaining_s = deadline - time.monotonic()
                    if remaining_s <= 0.0:
                        raise RuntimeError(self.timeout_message)
                    self._condition.wait(remaining_s)
                if self._error is not None:
                    raise self._error
                if self._latest_generation == delivered_generation and self._closed:
                    return
                frame = self._latest_frame
                delivered_generation = self._latest_generation
            assert frame is not None
            yield frame



def redact_url(url: str) -> str:
    parts = urlsplit(url)
    if "@" not in parts.netloc:
        return url
    userinfo, host = parts.netloc.rsplit("@", 1)
    username = userinfo.split(":", 1)[0]
    return urlunsplit((parts.scheme, f"{username}:***@{host}", parts.path, parts.query, parts.fragment))


@lru_cache(maxsize=2)
def _rtsp_timeout_option(binary: str) -> str:
    help_result = subprocess.run([binary, "-hide_banner", "-h", "demuxer=rtsp"], text=True, capture_output=True, check=True, timeout=10)
    options = {line.strip().split()[0] for line in help_result.stdout.splitlines() if line.strip().startswith("-")}
    if "-stimeout" in options:
        return "-stimeout"
    if "-timeout" in options:
        return "-timeout"
    raise RuntimeError(f"{binary} lacks a supported RTSP socket timeout option")


def _rtsp_client_input_options(timeout_s: float, binary: str) -> list:
    """在线预览要优先显示最新画面，禁止FFmpeg探测和解码默认缓存造成秒级延迟。"""
    # 这些参数必须位于RTSP输入URL之前，才能约束demux/probe阶段的客户端缓存。
    return [
        "-rtsp_transport",
        "tcp",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-analyzeduration",
        "0",
        "-probesize",
        "32",
        _rtsp_timeout_option(binary),
        str(int(timeout_s * 1_000_000)),
    ]



def build_ffprobe_command(rtsp_url: str, timeout_s: float = 10.0) -> list:
    return [
        "ffprobe",
        "-v",
        "error",
        *_rtsp_client_input_options(timeout_s, "ffprobe"),
        "-show_entries",
        "stream=codec_type,codec_name,width,height",
        "-of",
        "json",
        rtsp_url,
    ]


def parse_ffprobe_stream_info(payload: str, expected_size: Tuple[int, int]) -> StreamInfo:
    data = json.loads(payload)
    video_streams = [stream for stream in data.get("streams", []) if stream.get("codec_type") == "video"]
    if not video_streams:
        raise ValueError("ffprobe found no video stream")
    stream = video_streams[0]
    info = StreamInfo(
        codec_name=str(stream.get("codec_name", "")),
        width=int(stream.get("width", 0)),
        height=int(stream.get("height", 0)),
    )
    if info.codec_name not in {"h264", "h265", "hevc"}:
        raise ValueError(f"unsupported RTSP codec: {info.codec_name}")
    if (info.width, info.height) != expected_size:
        raise ValueError(f"RTSP stream size {(info.width, info.height)} does not match expected size {expected_size}")
    return info


def probe_rtsp_stream(rtsp_url: str, expected_size: Tuple[int, int], timeout_s: float = 10.0) -> StreamInfo:
    command = build_ffprobe_command(rtsp_url, timeout_s=timeout_s)
    completed = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False, timeout=timeout_s + 5.0)
    if completed.returncode != 0:
        detail = completed.stderr.strip().replace(rtsp_url, redact_url(rtsp_url))
        raise RuntimeError(f"ffprobe failed for {redact_url(rtsp_url)}: {detail}")
    return parse_ffprobe_stream_info(completed.stdout, expected_size=expected_size)


def build_ffmpeg_mono8_command(rtsp_url: str, expected_size: Tuple[int, int], timeout_s: float = 10.0) -> list:
    return [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        *_rtsp_client_input_options(timeout_s, "ffmpeg"),
        "-i",
        rtsp_url,
        "-an",
        "-sn",
        "-dn",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "gray",
        "pipe:1",
    ]


class DirectoryFrameSource:
    def __init__(self, directory: Path, expected_size: Tuple[int, int]) -> None:
        self.directory = Path(directory)
        self.expected_size = expected_size

    def frames(self) -> Generator[MonoFrame, None, None]:
        paths = sorted(
            path for path in self.directory.iterdir() if path.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp", ".pgm"}
        )
        for index, path in enumerate(paths):
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise RuntimeError(f"failed to read fixture frame: {path}")
            expected_width, expected_height = self.expected_size
            if (image.shape[1], image.shape[0]) != (expected_width, expected_height):
                raise ValueError(f"fixture frame {path} has size {(image.shape[1], image.shape[0])}, expected {self.expected_size}")
            yield MonoFrame(index=index, gray=image, source=str(path), monotonic_ns=time.monotonic_ns())


class RTSPFrameSource:
    def __init__(self, rtsp_url: str, expected_size: Tuple[int, int], timeout_s: float = 10.0) -> None:
        self.rtsp_url = rtsp_url
        self.expected_size = expected_size
        self.timeout_s = timeout_s
        self._process: Optional[subprocess.Popen] = None
        self._stderr_chunks: list[bytes] = []
        self._stderr_thread: Optional[threading.Thread] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame_buffer: Optional[_LatestFrameBuffer] = None
    def _drain_stderr(self) -> None:
        process = self._process
        if process is None or process.stderr is None:
            return
        while True:
            chunk = process.stderr.read(4096)
            if not chunk:
                return
            self._stderr_chunks.append(chunk)
            if len(self._stderr_chunks) > 16:
                del self._stderr_chunks[:-16]

    def _stderr_text(self) -> str:
        return b"".join(self._stderr_chunks[-16:]).decode("utf-8", errors="replace").strip()

    def _read_exact_frame(self, frame_bytes: int) -> bytes:
        assert self._process is not None and self._process.stdout is not None
        fd = self._process.stdout.fileno()
        deadline = time.monotonic() + self.timeout_s
        payload = bytearray()
        while len(payload) < frame_bytes:
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                raise RuntimeError(f"DECODER_TIMEOUT no complete mono8 frame from {redact_url(self.rtsp_url)}")
            ready, _, _ = select.select([fd], [], [], remaining_s)
            if not ready:
                raise RuntimeError(f"DECODER_TIMEOUT no complete mono8 frame from {redact_url(self.rtsp_url)}")
            chunk = os.read(fd, frame_bytes - len(payload))
            if not chunk:
                if self._process.poll() is not None:
                    raise RuntimeError(f"DECODER_EXIT ffmpeg exited for {redact_url(self.rtsp_url)}: {self._stderr_text()}")
                raise RuntimeError(f"DECODER_EOF ffmpeg returned EOF for {redact_url(self.rtsp_url)}")
            payload.extend(chunk)
        return bytes(payload)

    def _read_stdout_frames(self) -> None:
        width, height = self.expected_size
        frame_bytes = width * height
        index = 0
        buffer = self._frame_buffer
        if buffer is None:
            return
        try:
            while not self._stop_event.is_set():
                payload = self._read_exact_frame(frame_bytes)
                # 后台线程持续读取完整raw frame并复制成独立数组，避免FFmpeg stdout被检测/绘制耗时反压。
                gray = np.frombuffer(payload, dtype=np.uint8).reshape(height, width).copy()
                buffer.push(MonoFrame(index=index, gray=gray, source=redact_url(self.rtsp_url), monotonic_ns=time.monotonic_ns()))
                index += 1
        except Exception as exc:
            if not self._stop_event.is_set():
                buffer.fail(exc)
        finally:
            buffer.close()


    def start(self) -> None:
        if self._process is not None:
            return
        probe_rtsp_stream(self.rtsp_url, self.expected_size, timeout_s=self.timeout_s)
        command = build_ffmpeg_mono8_command(self.rtsp_url, self.expected_size, timeout_s=self.timeout_s)
        # RTSP URL作为argv传入,避免shell拼接泄露或注入；stdin断开可防止ffmpeg吞掉s/f/q操作命令。
        self._process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self._stop_event.clear()
        self._frame_buffer = _LatestFrameBuffer(self.timeout_s, f"DECODER_TIMEOUT no decoded mono8 frame from {redact_url(self.rtsp_url)}")
        self._stderr_thread = threading.Thread(target=self._drain_stderr, name="robobaton-ffmpeg-stderr", daemon=True)
        self._stderr_thread.start()
        self._reader_thread = threading.Thread(target=self._read_stdout_frames, name="robobaton-ffmpeg-stdout", daemon=True)
        self._reader_thread.start()


    def frames(self) -> Generator[MonoFrame, None, None]:
        if self._process is None:
            self.start()
        assert self._frame_buffer is not None
        yield from self._frame_buffer.frames()


    def stop(self) -> None:
        if self._process is None:
            if self._frame_buffer is not None:
                self._frame_buffer.close()
            return
        process = self._process
        self._process = None
        self._stop_event.set()
        if self._frame_buffer is not None:
            self._frame_buffer.close()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if process.stdout:
            process.stdout.close()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=0.5)
            self._reader_thread = None
        if process.stderr:
            process.stderr.close()
        if self._stderr_thread is not None:
            self._stderr_thread.join(timeout=0.5)
            self._stderr_thread = None
        self._frame_buffer = None

    def __enter__(self) -> "RTSPFrameSource":
        self.start()
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.stop()


# 板端 raw TCP 服务（sensor_demo --raw-server-*）协议常量：单请求/单响应。
# 响应头 96 字节小端 packed，与板端 RawFrameResponseHeader 布局一一对应；
# status==kOk 后接紧凑 NV12（Y 逐行 width，UV 交错同样逐行 width；总长 width*height*3/2）。
RAW_FRAME_MAGIC = 0x31574652  # "RWF1"
RAW_FRAME_VERSION = 1
RAW_FRAME_HEADER_SIZE = 96
RAW_FRAME_HEADER_FORMAT = "<IIiiQQQQIIIIQ3Q"
RAW_FRAME_HEADER_FIELDS = (
    "magic", "version", "status", "camera_id",
    "group_timestamp_ns", "camera_timestamp_ns", "frame_id", "group_id",
    "width", "height", "stride", "vstride", "payload_size",
    "reserved0", "reserved1", "reserved2",
)


class RawFrameStatus:
    OK = 0
    NO_MATCH = 1
    BAD_REQUEST = 2
    CAMERA_DISABLED = 3
    NO_FRAME = 4


class RawFrameError(RuntimeError):
    """板端 raw TCP 服务返回不可用或畸形响应。"""


def raw_request_line(camera_id: int, timestamp_ns: Optional[int]) -> bytes:
    """构造板端请求行；timestamp_ns 为 None 时发送 LATEST，否则 GET 按时间戳寻址。"""
    if not 0 <= camera_id <= 3:
        raise ValueError(f"camera_id must be 0..3, got {camera_id}")
    if timestamp_ns is None:
        return f"LATEST {camera_id}\n".encode("ascii")
    if timestamp_ns < 0:
        raise ValueError("timestamp_ns must be non-negative")
    return f"GET {camera_id} {timestamp_ns}\n".encode("ascii")


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    """整段读取；短读、超时或对端提前关闭都不得当作有效响应。"""
    chunks = bytearray()
    while len(chunks) < size:
        try:
            chunk = sock.recv(size - len(chunks))
        except socket.timeout:
            raise RawFrameError(f"timed out reading response after {len(chunks)} of {size} bytes") from None
        if not chunk:
            raise RawFrameError(f"connection closed after {len(chunks)} of {size} bytes")
        chunks.extend(chunk)
    return bytes(chunks)


def unpack_raw_header(raw: bytes) -> dict:
    """校验 magic/version 后返回响应头字段字典，键为 RAW_FRAME_HEADER_FIELDS。"""
    if len(raw) != RAW_FRAME_HEADER_SIZE:
        raise RawFrameError(f"short response header {len(raw)} != {RAW_FRAME_HEADER_SIZE}")
    header = dict(zip(RAW_FRAME_HEADER_FIELDS, struct.unpack(RAW_FRAME_HEADER_FORMAT, raw)))
    if header["magic"] != RAW_FRAME_MAGIC:
        raise RawFrameError(f"bad raw frame magic 0x{header['magic']:08x}")
    if header["version"] != RAW_FRAME_VERSION:
        raise RawFrameError(f"unsupported raw frame version {header['version']}")
    return header


def compact_nv12_to_gray(payload: bytes, width: int, height: int) -> np.ndarray:
    """从紧凑 NV12 取 Y 平面转 mono8；Y 是前 width*height 字节，逐行连续。"""
    y_bytes = width * height
    expected = y_bytes + y_bytes // 2
    if len(payload) != expected:
        raise RawFrameError(f"payload size {len(payload)} != compact NV12 {expected}")
    return np.frombuffer(payload, dtype=np.uint8, count=y_bytes).reshape(height, width).copy()


class RawTcpFrameSource:
    """从板端 raw TCP 服务按 LATEST 或时间戳 GET 取 mono8 Y 帧。

    服务端每条连接只应答一次：``fetch()`` 每次新建连接发送一行请求并读一个响应；
    ``frames()`` 持续轮询 LATEST。Y 平面取自紧凑 NV12，不等同于 RTSP 有损解码亮度。
    """

    def __init__(self, host: str, port: int, camera_id: int, expected_size: Tuple[int, int], timeout_s: float = 5.0) -> None:
        self.host = host
        self.port = int(port)
        self.camera_id = int(camera_id)
        if not 0 <= self.camera_id <= 3:
            raise ValueError(f"camera_id must be 0..3, got {self.camera_id}")
        self.expected_size = expected_size
        self.timeout_s = timeout_s
        self._index = 0
        self._last_header: Optional[dict] = None

    @property
    def source_label(self) -> str:
        return f"raw-tcp://{self.host}:{self.port}/cam{self.camera_id}"

    @property
    def last_header(self) -> Optional[dict]:
        return self._last_header

    def _open_socket(self) -> socket.socket:
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        sock.settimeout(self.timeout_s)
        return sock

    def _to_frame(self, header: dict, payload: bytes) -> MonoFrame:
        gray = compact_nv12_to_gray(payload, int(header["width"]), int(header["height"]))
        if (gray.shape[1], gray.shape[0]) != self.expected_size:
            raise RawFrameError(f"raw frame size {(gray.shape[1], gray.shape[0])} != expected {self.expected_size}")
        frame = MonoFrame(index=self._index, gray=gray, source=self.source_label, decoded_encoding="mono8", monotonic_ns=time.monotonic_ns())
        self._index += 1
        return frame

    def fetch(self, timestamp_ns: Optional[int] = None) -> Optional[MonoFrame]:
        """单次请求/响应；timestamp_ns 为 None 发送 LATEST，否则 GET 时间戳。

        GET 无匹配返回 None；LATEST 空 ring 返回 None；相机未启用或协议畸形抛异常。
        """
        request = raw_request_line(self.camera_id, timestamp_ns)
        with self._open_socket() as sock:
            sock.sendall(request)
            header = unpack_raw_header(_recv_exact(sock, RAW_FRAME_HEADER_SIZE))
            self._last_header = header
            status = header["status"]
            if status == RawFrameStatus.OK:
                payload_size = int(header["payload_size"])
                payload = _recv_exact(sock, payload_size) if payload_size else b""
                return self._to_frame(header, payload)
            if status in (RawFrameStatus.NO_MATCH, RawFrameStatus.NO_FRAME):
                return None
            if status == RawFrameStatus.CAMERA_DISABLED:
                raise RawFrameError(f"camera {self.camera_id} is disabled on the raw server")
            raise RawFrameError(f"unexpected raw frame status {status}")

    def frames(self) -> Generator[MonoFrame, None, None]:
        """持续轮询 LATEST；空 ring 短暂退避后重试，连接/协议错误向上抛出。"""
        while True:
            frame = self.fetch(None)
            if frame is not None:
                yield frame
            else:
                time.sleep(0.01)

    def stop(self) -> None:
        """保留接口：当前实现无长连接线程，无需释放资源。"""

    def __enter__(self) -> "RawTcpFrameSource":
        return self

    def __exit__(self, _exc_type, _exc, _tb) -> None:
        self.stop()

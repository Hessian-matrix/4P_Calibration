"""固化交互式标定会话状态机。"""

from dataclasses import dataclass
from enum import Enum
from typing import Callable, List, Optional

from .solver import Observation


class SessionState(str, Enum):
    PREVIEW = "PREVIEW"
    CAPTURING = "CAPTURING"
    CHECKING = "CHECKING"
    SOLVING = "SOLVING"
    PASS = "PASS"
    FAIL = "FAIL"


@dataclass(frozen=True)
class SessionConfig:
    min_observations: int = 12


@dataclass(frozen=True)
class SessionCommandResult:
    status: str
    message: str


class CalibrationSession:
    def __init__(
        self,
        config: SessionConfig,
        solver: Callable[[List[Observation]], object],
        reset_callback: Optional[Callable[[], None]] = None,
        progress_callback: Optional[Callable[[str], None]] = None,
        readiness_gate: Optional[Callable[[List[Observation]], object]] = None,
    ) -> None:
        self.config = config
        self.solver = solver
        self.reset_callback = reset_callback
        self.progress_callback = progress_callback
        self.readiness_gate = readiness_gate
        self.state = SessionState.PREVIEW
        self.observations: List[Observation] = []
        self.solve_result: Optional[object] = None
        self.terminal_status: Optional[str] = None

    def _emit_progress(self, message: str) -> None:
        """会话状态机只转发稳定文本事件，具体输出介质由外层交互入口决定。"""
        if self.progress_callback is not None:
            self.progress_callback(message)

    @property
    def accepted_count(self) -> int:
        return len(self.observations)

    def ingest_observation(self, observation: Observation) -> None:
        if self.state != SessionState.CAPTURING:
            return
        self.observations.append(observation)

    def handle_command(self, command: str) -> SessionCommandResult:
        command = command.strip().lower()
        if command == "s":
            return self._start_capture()
        if command == "f":
            return self._finish_capture()
        if command == "r":
            return self._reset_capture()
        if command == "q":
            return self._cancel()
        return SessionCommandResult(status="IGNORED", message=f"ignored command: {command}")

    def _start_capture(self) -> SessionCommandResult:
        if self.state != SessionState.PREVIEW:
            return SessionCommandResult(status="IGNORED", message=f"cannot start from {self.state}")
        self.state = SessionState.CAPTURING
        return SessionCommandResult(status="CAPTURING", message="capture started")

    def _finish_capture(self) -> SessionCommandResult:
        if self.state != SessionState.CAPTURING:
            return SessionCommandResult(status="IGNORED", message=f"cannot finish from {self.state}")
        self.state = SessionState.CHECKING
        if self.readiness_gate is not None:
            snapshot = self.readiness_gate(list(self.observations))
            missing_tokens = list(getattr(snapshot, "missing", []))
            if len(self.observations) < self.config.min_observations and "ACCEPTED_COUNT" not in missing_tokens and "SOLVE_ELIGIBLE_COUNT" not in missing_tokens:
                missing_tokens.insert(0, "ACCEPTED_COUNT")
            if not bool(getattr(snapshot, "ready", False)) or missing_tokens:
                if not missing_tokens:
                    missing_tokens.append("CAPTURE_READINESS")
                self.state = SessionState.CAPTURING
                return SessionCommandResult(
                    status="CAPTURE_NOT_READY",
                    message=f"capture not ready missing={','.join(missing_tokens)}",
                )
        elif len(self.observations) < self.config.min_observations:
            missing = self.config.min_observations - len(self.observations)
            self.state = SessionState.CAPTURING
            return SessionCommandResult(
                status="MISSING_COVERAGE",
                message=f"need {missing} more accepted observations before solving",
            )
        self.state = SessionState.SOLVING
        # 求解同步阻塞GUI owner线程，START必须在进入solver前输出并flush。
        self._emit_progress(f"SOLVER_START solve_eligible={len(self.observations)} min_observations={self.config.min_observations}")
        try:
            # owner线程唯一触发求解,防止采集线程在状态切换中重入优化。
            self.solve_result = self.solver(list(self.observations))
        except Exception as exc:
            self.state = SessionState.FAIL
            self.terminal_status = "SOLVER_ERROR"
            self._emit_progress("SOLVER_DONE status=FAIL reason=SOLVER_ERROR")
            return SessionCommandResult(status="FAIL", message=str(exc))
        status = getattr(self.solve_result, "status", self.solve_result)
        if status == "PASS":
            self.state = SessionState.PASS
            self.terminal_status = "PASS"
            self._emit_progress("SOLVER_DONE status=PASS")
            return SessionCommandResult(status="PASS", message="calibration solved")
        self.state = SessionState.FAIL
        self.terminal_status = "QUALITY_GATE_FAIL"
        self._emit_progress("SOLVER_DONE status=FAIL reason=QUALITY_GATE")
        return SessionCommandResult(status="FAIL", message="calibration quality gate failed")

    def _reset_capture(self) -> SessionCommandResult:
        if self.state in {SessionState.SOLVING, SessionState.PASS}:
            return SessionCommandResult(status="IGNORED", message=f"cannot reset from {self.state}")
        self.observations.clear()
        if self.reset_callback is not None:
            self.reset_callback()
        self.solve_result = None
        self.terminal_status = None
        self.state = SessionState.PREVIEW
        return SessionCommandResult(status="RESET", message="session reset")

    def _cancel(self) -> SessionCommandResult:
        self.state = SessionState.FAIL
        self.terminal_status = "CANCELED"
        return SessionCommandResult(status="CANCELED", message="session canceled")

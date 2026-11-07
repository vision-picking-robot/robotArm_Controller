"""Feedback-based grasp policy; hardware transport and unit conversion are external.

Motor current is a contact heuristic, not a force sensor. Real profiles require
physical tuning of jaw geometry, current units and each product's acceptance band.
"""
from dataclasses import dataclass
import time

from .config_loader import ConfigError
from .validation import finite_number


class GripError(Exception):
    def __init__(self, code, message):
        self.code = code
        super().__init__(f'{code}: {message}')


@dataclass(frozen=True)
class GripProfile:
    open_width_mm: float
    target_width_mm: float
    min_width_mm: float
    max_width_mm: float
    close_speed_mm_s: float
    current_limit_ma: float
    contact_current_ma: float
    timeout_s: float = 3.0
    settle_s: float = 0.2
    tolerance_mm: float = 1.0
    simulation_only: bool = True

    def __post_init__(self):
        for key, value in vars(self).items():
            if key != 'simulation_only':
                finite_number(value, key)
        if not (0 <= self.target_width_mm < self.min_width_mm <= self.max_width_mm < self.open_width_mm):
            raise ValueError('파지 폭은 목표 닫힘 폭 < 최소 파지 폭 <= 최대 파지 폭 < 열림 폭이어야 합니다')
        if not (0 < self.contact_current_ma <= self.current_limit_ma):
            raise ValueError('접촉 전류와 전류 상한을 확인하세요')
        if min(self.close_speed_mm_s, self.timeout_s, self.settle_s, self.tolerance_mm) <= 0:
            raise ValueError('속도·시간·허용 오차는 양수여야 합니다')
        if self.settle_s >= self.timeout_s:
            raise ValueError('안정화 시간은 타임아웃보다 짧아야 합니다')
        if self.target_width_mm + self.tolerance_mm >= self.min_width_mm:
            raise ValueError('빈 그리퍼 닫힘 범위와 정상 파지 범위가 겹칩니다')


def get_grip_profile(config, product, mode):
    if mode not in ('mock', 'real'):
        raise ConfigError('그리퍼 프로필 mode는 mock 또는 real입니다')
    try:
        source = config['profiles'][product][mode]
        if mode == 'real' and source.get('calibrated') is not True:
            raise ConfigError(f'{product}: 실물 파지 프로필이 검증되지 않았습니다')
        fields = {k: v for k, v in source.items() if k != 'calibrated'}
        return GripProfile(**fields, simulation_only=(mode == 'mock'))
    except (KeyError, TypeError, ValueError) as exc:
        raise ConfigError(f'{product}: 파지 프로필 오류: {exc}') from exc


@dataclass(frozen=True)
class GripFeedback:
    width_mm: float
    current_ma: float
    moving: bool
    observed_at: float  # Local monotonic time of this feedback sample.
    fault: str | None = None


class GripController:
    """backend: simulated, set_opening(width, speed, current_limit), read(), halt().

    halt must stop jaw travel while preserving holding torque when possible;
    it must never implicitly open the fingers or disable torque.
    """
    def __init__(self, backend, now=time.monotonic, sleep=time.sleep):
        self.backend, self.now, self.sleep = backend, now, sleep

    def _abort(self, code, message):
        try:
            self.backend.halt()
        except Exception as exc:
            raise GripError(code, f'{message}; 그리퍼 정지도 실패: {exc}') from exc
        raise GripError(code, message)

    def _read(self, profile):
        try:
            feedback = self.backend.read()
            width = finite_number(feedback.width_mm, 'width_mm')
            current = abs(finite_number(feedback.current_ma, 'current_ma'))
            observed = finite_number(feedback.observed_at, 'observed_at')
        except Exception as exc:
            self._abort('FEEDBACK_INVALID', str(exc))
        age = self.now() - observed
        if age < 0 or age > 0.5:
            self._abort('FEEDBACK_STALE', '최신 그리퍼 상태를 확인할 수 없습니다')
        if width < 0 or type(feedback.moving) is not bool:
            self._abort('FEEDBACK_INVALID', '잘못된 폭 또는 동작 상태')
        if feedback.fault:
            self._abort('DEVICE_FAULT', feedback.fault)
        if current > profile.current_limit_ma:
            self._abort('OVERCURRENT', '측정 전류가 설정 상한을 넘었습니다')
        return feedback

    def _command(self, profile, width):
        if profile.simulation_only and getattr(self.backend, 'simulated', False) is not True:
            raise GripError('MOCK_PROFILE', '가상 파지값을 실제 백엔드에 전달할 수 없습니다')
        try:
            self.backend.set_opening(width, profile.close_speed_mm_s, profile.current_limit_ma)
        except Exception as exc:
            self._abort('COMMAND_FAILED', str(exc))

    def open(self, profile, cancelled=lambda: False):
        if cancelled():
            self._abort('CANCELLED', '그리퍼 동작 취소')
        self._command(profile, profile.open_width_mm)
        deadline = self.now() + profile.timeout_s
        while self.now() < deadline:
            if cancelled():
                self._abort('CANCELLED', '그리퍼 동작 취소')
            feedback = self._read(profile)
            if not feedback.moving and abs(feedback.width_mm - profile.open_width_mm) <= profile.tolerance_mm:
                return feedback
            self.sleep(0.05)
        self._abort('OPEN_TIMEOUT', '열림 위치를 확인하지 못했습니다')

    def grasp(self, profile, cancelled=lambda: False):
        if cancelled():
            self._abort('CANCELLED', '그리퍼 동작 취소')
        self._command(profile, profile.target_width_mm)
        deadline = self.now() + profile.timeout_s
        stable_since = None
        last_sample = None
        while self.now() < deadline:
            if cancelled():
                self._abort('CANCELLED', '그리퍼 동작 취소')
            feedback = self._read(profile)
            if last_sample is not None and feedback.observed_at <= last_sample:
                stable_since = None
                self.sleep(0.05)
                continue
            if last_sample is not None and feedback.observed_at - last_sample > 0.5:
                stable_since = None
            last_sample = feedback.observed_at
            in_band = profile.min_width_mm <= feedback.width_mm <= profile.max_width_mm
            contact = abs(feedback.current_ma) >= profile.contact_current_ma
            if not feedback.moving and not in_band:
                if feedback.width_mm <= profile.target_width_mm + profile.tolerance_mm:
                    self._abort('EMPTY_GRASP', '빈 그리퍼 닫힘 범위에 도달했습니다')
                if contact:
                    self._abort('WIDTH_MISMATCH', '예상 폭 밖에서 접촉했습니다')
            if not feedback.moving and in_band and contact:
                stable_since = self.now() if stable_since is None else stable_since
                if self.now() - stable_since >= profile.settle_s:
                    return feedback
            else:
                stable_since = None
            self.sleep(0.05)
        self._abort('GRASP_TIMEOUT', '폭·전류가 안정된 파지 상태를 확인하지 못했습니다')

    def check_hold(self, profile):
        feedback = self._read(profile)
        if (feedback.moving or not profile.min_width_mm <= feedback.width_mm <= profile.max_width_mm
                or abs(feedback.current_ma) < profile.contact_current_ma):
            self._abort('GRASP_LOST', '들어 올린 뒤 파지 상태가 유지되지 않습니다')
        return feedback

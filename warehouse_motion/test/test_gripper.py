from pathlib import Path
from dataclasses import replace

import pytest

from warehouse_motion.config_loader import ConfigError, load_yaml
from offline_picking.gripper import GripController, GripError, GripFeedback, get_grip_profile
from offline_picking.mock_demo import MockGripper, SimClock

CONFIG = load_yaml(Path(__file__).resolve().parents[1] / 'offline_picking' / 'config' / 'gripper_profiles.yaml')


@pytest.mark.parametrize('name', list(CONFIG['profiles']))
def test_each_product_has_a_working_mock_and_blocked_real_profile(name):
    profile = get_grip_profile(CONFIG, name, 'mock')
    with pytest.raises(ConfigError, match='검증'):
        get_grip_profile(CONFIG, name, 'real')
    clock = SimClock()
    backend = MockGripper(clock, profile)
    grip = GripController(backend, clock.now, clock.sleep)
    grip.open(profile)
    feedback = grip.grasp(profile)
    assert profile.min_width_mm <= feedback.width_mm <= profile.max_width_mm
    assert clock.now() >= profile.settle_s
    grip.check_hold(profile)
    grip.open(profile)
    assert backend.width == profile.open_width_mm
    assert not backend.halted


@pytest.mark.parametrize('outcome,code', [
    ('empty', 'EMPTY_GRASP'), ('jam', 'WIDTH_MISMATCH'),
    ('overcurrent', 'OVERCURRENT'), ('stale', 'FEEDBACK_STALE'),
    ('timeout', 'GRASP_TIMEOUT'),
])
def test_grasp_faults_halt_without_opening_or_increasing_current(outcome, code):
    profile = get_grip_profile(CONFIG, 'snack_box', 'mock')
    clock = SimClock()
    backend = MockGripper(clock, profile, outcome)
    grip = GripController(backend, clock.now, clock.sleep)
    with pytest.raises(GripError) as error:
        grip.grasp(profile)
    assert error.value.code == code
    assert backend.halted
    assert len(backend.commands) == 1
    assert backend.commands[0][0] == profile.target_width_mm
    assert backend.commands[0][2] == profile.current_limit_ma


def test_mock_profile_cannot_send_to_non_mock_backend():
    profile = get_grip_profile(CONFIG, 'snack_box', 'mock')
    clock = SimClock()
    backend = MockGripper(clock, profile)
    backend.simulated = False
    with pytest.raises(GripError, match='MOCK_PROFILE'):
        GripController(backend).open(profile)
    assert backend.commands == []


def test_lost_grasp_is_reported_after_lift():
    profile = get_grip_profile(CONFIG, 'snack_box', 'mock')
    clock = SimClock()
    backend = MockGripper(clock, profile)
    grip = GripController(backend, clock.now, clock.sleep)
    grip.grasp(profile)
    backend.current = 0
    with pytest.raises(GripError, match='GRASP_LOST'):
        grip.check_hold(profile)
    assert backend.halted


@pytest.mark.parametrize('change', [dict(width_mm=float('nan')), dict(current_ma=float('inf')),
                                  dict(fault='driver fault'), dict(moving='false')])
def test_invalid_feedback_cannot_confirm_grasp(change):
    profile = get_grip_profile(CONFIG, 'snack_box', 'mock')
    clock = SimClock()
    backend = MockGripper(clock, profile)
    backend.read = lambda: replace(GripFeedback(54, 33, False, clock.now()), **change)
    with pytest.raises(GripError):
        GripController(backend, clock.now, clock.sleep).grasp(profile)
    assert backend.halted


@pytest.mark.parametrize('change', [dict(open_width_mm=float('nan')), dict(current_limit_ma=0),
                                  dict(target_width_mm=50), dict(settle_s=4)])
def test_invalid_profile_is_rejected(change):
    with pytest.raises(ValueError):
        replace(get_grip_profile(CONFIG, 'snack_box', 'mock'), **change)


def test_cached_feedback_cannot_satisfy_stability_window():
    profile = get_grip_profile(CONFIG, 'snack_box', 'mock')
    clock = SimClock()
    backend = MockGripper(clock, profile)
    backend.read = lambda: GripFeedback(54, 33, False, 0)
    with pytest.raises(GripError, match='FEEDBACK_STALE'):
        GripController(backend, clock.now, clock.sleep).grasp(profile)
    assert backend.halted


def test_cancel_during_grasp_polling_halts_without_release():
    profile = get_grip_profile(CONFIG, 'snack_box', 'mock')
    clock = SimClock()
    backend = MockGripper(clock, profile, 'timeout')
    with pytest.raises(GripError, match='CANCELLED'):
        GripController(backend, clock.now, clock.sleep).grasp(profile, lambda: clock.now() >= 0.1)
    assert clock.now() < profile.timeout_s
    assert backend.halted and len(backend.commands) == 1

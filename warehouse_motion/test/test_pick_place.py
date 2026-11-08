from pathlib import Path

import pytest

from warehouse_motion.config_loader import load_yaml
from offline_picking.gripper import GripController, GripError, get_grip_profile
from offline_picking.mock_demo import MockMotion, MockGripper, SimClock, run_mock_order
from offline_picking.pick_place import execute_pick_place
from warehouse_motion.safe_move import SafeMover, SafetyLimitError

GRIPS = load_yaml(Path(__file__).resolve().parents[1] / 'offline_picking' / 'config' / 'gripper_profiles.yaml')


def setup(outcome='ok'):
    clock, motion = SimClock(), MockMotion()
    profile = get_grip_profile(GRIPS, 'snack_box', 'mock')
    backend = MockGripper(clock, profile, outcome)
    grip = GripController(backend, clock.now, clock.sleep)
    poses = {name: [400, 0, z, 0, 180, 0] for name, z in [
        ('approach', 300), ('grasp', 200), ('lift', 300), ('via', 400),
        ('place_approach', 350), ('place', 200), ('retract', 350)]}
    return motion, backend, grip, profile, poses


def test_complete_cycle_checks_hold_before_transport_and_opens_before_retract():
    motion, backend, grip, profile, poses = setup()
    steps = execute_pick_place(SafeMover(110, motion, 30, 30), grip, profile, poses, motion.stop, vel=10, acc=10)
    assert steps == ['open', 'approach', 'grasp', 'grip_confirmed', 'lift',
                     'hold_checked', 'via', 'place_approach', 'place', 'jaw_opened', 'retract']
    assert len(motion.commands) == 7
    assert not motion.stopped


def test_bad_place_target_is_rejected_before_any_command():
    motion, backend, grip, profile, poses = setup()
    poses['place'][2] = 90
    with pytest.raises(SafetyLimitError):
        execute_pick_place(SafeMover(110, motion, 30, 30), grip, profile, poses, motion.stop, vel=10, acc=10)
    assert motion.commands == []
    assert backend.commands == []


def test_empty_grasp_stops_before_lift_and_never_returns_home():
    motion, backend, grip, profile, poses = setup('empty')
    with pytest.raises(GripError, match='EMPTY_GRASP'):
        execute_pick_place(SafeMover(110, motion, 30, 30), grip, profile, poses, motion.stop, vel=10, acc=10)
    assert motion.commands == [poses['approach'], poses['grasp']]
    assert motion.stopped and backend.halted
    assert len(backend.commands) == 2  # Open once, close once; no auto-release.


def test_cancel_before_start_stops_without_opening_or_moving():
    motion, backend, grip, profile, poses = setup()
    with pytest.raises(GripError, match='CANCELLED'):
        execute_pick_place(SafeMover(110, motion, 30, 30), grip, profile, poses, motion.stop,
                           vel=10, acc=10, cancelled=lambda: True)
    assert motion.commands == [] and backend.commands == []
    assert motion.stopped and backend.halted


def test_mock_demo_runs_all_five_products_without_ros():
    result = run_mock_order(['snack_box', 'pet_bottle', 'can', 'wet_tissue', 'doll'])
    assert result['cycles'] == 5
    assert not result['hardware_connected']
    assert result['motion_commands'] == 37


def test_mixed_mock_gripper_and_real_motion_is_blocked_before_commands():
    motion, backend, grip, profile, poses = setup()
    motion.simulated = False
    with pytest.raises(SafetyLimitError, match='가상 프로필'):
        execute_pick_place(SafeMover(110, motion, 30, 30), grip, profile, poses, motion.stop, vel=10, acc=10)
    assert motion.commands == [] and backend.commands == []


def test_lost_grasp_after_lift_stops_before_transport():
    motion, backend, grip, profile, poses = setup()
    original = motion.movel
    def move(*args):
        result = original(*args)
        if len(motion.commands) == 3:
            backend.current = 0
        return result
    motion.movel = move
    with pytest.raises(GripError, match='GRASP_LOST'):
        execute_pick_place(SafeMover(110, motion, 30, 30), grip, profile, poses, motion.stop, vel=10, acc=10)
    assert len(motion.commands) == 3
    assert motion.stopped and backend.halted
    assert len(backend.commands) == 2

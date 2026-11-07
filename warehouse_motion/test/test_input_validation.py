"""Independent review probes. Uses fake backends only; never imports ROS drivers."""
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

SOURCE = Path(__file__).resolve().parents[1]
from warehouse_motion import config_loader as config
from warehouse_motion.safe_move import DR_BASE, SafeMover, SafetyLimitError, DsrBackend, tool_offset_to_base


class Backend:
    def __init__(self):
        self.sent = []

    def movel(self, *args):
        self.sent.append(args)
        return 0


@pytest.mark.parametrize('field', ['floor_z', 'safety_margin', 'nfc_protrusion'])
def test_nonfinite_floor_cannot_disable_height_guard(field):
    cfg = {'floor': {'floor_z': 100.0, 'safety_margin': 10.0, 'nfc_protrusion': 0.0}}
    cfg['floor'][field] = float('nan')  # YAML's .nan produces this same value.
    backend = Backend()
    with pytest.raises((config.ConfigError, SafetyLimitError, ValueError)):
        limit = config.floor_limit(cfg, 'real')
        SafeMover(limit.z_min, backend, 30, 30).movel([0, 0, -100, 0, 180, 0], 30, 30, DR_BASE)
    assert not backend.sent


def test_nonfinite_zmin_cannot_disable_height_guard():
    backend = Backend()
    with pytest.raises((SafetyLimitError, ValueError)):
        SafeMover(float('nan'), backend, 30, 30).movel([0, 0, -100, 0, 180, 0], 30, 30, DR_BASE)
    assert not backend.sent


@pytest.mark.parametrize('which', ['max_vel', 'max_acc'])
def test_nonfinite_speed_limit_cannot_disable_cap(which):
    backend = Backend()
    kwargs = dict(max_vel=30, max_acc=30)
    kwargs[which] = float('nan')
    speed = (999, 30) if which == 'max_vel' else (30, 999)
    with pytest.raises((SafetyLimitError, ValueError)):
        SafeMover(110, backend, **kwargs).movel([400, 0, 300, 0, 180, 0], *speed, DR_BASE)
    assert not backend.sent


@pytest.mark.parametrize('vel,acc', [
    (float('nan'), 30), (30, float('nan')),
    ([30, float('nan')], [30, 30]), ([30, 30], [30, float('nan')]),
])
def test_nonfinite_speed_is_rejected_before_backend(vel, acc):
    backend = Backend()
    with pytest.raises((SafetyLimitError, ValueError)):
        SafeMover(110, backend, 30, 30).movel([400, 0, 300, 0, 180, 0], vel, acc, DR_BASE)
    assert not backend.sent


@pytest.mark.parametrize('kind', ['pose', 'slot', 'product'])
def test_quoted_false_is_not_a_taught_pose(kind):
    poses = config.load_yaml(SOURCE / 'config' / 'poses.yaml')
    products = config.load_yaml(SOURCE / 'config' / 'products.yaml')
    pose = {'posx': [400, 0, 300, 0, 180, 0], 'taught': 'false'}
    poses['poses']['home'] = copy.deepcopy(pose)
    poses['box_slots'][1] = copy.deepcopy(pose)
    products['products']['can']['real'] = dict(x=400, y=0, rz=0, grasp_height_from_floor=40, taught='false')
    with pytest.raises(config.ConfigError):
        if kind == 'pose':
            config.get_pose(poses, 'home', 'real')
        elif kind == 'slot':
            config.get_box_slot(poses, 1, 'real')
        else:
            config.get_product(products, 'can', 'real', config.FloorLimit(100, 10, 0))


@pytest.mark.parametrize('reply,expected', [
    (None, None), (-1, None), ((None, None), None),
    (([1, 2, 3], 0), None), (([1, 2, 3, 4, 5, 6], 0), [1, 2, 3, 4, 5, 6]),
])
def test_dsr_current_pose_return_contract_without_ros(reply, expected):
    backend = DsrBackend.__new__(DsrBackend)
    backend._dr = SimpleNamespace(get_current_posx=lambda ref: reply)
    assert backend.get_current_posx(DR_BASE) == expected


@pytest.mark.parametrize('reply,expected', [
    (None, None), (-1, None), ([1, 2, 3], None), ([1, 2, 3, 4, 5, 6], [1, 2, 3, 4, 5, 6]),
])
def test_dsr_fkin_return_contract_without_ros(reply, expected):
    backend = DsrBackend.__new__(DsrBackend)
    backend._dr = SimpleNamespace(fkin=lambda posj, ref: reply)
    assert backend.fkin([0] * 6, DR_BASE) == expected


@pytest.mark.parametrize('pose,delta,expected', [
    ([0, 0, 0, 90, 0, 0], [10, 0, 0], [0, 10, 0]),
    ([0, 0, 0, 0, 90, 0], [0, 0, 10], [10, 0, 0]),
    ([0, 0, 0, 0, 180, 0], [0, 0, 10], [0, 0, -10]),
])
def test_tool_axis_conversion_independent_examples(pose, delta, expected):
    assert tool_offset_to_base(pose, delta) == pytest.approx(expected, abs=1e-9)

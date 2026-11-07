"""config_loader 단위 테스트 — 배포되는 YAML 을 그대로 읽어 검사한다."""
import copy
import os

import pytest

from warehouse_motion import config_loader as c

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


@pytest.fixture
def poses():
    return c.load_yaml(os.path.join(CONFIG, 'poses.yaml'))


@pytest.fixture
def products():
    return c.load_yaml(os.path.join(CONFIG, 'products.yaml'))


def test_real_mode_blocked_until_floor_measured(poses):
    p = copy.deepcopy(poses)
    p['floor'].update(floor_z=None, nfc_protrusion=0.0)
    with pytest.raises(c.ConfigError, match='floor_z'):
        c.floor_limit(p, 'real')


def test_real_mode_blocked_until_nfc_measured(poses):
    p = copy.deepcopy(poses)
    p['floor'].update(floor_z=12.0, nfc_protrusion=None)
    with pytest.raises(c.ConfigError, match='nfc_protrusion'):
        c.floor_limit(p, 'real')


def test_lower_limit_adds_margin_and_nfc(poses):
    p = copy.deepcopy(poses)
    p['floor'].update(floor_z=12.0, nfc_protrusion=7.0)
    assert c.floor_limit(p, 'real').z_min == pytest.approx(12.0 + 10.0 + 7.0)


def test_negative_nfc_rejected(poses):
    p = copy.deepcopy(poses)
    p['floor'].update(floor_z=12.0, nfc_protrusion=-3.0)
    with pytest.raises(c.ConfigError):
        c.floor_limit(p, 'real')


def test_untaught_pose_and_slot_rejected_in_real(poses):
    with pytest.raises(c.ConfigError, match='교시'):
        c.get_pose(poses, 'home', 'real')
    with pytest.raises(c.ConfigError, match='교시'):
        c.get_box_slot(poses, 1, 'real')


def test_untaught_product_rejected_in_real(poses, products):
    p = copy.deepcopy(poses)
    p['floor'].update(floor_z=12.0, nfc_protrusion=0.0)
    lim = c.floor_limit(p, 'real')
    with pytest.raises(c.ConfigError, match='교시'):
        c.get_product(products, 'can', 'real', lim)


def test_virtual_products_all_above_limit(poses, products):
    lim = c.floor_limit(poses, 'virtual')
    for name in products['products']:
        t = c.get_product(products, name, 'virtual', lim)
        assert t.grasp_z >= lim.z_min


def test_product_below_limit_is_refused(poses, products):
    lim = c.floor_limit(poses, 'virtual')        # floor 100 + margin 10
    pr = copy.deepcopy(products)
    pr['products']['can']['virtual']['grasp_height_from_floor'] = 5.0   # z = 105 < 110
    with pytest.raises(c.ConfigError, match='하한선'):
        c.get_product(pr, 'can', 'virtual', lim)


def test_unknown_mode_rejected(poses):
    with pytest.raises(c.ConfigError):
        c.floor_limit(poses, 'sim')

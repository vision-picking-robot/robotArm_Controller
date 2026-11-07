"""safe_move 단위 테스트 — 로봇·ROS 없이 실행된다.

가짜 backend 가 받은 명령을 기록하므로, '차단된 이동은 실제로 전송되지 않았다'를 확인할 수 있다.
실행: cd ~/ros2_ws && python3 -m pytest src/robotArm_Controller/warehouse_motion/test -v
"""
import math

import pytest

from warehouse_motion.safe_move import (
    DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_TOOL, DR_WORLD,
    MotionError, SafeMover, SafetyLimitError, tool_offset_to_base, zyz_rotation,
)

Z_MIN = 110.0
DOWN = [0.0, 180.0, 0.0]          # 그리퍼가 아래를 보는 자세 (툴 z축 = base -z)


class FakeBackend:
    def __init__(self, posx=None, posj=None, fkin_result=None, ret=0):
        self.posx = posx if posx is not None else [400.0, 0.0, 200.0] + DOWN
        self.posj = posj if posj is not None else [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.fkin_result = fkin_result
        self.fkin_calls = []
        self.sent = []
        self.ret = ret

    def get_current_posx(self, ref):
        assert ref == DR_BASE
        return None if self.posx is None else list(self.posx)

    def get_current_posj(self):
        return None if self.posj is None else list(self.posj)

    def fkin(self, posj, ref):
        self.fkin_calls.append((list(posj), ref))
        r = self.fkin_result
        return r(posj) if callable(r) else r

    def movel(self, pos, vel, acc, ref, mod, radius):
        self.sent.append(('movel', list(pos), ref, mod))
        return self.ret

    def movej(self, pos, vel, acc, mod, radius):
        self.sent.append(('movej', list(pos), mod))
        return self.ret


class Logger:
    def __init__(self):
        self.errors = []

    def error(self, msg):
        self.errors.append(msg)


def mover(backend, logger=None):
    return SafeMover(Z_MIN, backend, max_vel=30, max_acc=30, logger=logger)


# ---------------- movel 절대 ----------------

def test_movel_abs_below_limit_is_blocked_and_not_sent():
    be, log = FakeBackend(), Logger()
    with pytest.raises(SafetyLimitError):
        mover(be, log).movel([400, 0, 100] + DOWN, vel=30, acc=30, ref=DR_BASE)
    assert be.sent == []
    assert 'z=100.00' in log.errors[0] and 'z_min=110.00' in log.errors[0]


def test_movel_abs_at_limit_is_allowed():
    be = FakeBackend()
    mover(be).movel([400, 0, Z_MIN] + DOWN, vel=30, acc=30, ref=DR_BASE)
    assert len(be.sent) == 1


def test_movel_abs_above_limit_is_sent():
    be = FakeBackend()
    mover(be).movel([400, 0, 300] + DOWN, vel=[30, 30], acc=[30, 30], ref=DR_BASE)
    assert be.sent == [('movel', [400, 0, 300] + DOWN, DR_BASE, DR_MV_MOD_ABS)]


@pytest.mark.parametrize('ref', [DR_TOOL, DR_WORLD, 101, None])
def test_movel_abs_non_base_ref_is_rejected(ref):
    be = FakeBackend()
    with pytest.raises(SafetyLimitError):
        mover(be).movel([400, 0, 300] + DOWN, vel=30, acc=30, ref=ref)
    assert be.sent == []


def test_movel_requires_ref_argument():
    with pytest.raises(TypeError):
        mover(FakeBackend()).movel([400, 0, 300] + DOWN, vel=30, acc=30)


# ---------------- movel 상대 ----------------

def test_movel_rel_base_uses_current_position():
    be = FakeBackend(posx=[400, 0, 150] + DOWN)
    with pytest.raises(SafetyLimitError):                      # 150 - 60 = 90 < 110
        mover(be).movel([0, 0, -60, 0, 0, 0], vel=30, acc=30, ref=DR_BASE, mod=DR_MV_MOD_REL)
    assert be.sent == []
    target = mover(be).movel([0, 0, -40, 0, 0, 0], vel=30, acc=30, ref=DR_BASE,
                             mod=DR_MV_MOD_REL)                # 150 - 40 = 110 → 허용
    assert target[2] == pytest.approx(110)
    assert len(be.sent) == 1


def test_movel_rel_tool_down_facing_plus_z_goes_down_in_base():
    # 툴이 아래를 볼 때 툴 +z 이동은 base 에서 아래로 내려간다
    be = FakeBackend(posx=[400, 0, 150] + DOWN)
    with pytest.raises(SafetyLimitError):                      # 150 - 60 = 90
        mover(be).movel([0, 0, 60, 0, 0, 0], vel=30, acc=30, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    assert be.sent == []


def test_movel_rel_tool_minus_z_goes_up_and_is_allowed():
    be = FakeBackend(posx=[400, 0, 150] + DOWN)
    target = mover(be).movel([0, 0, -100, 0, 0, 0], vel=30, acc=30, ref=DR_TOOL,
                             mod=DR_MV_MOD_REL)
    assert target[2] == pytest.approx(250)
    assert len(be.sent) == 1


def test_movel_rel_fails_closed_when_posx_unreadable():
    be = FakeBackend()
    be.posx = None
    with pytest.raises(SafetyLimitError):
        mover(be).movel([0, 0, 10, 0, 0, 0], vel=30, acc=30, ref=DR_BASE, mod=DR_MV_MOD_REL)
    assert be.sent == []


def test_movel_rel_world_ref_is_rejected():
    be = FakeBackend()
    with pytest.raises(SafetyLimitError):
        mover(be).movel([0, 0, 10, 0, 0, 0], vel=30, acc=30, ref=DR_WORLD, mod=DR_MV_MOD_REL)
    assert be.sent == []


# ---------------- movej ----------------

def test_movej_blocked_when_fkin_target_below_limit():
    be = FakeBackend(fkin_result=[400, 0, 80] + DOWN)
    with pytest.raises(SafetyLimitError):
        mover(be).movej([0, 30, 100, 0, 50, 0], vel=30, acc=30)
    assert be.sent == []
    assert be.fkin_calls == [([0, 30, 100, 0, 50, 0], DR_BASE)]


def test_movej_allowed_when_fkin_target_above_limit():
    be = FakeBackend(fkin_result=[373, 0, 405] + DOWN)
    mover(be).movej([0, 0, 90, 0, 90, 0], vel=30, acc=30)
    assert be.sent == [('movej', [0, 0, 90, 0, 90, 0], DR_MV_MOD_ABS)]


def test_movej_rel_adds_current_joints_before_fkin():
    be = FakeBackend(posj=[10, 0, 90, 0, 90, 0], fkin_result=[373, 0, 405] + DOWN)
    mover(be).movej([5, 0, -10, 0, 0, 0], vel=30, acc=30, mod=DR_MV_MOD_REL)
    assert be.fkin_calls[0][0] == [15, 0, 80, 0, 90, 0]
    # 컨트롤러에는 원래의 상대값이 그대로 전달된다
    assert be.sent == [('movej', [5, 0, -10, 0, 0, 0], DR_MV_MOD_REL)]


@pytest.mark.parametrize('bad', [None, [1, 2, 3]])
def test_movej_fails_closed_when_fkin_fails(bad):
    be = FakeBackend(fkin_result=bad)
    with pytest.raises((SafetyLimitError, ValueError)):
        mover(be).movej([0, 0, 90, 0, 90, 0], vel=30, acc=30)
    assert be.sent == []


# ---------------- 속도·입력 검사 ----------------

@pytest.mark.parametrize('vel,acc', [(None, 30), (30, None), (31, 30), (30, 31),
                                     ([30, 50], 30), (0, 30), (-5, 30)])
def test_speed_limits_are_enforced(vel, acc):
    be = FakeBackend()
    with pytest.raises(SafetyLimitError):
        mover(be).movel([400, 0, 300] + DOWN, vel=vel, acc=acc, ref=DR_BASE)
    assert be.sent == []


@pytest.mark.parametrize('pos', [[1, 2, 3], [1, 2, 3, 4, 5, float('nan')], 'abc'])
def test_invalid_target_is_rejected(pos):
    be = FakeBackend()
    with pytest.raises(ValueError):
        mover(be).movel(pos, vel=30, acc=30, ref=DR_BASE)
    assert be.sent == []


def test_controller_failure_raises_motion_error():
    be = FakeBackend(ret=-1)
    with pytest.raises(MotionError):
        mover(be).movel([400, 0, 300] + DOWN, vel=30, acc=30, ref=DR_BASE)


# ---------------- 회전 변환 ----------------

def test_zyz_identity():
    r = zyz_rotation(0, 0, 0)
    for i in range(3):
        for j in range(3):
            assert r[i][j] == pytest.approx(1.0 if i == j else 0.0)


def test_tool_offset_down_facing():
    d = tool_offset_to_base([0, 0, 0] + DOWN, [0, 0, 10])
    assert d == pytest.approx([0, 0, -10], abs=1e-9)


def test_zyz_rotation_is_orthonormal():
    r = zyz_rotation(30, 70, -45)
    for i in range(3):
        for j in range(3):
            dot = sum(r[i][k] * r[j][k] for k in range(3))
            assert dot == pytest.approx(1.0 if i == j else 0.0, abs=1e-12)
    assert not any(math.isnan(v) for row in r for v in row)

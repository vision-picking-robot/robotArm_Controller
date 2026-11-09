import pytest

from warehouse_motion.gripper_feedback import parse_feedback, wait_for_feedback
from warehouse_motion.safe_move import MotionError


class Clock:
    now = 0.0

    def sleep(self, seconds):
        self.now += seconds


def status(position=0, moving=0, current=0):
    return f'position_pulses={position} current={current} moving={moving} moving_status=49'


def test_moving_status_does_not_prevent_open_completion():
    clock = Clock()
    calls = []

    def read(remaining):
        calls.append(remaining)
        return status()

    wait_for_feedback(read, lambda p, _c: abs(p) <= 10, 'open',
                      clock=lambda: clock.now, sleep=clock.sleep)
    assert len(calls) == 2
    assert clock.now == 0.1
    assert calls[1] < calls[0]


def test_grasp_requires_stable_position_and_current_after_motion():
    clock = Clock()
    replies = iter([status(300, 1, -100), status(300, 0, -100),
                    status(300, 1, -100), status(300, 0, -100), status(300, 0, -100)])
    wait_for_feedback(lambda _remaining: next(replies), lambda p, c: 250 <= p <= 350 and c >= 80,
                      'grasp', clock=lambda: clock.now, sleep=clock.sleep)
    assert clock.now == pytest.approx(0.4)


@pytest.mark.parametrize('reply', [status(740), status(300, 0, 0), status(300, 1, 400)])
def test_empty_low_current_or_still_moving_does_not_confirm_grasp(reply):
    clock = Clock()
    with pytest.raises(MotionError, match='did not confirm grasp'):
        wait_for_feedback(lambda _remaining: reply, lambda p, c: 250 <= p <= 350 and c >= 80,
                          'grasp', timeout=0.25, clock=lambda: clock.now, sleep=clock.sleep)
    assert clock.now == 0.25


@pytest.mark.parametrize('message', [
    '', 'bad', 'moving=0 current=1', status(moving=256), status(moving=-1),
    'position_pulses=nan current=0 moving=0', status() + ' moving=0', None,
])
def test_bad_or_old_packed_feedback_fails_closed(message):
    with pytest.raises(MotionError):
        parse_feedback(message)


def test_communication_failure_does_not_retry():
    calls = []

    def failed(_remaining):
        calls.append('status')
        raise TimeoutError('reply lost')

    with pytest.raises(TimeoutError, match='reply lost'):
        wait_for_feedback(failed, lambda _p, _c: True, 'open')
    assert calls == ['status']


def test_reply_arriving_after_deadline_cannot_confirm():
    clock = Clock()

    def late(_remaining):
        clock.now += 1.0
        return status()

    with pytest.raises(MotionError, match='did not confirm'):
        wait_for_feedback(late, lambda _p, _c: True, 'open', timeout=0.5,
                          clock=lambda: clock.now, sleep=clock.sleep)

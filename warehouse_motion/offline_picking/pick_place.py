"""One pick/place cycle. Order lookup, inventory writes and recovery are external."""
from .gripper import GripError
from warehouse_motion.safe_move import DR_BASE, SafetyLimitError
from warehouse_motion.validation import pose6


def execute_pick_place(mover, gripper, profile, poses, stop_motion, *, vel, acc,
                       cancelled=lambda: False, on_step=None):
    """Blocking core for a future ROS action worker; not a ROS callback.

    Motion backend must expose the actual stop request via stop_motion.
    All Cartesian targets are Base/mm/Doosan ZYZ-degrees. Completion verifies
    jaw opening, not independent visual confirmation that the item was deposited.
    """
    required = ('approach', 'grasp', 'lift', 'via', 'place_approach', 'place', 'retract')
    if profile.simulation_only and getattr(mover.backend, 'simulated', False) is not True:
        raise SafetyLimitError('가상 프로필로 실물 이동 백엔드를 사용할 수 없습니다')
    targets = {name: pose6(poses[name], name) for name in required}
    for name, pose in targets.items():
        if pose[2] < mover.z_min:
            raise SafetyLimitError(f'{name}: 목표 z가 하한선보다 낮습니다')
    steps = []

    def record(name):
        steps.append(name)
        if on_step is not None:
            on_step(name)

    def check():
        if cancelled():
            raise GripError('CANCELLED', '작업 취소 요청')

    def move(name):
        check()
        mover.movel(targets[name], vel=vel, acc=acc, ref=DR_BASE)
        record(name)

    try:
        check()
        gripper.open(profile, cancelled)
        record('open')
        move('approach')
        move('grasp')
        check()
        gripper.grasp(profile, cancelled)
        record('grip_confirmed')
        move('lift')
        gripper.check_hold(profile)
        record('hold_checked')
        move('via')
        move('place_approach')
        move('place')
        check()
        gripper.open(profile, cancelled)
        record('jaw_opened')
        move('retract')
        return steps
    except Exception as cause:
        failures = []
        for stop in (stop_motion, gripper.backend.halt):
            try:
                stop()
            except Exception as exc:
                failures.append(str(exc))
        if failures:
            raise RuntimeError(f'원인: {cause}; 정지 실패: {failures}') from cause
        raise

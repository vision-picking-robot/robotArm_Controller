"""Complete scenario using in-memory fixtures and fake motion only."""
import argparse
import copy
from dataclasses import asdict
import json
import os
import threading

from .config_loader import default_config_dir, floor_limit, get_product, load_yaml
from .contracts import PickRequest, identifier
from .gripper import GripController, GripError, get_grip_profile
from .mock_demo import MockMotion, MockGripper, SimClock, mock_plan
from .pick_place import execute_pick_place
from .safe_move import DR_BASE, SafeMover


CASES = ('normal', 'unknown-tag', 'missing-product', 'empty-grasp', 'unconfirmed-place', 'cancel')


class MockScenario:
    """One simulated box at a time. Never constructs a real backend.

    shortcut: replay history lasts for this mock process only; persist execution
    records and reconcile physical state before implementing live retries.
    """
    def __init__(self, config_dir=None):
        directory = config_dir or default_config_dir()
        self.config = load_yaml(os.path.join(directory, 'mock_scenario.yaml'))
        if self.config.get('mode') != 'mock':
            raise ValueError('모의 시나리오 설정만 허용합니다')
        self.products = load_yaml(os.path.join(directory, 'products.yaml'))
        self.grips = load_yaml(os.path.join(directory, 'gripper_profiles.yaml'))
        self.limit = floor_limit(load_yaml(os.path.join(directory, 'poses.yaml')), 'virtual')
        self._lock = threading.Lock()
        self._history = {}

    def run(self, request_id, uid='000000A1', case='normal', cancelled=lambda: False, on_event=None):
        identifier(request_id, 'request_id')
        identifier(uid, 'uid')
        if case not in CASES:
            raise ValueError('알 수 없는 모의 사례')
        if not self._lock.acquire(blocking=False):
            raise RuntimeError('BUSY: 다른 모의 작업이 실행 중입니다')
        try:
            signature = (uid, case)
            if request_id in self._history:
                prior, result = self._history[request_id]
                if signature != prior:
                    raise ValueError('request_id가 다른 요청에 이미 사용되었습니다')
                return {**copy.deepcopy(result), 'replayed': True}
            result = self._run(request_id, uid, case, cancelled, on_event)
            self._history[request_id] = (signature, copy.deepcopy(result))
            return result
        finally:
            self._lock.release()

    def _run(self, request_id, uid, case, cancelled, on_event):
        trace, requests, confirmed = [], [], []
        clock, motion = SimClock(), MockMotion()
        mover = SafeMover(self.limit.z_min, motion, 30, 30)
        home = [350, 0, 500, 0, 180, 0]
        grip = None
        current_steps = []
        forced_cancel = False
        code = None
        status = 'completed'
        order_id = box_id = None
        target_quantity = 0

        def is_cancelled(): return forced_cancel or cancelled()

        def emit(phase, *, notify=True, **fields):
            event = dict(sequence=len(trace) + 1, phase=phase, **fields)
            trace.append(event)
            if notify and on_event is not None:
                try:
                    on_event(copy.deepcopy(event))
                except Exception as exc:
                    raise GripError('OBSERVER_FAILED', '상태 수신 콜백이 실패했습니다') from exc

        def check():
            if is_cancelled():
                raise GripError('CANCELLED', '모의 작업 취소')

        def move(phase, pose):
            check()
            emit(phase)
            mover.movel(pose, 10, 10, ref=DR_BASE)

        try:
            emit('WAIT_BOX_TRIGGER')
            check()
            emit('BOX_TRIGGERED')
            move('SCAN_APPROACH', [300, -250, 400, 0, 180, 0])
            move('SCAN_RFID', [300, -250, 250, 0, 180, 0])
            emit('RFID_READ', simulated=True)
            check()
            order = None if case == 'unknown-tag' else self.config['boxes'].get(uid)
            emit('ORDER_LOOKUP')
            if not order:
                raise GripError('UNKNOWN_TAG', '연결된 모의 주문이 없습니다')
            order_id, box_id = order['order_id'], order['box_id']
            for line in order['lines']:
                quantity = line['quantity']
                if type(quantity) is not int or quantity <= 0:
                    raise ValueError('주문 수량은 양의 정수여야 합니다')
                get_product(self.products, line['product_id'], 'virtual', self.limit)
                get_grip_profile(self.grips, line['product_id'], 'mock')
                target_quantity += quantity
            if not 1 <= target_quantity <= 5:
                raise ValueError('모의 장면은 주문당 1~5개 슬롯을 지원합니다')
            for line in order['lines']:
                for _ in range(line['quantity']):
                    current_steps = []
                    move('VISION_CLEAR', home)
                    emit('VISION_DETECT', product_id=line['product_id'])
                    check()
                    if case == 'missing-product':
                        raise GripError('TARGET_NOT_FOUND', '대상 상품 미검출 모의 사례')
                    product = get_product(self.products, line['product_id'], 'virtual', self.limit)
                    slot = len(confirmed) + 1
                    request = PickRequest(
                        f'{request_id}:item:{slot}', order_id, line['product_id'], box_id, slot,
                        (product.x, product.y, product.grasp_z, 0, 180, product.rz))
                    request.require_mock()
                    requests.append(asdict(request))
                    profile = get_grip_profile(self.grips, request.product_id, 'mock')
                    outcome = 'empty' if case == 'empty-grasp' else 'ok'
                    grip = GripController(MockGripper(clock, profile, outcome), clock.now, clock.sleep)

                    def step(name):
                        nonlocal forced_cancel
                        current_steps.append(name)
                        emit('PICK_PLACE', step=name, product_id=request.product_id, slot_id=slot)
                        if case == 'cancel' and name == 'grip_confirmed':
                            forced_cancel = True

                    execute_pick_place(mover, grip, profile, mock_plan(request.grasp_posx, slot),
                                       motion.stop, vel=10, acc=10, cancelled=is_cancelled, on_step=step)
                    emit('VERIFY_PLACE', slot_id=slot)
                    check()
                    if case == 'unconfirmed-place':
                        raise GripError('PLACE_UNCONFIRMED', '손가락은 열렸으나 적치는 확인되지 않았습니다')
                    confirmed.append(dict(product_id=request.product_id, slot_id=slot, quantity=1))
                    current_steps = []
                    emit('PLACE_CONFIRMED', slot_id=slot, confirmed_quantity=len(confirmed))
                    emit('CHECK_REMAINING', remaining=target_quantity - len(confirmed))
            move('RETURN_HOME', home)
            emit('IDLE')
        except Exception as exc:
            code = exc.code if isinstance(exc, GripError) else 'INVALID_INPUT'
            status = 'cancelled' if code == 'CANCELLED' else 'failed'
            if code == 'PLACE_UNCONFIRMED': status = 'needs_reconciliation'
            motion.stop()
            if grip is not None: grip.backend.halt()
            emit('CANCELLED' if code == 'CANCELLED' else 'ERROR', notify=False, error_code=code)
        requires_reconciliation = (status != 'completed' and 'grip_confirmed' in current_steps)
        return dict(protocol_version=1, request_id=request_id, mode='mock', status=status,
                    order_id=order_id, box_id=box_id, target_quantity=target_quantity,
                    confirmed_quantity=len(confirmed), placements=confirmed, requests=requests,
                    error_code=code, requires_reconciliation=requires_reconciliation,
                    replayed=False, trace=trace, motion_commands=len(motion.commands),
                    hardware_connected=False)


def main():
    parser = argparse.ArgumentParser(description='RFID·주문·비전·피킹 전체 모의 시나리오')
    parser.add_argument('--case', choices=CASES, default='normal')
    parser.add_argument('--config-dir')
    args = parser.parse_args()
    result = MockScenario(args.config_dir).run('DEMO-RUN-01', case=args.case)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['status'] == 'completed' else 1


if __name__ == '__main__': raise SystemExit(main())

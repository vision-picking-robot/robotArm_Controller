from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import threading

import pytest

from warehouse_motion.contracts import PickRequest
from warehouse_motion.mock_scenario import MockScenario


def test_scenario_follows_order_and_confirms_three_placements_before_home():
    result = MockScenario().run('run-1')
    assert result['status'] == 'completed'
    assert result['target_quantity'] == result['confirmed_quantity'] == 3
    assert [p['product_id'] for p in result['placements']] == ['snack_box', 'pet_bottle', 'wet_tissue']
    assert [p['slot_id'] for p in result['placements']] == [1, 2, 3]
    phases = [e['phase'] for e in result['trace']]
    assert phases[:6] == ['WAIT_BOX_TRIGGER', 'BOX_TRIGGERED', 'SCAN_APPROACH', 'SCAN_RFID', 'RFID_READ', 'ORDER_LOOKUP']
    assert phases.count('VISION_CLEAR') == 3
    assert phases.count('VISION_DETECT') == 3
    assert phases[-2:] == ['RETURN_HOME', 'IDLE']
    assert len([p for p in phases if p == 'VERIFY_PLACE']) == 3
    assert result['hardware_connected'] is False
    assert all(r['simulation'] is True for r in result['requests'])


@pytest.mark.parametrize('case,code,status', [
    ('unknown-tag', 'UNKNOWN_TAG', 'failed'),
    ('missing-product', 'TARGET_NOT_FOUND', 'failed'),
    ('empty-grasp', 'EMPTY_GRASP', 'failed'),
    ('unconfirmed-place', 'PLACE_UNCONFIRMED', 'needs_reconciliation'),
    ('cancel', 'CANCELLED', 'cancelled'),
])
def test_failure_cases_do_not_count_placement_or_return_home(case, code, status):
    result = MockScenario().run('run-error', case=case)
    assert result['status'] == status
    assert result['error_code'] == code
    assert result['confirmed_quantity'] == 0
    assert result['placements'] == []
    assert 'RETURN_HOME' not in [e['phase'] for e in result['trace']]
    if case in ('unconfirmed-place', 'cancel'):
        assert result['requires_reconciliation'] is True


def test_replayed_request_does_not_emit_or_move_again():
    scenario = MockScenario()
    first = scenario.run('same')
    events = []
    replay = scenario.run('same', on_event=events.append)
    assert replay['replayed'] is True
    assert events == []
    assert replay['motion_commands'] == first['motion_commands']
    assert replay['confirmed_quantity'] == 3
    replay['placements'].clear()
    assert len(scenario.run('same')['placements']) == 3
    with pytest.raises(ValueError, match='request_id'):
        scenario.run('same', uid='000000A2')


def test_ambiguous_result_is_replayed_without_automatic_retry():
    scenario = MockScenario()
    first = scenario.run('ambiguous', case='unconfirmed-place')
    events = []
    second = scenario.run('ambiguous', case='unconfirmed-place', on_event=events.append)
    assert first['status'] == second['status'] == 'needs_reconciliation'
    assert second['replayed'] is True and events == []


def test_order_quantity_repeats_pick_and_uses_distinct_slots():
    scenario = MockScenario()
    order = scenario.config['boxes']['000000A1']
    order['lines'] = [dict(product_id='snack_box', quantity=2), dict(product_id='can', quantity=1)]
    result = scenario.run('quantity')
    assert result['status'] == 'completed'
    assert result['confirmed_quantity'] == 3
    assert [p['product_id'] for p in result['placements']] == ['snack_box', 'snack_box', 'can']
    assert [p['slot_id'] for p in result['placements']] == [1, 2, 3]


def test_failure_on_second_item_keeps_only_prior_confirmed_count():
    scenario = MockScenario()
    cancel = threading.Event()
    def observe(event):
        if event['phase'] == 'PLACE_CONFIRMED' and event['confirmed_quantity'] == 1:
            cancel.set()
    result = scenario.run('partial', cancelled=cancel.is_set, on_event=observe)
    assert result['status'] == 'cancelled'
    assert result['confirmed_quantity'] == 1
    assert len(result['placements']) == 1
    assert result['requires_reconciliation'] is False
    assert 'RETURN_HOME' not in [e['phase'] for e in result['trace']]


def test_observer_failure_stops_and_caches_result_without_reexecution():
    scenario = MockScenario()
    def observe(event):
        if event.get('step') == 'grip_confirmed':
            raise OSError('dashboard unavailable')
    first = scenario.run('observer-failed', on_event=observe)
    assert first['error_code'] == 'OBSERVER_FAILED'
    assert first['confirmed_quantity'] == 0
    assert first['requires_reconciliation'] is True
    assert first['trace'][-1]['phase'] == 'ERROR'
    events = []
    replay = scenario.run('observer-failed', on_event=events.append)
    assert replay['replayed'] is True and events == []


def test_cancel_during_placement_verification_does_not_increment_count():
    cancel = threading.Event()
    def observe(event):
        if event['phase'] == 'VERIFY_PLACE':
            cancel.set()
    result = MockScenario().run('cancel-verify', cancelled=cancel.is_set, on_event=observe)
    assert result['status'] == 'cancelled'
    assert result['confirmed_quantity'] == 0
    assert result['requires_reconciliation'] is True


def test_another_request_is_rejected_while_one_is_running():
    scenario = MockScenario()
    started, proceed = threading.Event(), threading.Event()
    def observe(event):
        if event['phase'] == 'WAIT_BOX_TRIGGER':
            started.set()
            assert proceed.wait(timeout=5)
    with ThreadPoolExecutor(max_workers=1) as pool:
        future = pool.submit(scenario.run, 'first', on_event=observe)
        try:
            assert started.wait(timeout=5)
            with pytest.raises(RuntimeError, match='BUSY'):
                scenario.run('second')
        finally:
            proceed.set()
        assert future.result(timeout=5)['status'] == 'completed'


@pytest.mark.parametrize('quantity', [0, -1, True, 1.5, 6])
def test_invalid_or_over_capacity_order_never_starts_picking(quantity):
    scenario = MockScenario()
    scenario.config['boxes']['000000A1']['lines'] = [dict(product_id='snack_box', quantity=quantity)]
    result = scenario.run('invalid-quantity')
    assert result['status'] == 'failed'
    assert result['confirmed_quantity'] == 0
    assert result['requests'] == []


@pytest.mark.parametrize('change', [
    dict(frame_id='camera'), dict(position_unit='m'), dict(orientation='rpy'),
    dict(grasp_posx=[0, 0, float('nan'), 0, 0, 0]), dict(slot_id=True),
    dict(simulation='true'), dict(request_id=''),
])
def test_motion_contract_rejects_ambiguous_frames_units_or_values(change):
    request = PickRequest('id', 'order', 'snack_box', 'BOX-01', 1, (400, 0, 200, 0, 180, 0))
    with pytest.raises(ValueError):
        replace(request, **change)


def test_real_request_cannot_be_dispatched_to_mock_runner():
    request = PickRequest('id', 'order', 'snack_box', 'BOX-01', 1, (400, 0, 200, 0, 180, 0), simulation=False)
    with pytest.raises(ValueError, match='모의 요청'):
        request.require_mock()

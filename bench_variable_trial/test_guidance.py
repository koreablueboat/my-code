from types import SimpleNamespace

import pytest

from guidance import Guide, Incomplete


def sample(**fields):
    return dict(dict(deadline_mono=20., read_counts=None, oe_low_requested=False,
                     armed=False, oe_request_ns=0, readback_ns=None), **fields)


def test_operator_sequence():
    g = Guide()
    rc = SimpleNamespace(permit=False, sticks_neutral=True)
    low = {'phase':'KILLED', 'reason':'radio_kill'}
    g.advance(.1, sample(), low, rc, False)
    g.advance(1.21, sample(), low, rc, False)
    assert g.phase == 'SA 허용'
    rc.permit = True
    g.advance(2, sample(), low, rc, False)
    assert g.phase == '재무장 확인'
    running = {'phase':'RUNNING'}
    on = sample(armed=True, oe_low_requested=True, oe_request_ns=4_000_000_000,
                read_counts=[307,307], readback_ns=4_000_000_000)
    g.advance(4.1, on, running, rc, False)
    g.advance(7.2, on, running, rc, False)
    assert g.phase == '중립 초기화'  # only an old readback
    on['readback_ns'] = 7_500_000_000
    g.advance(7.51, on, running, rc, False)
    assert g.phase == '직진'
    on['read_counts'] = [315, 315]
    for ns in (8_000_000_000,8_550_000_000,9_150_000_000):
        on['readback_ns'] = ns
        g.advance(ns/1e9+.01, on, running, rc, True)
    assert g.phase == 'SA KILL'
    low['previous_phase'] = 'RUNNING'
    rc.permit = False
    off = sample(oe_request_ns=10_000_000_000)
    assert not g.advance(10.1, off, low, rc, False)
    assert not g.advance(11., off, low, rc, False)
    assert g.advance(11.5, off, low, rc, False)


@pytest.mark.parametrize('phase,now', [('중앙·KILL 유지',11.),('중립 초기화',16.),
                                       ('직진',17.),('SA KILL',18.1),('차단 관찰',19.95)])
def test_too_late(phase,now):
    init = phase == '중립 초기화'
    m = sample(armed=init,oe_low_requested=init,read_counts=[307,307],
               oe_request_ns=int(now*1e9),readback_ns=int(now*1e9))
    with pytest.raises(Incomplete, match='시간 .*부족'):
        Guide(phase).advance(now,m,{},SimpleNamespace(permit=init,sticks_neutral=True),False)


def test_unexpected_cutoff():
    with pytest.raises(Incomplete,match='예정 전'):
        Guide('직진').advance(5,sample(),dict(phase='KILLED',previous_phase='RUNNING',
                            reason='rc_timeout'),SimpleNamespace(),False)


def test_cold_start_after_four_seconds_and_remaining_budget():
    from guidance import awaiting_status
    remaining, stage = awaiting_status(104.13, 100., True)
    assert remaining == pytest.approx(15.87) and '첫 상태' in stage
    with pytest.raises(Incomplete, match='후속 단계'):
        awaiting_status(110., 100., False)


@pytest.mark.parametrize('phase,permit,neutral', [('모터 기동',True,True),
    ('모터 기동',False,False),('재무장 확인',True,False),('중립 초기화',True,False)])
def test_early_input_even_without_status(phase, permit, neutral):
    from guidance import check_operator_input
    with pytest.raises(Incomplete, match='안내 전'):
        check_operator_input(phase, SimpleNamespace(permit=permit, sticks_neutral=neutral))


@pytest.mark.parametrize('name,action,counts,v,w', [
    ('reverse','후진',[304,304],-.009,0),('left','좌회전',[304,310],0,.06),
    ('right','우회전',[310,304],0,-.06)])
def test_direction_and_return(name, action, counts, v, w):
    from guidance import matches_direction
    g = Guide(direction=name, phase='중립 초기화')
    rc = SimpleNamespace(permit=True, sticks_neutral=True)
    m = sample(armed=True,oe_low_requested=True,oe_request_ns=0,readback_ns=3_100_000_000,
               read_counts=[307,307])
    run = {'phase':'RUNNING'}
    g.advance(3.1,m,run,rc,False)
    assert g.phase == action and matches_direction(name,v,w)
    assert not matches_direction(name,-v,-w)
    for phase,neutral,target,start in [(action,False,counts,4.),('중립 복귀',True,[307,307],6.)]:
        rc.sticks_neutral = neutral
        for i in range(4):
            now = start+i*.5
            m.update(readback_ns=round(now*1e9),read_counts=target,target_counts=target)
            assert g.phase == phase
            g.advance(now,m,run,rc,not neutral)
    assert g.phase == 'SA KILL'

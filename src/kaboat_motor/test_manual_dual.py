"""Only the new dual-output mapping, admission and RC integration. No devices."""
import json
from pathlib import Path

import pytest
from manual_dual import validate_dual
from test_rc_fixed_neutral import Trial


@pytest.fixture
def profile():
    return json.loads(Path(__file__).with_name('manual_dual_candidate.json').read_text())


def test_admission(profile):
    validate_dual(profile)
    with pytest.raises(ValueError, match='hardware refused'):
        validate_dual(profile, hardware=True)
    for k in ('esc_spec_verified','oe_default_disabled_verified','independent_stop_verified','signal_loss_stop_verified'):
        profile[k] = True
    with pytest.raises(ValueError, match='mapping is unverified'):
        validate_dual(profile, hardware=True)
    profile['count_mapping']['right']['reverse_start'] = 304
    with pytest.raises(ValueError, match='count envelope'):
        validate_dual(profile)


@pytest.fixture
def dual(monkeypatch, tmp_path, profile):
    # Ephemeral, simulated installation accepted only inside a mocked Node and Bus.
    for key in profile:
        if key.endswith('_verified'):
            profile[key] = True
    profile.update(propulsion_disconnect_model='simulated',approval_record='memory fixture')
    path = tmp_path/'profile.json'
    path.write_text(json.dumps(profile))
    t = Trial(monkeypatch,'manual_dual',profile=path)
    t.rearm()
    for _ in range(65):
        t.tick()
    return t


@pytest.mark.parametrize('steer,throttle,pair', [
    (992,1811,(317,317)),(992,174,(299,299)),(174,992,(298,315)),
    (1811,992,(315,298)),(174,1811,(307,319)),(174,174,(298,316))])
def test_manual_rc_to_registers(dual, steer, throttle, pair):
    t=dual
    t.axes[:2]=[steer,throttle]
    prev=(307,307)
    for _ in range(28):
        t.tick()
        # At 50ms per step the fractional slew accumulator changes by 1.2 ticks.
        assert max(abs(a-b) for a,b in zip(t.gate.read_counts,prev)) <= 2
        prev=t.gate.read_counts
    assert prev==pair
    t.axes[:2]=[992,992]
    t.tick()
    assert t.gate.read_counts==(307,307)
    t.axes[4]=191
    assert t.tick()['reason']=='radio_kill' and not t.gate.enabled


def test_reverse_dwell_and_lost_rc(dual):
    t=dual
    t.axes[1]=1811
    for _ in range(20): t.tick()
    t.axes[1]=174
    for _ in range(8):
        t.tick()
        assert t.gate.read_counts==(307,307)
    for _ in range(20): t.tick()
    assert t.gate.read_counts==(299,299)
    for _ in range(10): t.tick(rc=False)
    assert not t.gate.enabled and not t.node.controller.armed
    t.axes[1]=992
    for _ in range(12): t.tick()
    assert not t.gate.enabled


def test_readback_fault_dual(dual):
    dual.bus.registers[10]=[0,0,0,0]
    with pytest.raises(RuntimeError,match='CH1'):
        dual.tick()
    assert dual.gate.failed and not dual.gate.enabled

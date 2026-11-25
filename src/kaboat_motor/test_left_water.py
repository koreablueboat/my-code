"""New single-output restrictions, using only a memory register map."""
import json
from pathlib import Path
from unittest.mock import Mock
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from bench_pca import LeftWaterOutput, LED0, FULL_OFF, validate_left_water
from boat_motor_node import BoatMotorNode
from pca_backend import PCAOutput
from test_neutral_probe import Bus


def candidate():
    return json.loads(Path(__file__).with_name('left_water_candidate.json').read_text())


def memory_profile():
    p = candidate()
    for key in p:
        if key.endswith('_verified'):
            p[key] = True
    p.update(propulsion_disconnect_model='simulated', approval_record='memory test only')
    return p


def test_scope_and_approvals():
    with pytest.raises(ValueError, match='hardware refused'):
        validate_left_water(candidate())
    p = memory_profile()
    with pytest.raises(ValueError, match='single-channel'):
        PCAOutput(p)
    for key in ('trial_scope', 'approval_record', 'propulsion_disconnect_model'):
        with pytest.raises(ValueError):
            validate_left_water(dict(p, **{key: None}))


def test_left_only_and_cutoff_on_inactive_channel_fault(monkeypatch):
    bus = Bus([], None)
    oe = Mock()
    out = LeftWaterOutput(oe, bus, memory_profile())
    out.enable_neutral(lambda: True)
    assert out.read_counts == (307,)
    assert all(bus.registers[LED0+4*i] == FULL_OFF for i in range(1,16))
    monkeypatch.setattr('bench_pca.time.monotonic', lambda: out.ready_at + .1)
    out.write(.1, 0.)
    assert out.read_counts == (315,)
    out.write(0., 0.)
    assert out.read_counts == (307,)
    bus.registers[LED0+4] = [0, 0, 51, 1]
    with pytest.raises(RuntimeError, match='FULL OFF'):
        out.write(0., 0.)
    assert out.failed and not out.enabled
    oe.set_value.assert_called_with(1)


@pytest.mark.parametrize('v,w,ready', [(-.001,0.,True),(.01,.01,True),(.031,0.,True),(.02,0.,False)])
def test_forbidden_input_cuts_output(v,w,ready):
    node = SimpleNamespace(args=SimpleNamespace(backend='left_t200_water'), bridge=Mock(),
                           controller=Mock(armed=True))
    node.bridge.poll.return_value = ready
    msg = Twist()
    msg.linear.x, msg.angular.z = v,w
    BoatMotorNode.command(node,msg)
    node.bridge.disable.assert_called_once()
    node.controller.disarm.assert_called_once()
    node.controller.command.assert_not_called()

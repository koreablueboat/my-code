"""Single-process RC → supervisor → motor simulation, with no ROS graph or devices."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import bench_pca as pca
import boat_motor_node as motor
from test_neutral_probe import Bus

control_dir = Path(__file__).resolve().parents[1] / 'kaboat_control/scripts'
sys.path.insert(0, str(control_dir))
from supervisor import Supervisor


class Trial:
    def __init__(self, monkeypatch, backend="bench_neutral", profile=None):
        self.now = 10.
        self.log = [('oe', 1)]
        self.bus = Bus(self.log, None)
        oe = Mock()
        oe.set_value.side_effect = lambda value: self.log.append(('oe', value))
        self.backend = backend
        self.gate = pca.FixedNeutralGate(oe, self.bus)
        self.axes = [992] * 16
        self.axes[4:6] = [191, 997]
        self.lq = 100
        self.supervisor = Supervisor(json.loads((control_dir / 'control_config.json').read_text()),
                                     expected_backend=backend)
        monkeypatch.setattr(motor.time, 'monotonic', lambda: self.now)
        monkeypatch.setattr(motor.Node, '__init__', lambda *args: None)
        for name in ('create_subscription', 'create_service', 'create_timer', 'get_logger'):
            monkeypatch.setattr(motor.Node, name, Mock())
        monkeypatch.setattr(motor.Node, 'create_publisher', lambda *args: Mock())
        monkeypatch.setattr(motor.Node, 'count_publishers', lambda *args: 1)
        args = SimpleNamespace(backend=backend, duration=120. if backend == 'bench_neutral' else 20., test_estop_gpio=False,
                               profile=Path(motor.__file__).with_name('preview_bidirectional.json'))
        if backend in ('bench_variable','manual_dual'):
            args.profile = profile or args.profile.with_name('replacement_board_candidate.json')
            self.node = motor.BoatMotorNode(args, (oe, self.bus))
            self.gate = self.node.bridge
        else:
            self.node = motor.BoatMotorNode(args, self.gate)
        self.node.status_pub.publish.side_effect = self.record_status

    def record_status(self, msg):
        state = json.loads(msg.data)
        assert state['backend'] == self.backend
        assert state['oe_low_requested'] == self.gate.enabled
        if self.backend == 'bench_neutral':
            assert state['fixed_neutral_bench'] and state['logical_left'] == state['logical_right'] == 0.
        self.supervisor.set_motor(state, self.now)

    def tick(self, *, rc=True, lq=True):
        self.now += .05
        if lq:
            self.supervisor.set_lq(self.lq, self.now)
        if rc:
            self.supervisor.set_rc(self.axes, self.now)
        state = self.supervisor.tick(self.now)
        self.node.estop_cb(Bool(data=state['estop']))
        twist = Twist()
        twist.linear.x, twist.angular.z = state['v'], state['w']
        self.node.command(twist)
        action = state['action']
        if action:  # In-memory handshake, no service clients.
            response = SimpleNamespace()
            if action == 'reset':
                self.node.reset_cb(None, response)
            else:
                self.node.arm_cb(SimpleNamespace(data=True), response)
            assert response.success
            self.supervisor.service_result(action, state['epoch'], response.success, self.now)
        self.node.tick()
        if self.backend == 'bench_neutral':
            assert list(self.node.preview_pub.publish.call_args.args[0].data) == [0, 0, 1500, 1500]
        return state

    def rearm(self):
        self.axes[1], self.axes[4] = 992, 191
        for _ in range(25):
            self.tick()
            assert not self.gate.enabled
        self.axes[4] = 1792
        for _ in range(45):
            state = self.tick()
            if state['phase'] == 'RUNNING':
                break
        assert state['phase'] == 'RUNNING' and self.gate.enabled
        assert self.bus.registers[pca.PRESCALE] == 0x8A
        for ch in range(16):
            assert self.bus.registers[pca.LED0 + 4 * ch] == (
                [0, 0, 51, 1] if ch < 2 else pca.FULL_OFF)


@pytest.mark.parametrize('fault,reason', [('sa', 'radio_kill'), ('rc', 'rc_timeout'),
                                         ('lq', 'rf_link_stale_or_lost'), ('zero_lq', 'rf_link_lost_or_invalid')])
def test_rc_trial(monkeypatch, fault, reason):
    trial = Trial(monkeypatch)
    trial.axes[4] = 1792
    for _ in range(30):
        trial.tick()
    assert ('oe', 0) not in trial.log
    trial.rearm()
    writes = list(trial.log)
    trial.axes[1] = 1400
    assert trial.tick()['v'] > 0
    assert trial.gate.enabled and trial.log == writes
    if fault == 'sa':
        trial.axes[4] = 191
    if fault == 'zero_lq':
        trial.lq = 0
    for _ in range(23):
        state = trial.tick(rc=fault != 'rc', lq=fault != 'lq')
    assert state['reason'] == reason and state['estop']
    assert not trial.gate.enabled and trial.log[-1] == ('oe', 1)
    trial.axes[1], trial.axes[4], trial.lq = 992, 1792, 100
    for _ in range(30):
        assert trial.tick()['phase'] == 'KILLED'
        assert not trial.gate.enabled
    trial.rearm()
    trial.node.stop()
    assert not trial.gate.enabled and trial.log[-1] == ('oe', 1)


def test_failed_readback_never_enables(monkeypatch):
    trial = Trial(monkeypatch)
    monkeypatch.setattr(trial.bus, 'read_i2c_block_data', lambda *_: [0, 0, 0, 0])
    with pytest.raises(RuntimeError, match='readback failed'):
        trial.rearm()
    assert ('oe', 0) not in trial.log
    assert not trial.node.controller.armed


def test_non_neutral_sa_edge_and_estop_gap(monkeypatch):
    trial = Trial(monkeypatch)
    for _ in range(25):
        trial.tick()
    trial.axes[1], trial.axes[4] = 1400, 1792
    assert trial.tick()['phase'] == 'KILLED'
    assert not trial.gate.enabled
    trial.rearm()
    trial.now += 0.6
    trial.node.estop_cb(Bool(data=False))
    assert not trial.gate.enabled and trial.node.controller.estop
    assert not trial.node.controller.armed


def test_deadline_during_readback(monkeypatch):
    trial = Trial(monkeypatch)
    prepare = pca.prepare

    def delayed_prepare(bus):
        result = prepare(bus)
        trial.now = trial.node.deadline
        return result

    monkeypatch.setattr(pca, 'prepare', delayed_prepare)
    with pytest.raises(RuntimeError, match='enable expired'):
        trial.rearm()
    assert ('oe', 0) not in trial.log


@pytest.mark.parametrize('fault', ['sa', 'rf', 'lq', 'cmd_timeout', 'estop_timeout', 'deadline',
                                   'partial_write', 'readback'])
def test_variable_cutoff(monkeypatch, fault):
    t = Trial(monkeypatch, 'bench_variable')
    assert not t.gate.enabled
    t.rearm()
    t.axes[1] = 1811
    for _ in range(40):
        t.tick()
        assert t.gate.read_counts == (307, 307)
        assert t.node.controller.target == (0., 0.)
    for _ in range(45):
        t.tick()
    assert t.gate.read_counts == (315, 315)
    if fault in ('partial_write', 'readback'):
        if fault == 'partial_write':
            write = t.bus.write_i2c_block_data
            def fail(address, reg, data):
                if reg == pca.LED0 + 4:
                    raise OSError('CH1 write')
                write(address, reg, data)
            monkeypatch.setattr(t.bus, 'write_i2c_block_data', fail)
        else:
            monkeypatch.setattr(t.bus, 'read_i2c_block_data', lambda *_: pca.FULL_OFF)
        with pytest.raises((OSError, RuntimeError)):
            t.gate.write(0., 0.)
        assert t.gate.failed
        with pytest.raises(RuntimeError):
            t.gate.enable_neutral(lambda: True)
    elif fault == 'deadline':
        t.now = t.node.deadline
        t.node.tick()
    elif fault == 'cmd_timeout':
        t.now += .51
        t.node.last_estop = t.now
        t.node.tick()
        assert t.node.controller.reason == 'command_timeout_rearm_required'
    elif fault == 'estop_timeout':
        t.now += .51
        t.node.estop_cb(Bool(data=False))
        assert t.node.controller.estop
    else:
        if fault == 'sa':
            t.axes[4] = 191
        for _ in range(25):
            state = t.tick(rc=fault != 'rf', lq=fault != 'lq')
        assert state['reason'] == {'sa':'radio_kill','rf':'rc_timeout','lq':'rf_link_stale_or_lost'}[fault]
        t.axes[1], t.axes[4] = 992, 1792
        for _ in range(25):
            assert t.tick()['phase'] == 'KILLED' and not t.gate.enabled
    assert not t.gate.enabled and ('oe', 1) == t.log[-1]


def test_variable_expiry_before_enable(monkeypatch):
    t = Trial(monkeypatch, 'bench_variable')
    prepare = pca.prepare
    def expire(bus, channels):
        result = prepare(bus, channels)
        t.now = t.node.deadline
        return result
    monkeypatch.setattr(pca, 'prepare', expire)
    with pytest.raises(RuntimeError, match='enable expired'):
        t.rearm()
    assert ('oe', 0) not in t.log and not t.node.controller.armed


@pytest.mark.parametrize('arguments', [
    ['--duration', '20'],
    ['--disconnected', '--duration', '21'],
    ['--disconnected', '--duration', '0'],
    ['--disconnected', '--duration', '10', '--profile', 'custom.json'],
    ['--disconnected', '--duration', '20', '--test-estop-gpio'],
])
def test_variable_cli_refuses_before_ros(monkeypatch, arguments):
    monkeypatch.setattr(sys, 'argv', ['boat_motor', '--backend', 'bench_variable', *arguments])
    monkeypatch.setattr(motor.rclpy, 'init', lambda **kw: pytest.fail('entered ROS/device path'))
    with pytest.raises(SystemExit) as error:
        motor.main()
    assert error.value.code == 2


def test_variable_auto_refused(monkeypatch):
    t = Trial(monkeypatch, 'bench_variable')
    t.axes[5] = 1792
    for _ in range(27):
        t.tick()
    t.axes[4] = 1792
    assert t.tick()['phase'] == 'KILLED'
    assert not t.gate.enabled and ('oe', 0) not in t.log

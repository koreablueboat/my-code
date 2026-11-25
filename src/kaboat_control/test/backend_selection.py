"""Installed controller, synthetic inputs and in-memory handshake replies only."""
import sys
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix

lib = Path(get_package_prefix('kaboat_control')) / 'lib/kaboat_control'
sys.path.insert(0, str(lib))
loader = SourceFileLoader('installed_control', str(lib / 'control'))
adapter = module_from_spec(spec_from_loader(loader.name, loader))
loader.exec_module(adapter)


@pytest.fixture
def controller(request):
    backend = request.param
    args = [] if backend is None else ['--ros-args', '-p', 'expected_backend:=' + backend]
    rclpy.init(args=args)
    try:
        node = adapter.Control()
        try:
            yield node.core  # No executor spins and Control.dispatch is never invoked.
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


class Inputs:
    def __init__(self, core, backend):
        self.core = core
        self.axes = [992] * 16
        self.axes[4:6] = [191, 997]
        self.status = dict(backend=backend, armed=False, estop_latched=True,
                           backend_ready=True, esc_mode='bidirectional')
        self.step = 0

    def tick(self, rc=True, lq=True):
        self.step += 1
        now = self.step / 10
        if lq:
            self.core.set_lq(98, now)
        self.core.set_motor(self.status, now)
        if rc:
            self.core.set_rc(self.axes, now)
        return self.core.tick(now)

    def locked(self, **inputs):
        state = self.tick(**inputs)
        assert state['phase'] == 'KILLED' and state['estop']
        assert (state['v'], state['w'], state['action']) == (0., 0., None)
        return state

    def arm_sequence(self):
        self.axes[4] = 191
        for _ in range(12):
            self.locked()
        assert self.core.ready_to_arm
        self.axes[4] = 1792
        assert self.tick()['phase'] == 'RELEASE'
        actions = []
        for _ in range(20):
            state = self.tick()
            if state['action']:
                kind = state['action']
                actions.append(kind)
                self.core.service_result(kind, state['epoch'], True, self.step / 10)
                if kind == 'reset':
                    self.status['estop_latched'] = False
                elif kind == 'arm':
                    self.status['armed'] = True
            if state['phase'] == 'RUNNING':
                break
        assert actions == ['reset', 'arm']
        assert state['phase'] == 'RUNNING'


@pytest.mark.parametrize('controller,reported', [(None, 'preview'), ('pca9685', 'pca9685')],
                         indirect=['controller'])
def test_matching_backend_and_sa_kill(controller, reported):
    inputs = Inputs(controller, reported)
    inputs.arm_sequence()
    inputs.axes[4] = 191
    assert inputs.locked()['reason'] == 'radio_kill'
    inputs.status.update(armed=False, estop_latched=True)
    inputs.axes[4] = 1792  # Too soon after KILL.
    for _ in range(15):
        inputs.locked()
    inputs.arm_sequence()


@pytest.mark.parametrize('controller,reported', [(None, 'pca9685'), ('pca9685', 'preview')],
                         indirect=['controller'])
def test_backend_mismatch(controller, reported):
    inputs = Inputs(controller, reported)
    for _ in range(12):
        assert inputs.locked()['reason'] == 'motor_backend_mismatch'
        assert not controller.ready_to_arm
    inputs.axes[4] = 1792
    assert inputs.locked()['reason'] == 'motor_backend_mismatch'
    inputs.status['backend'] = controller.backend
    inputs.locked()
    inputs.arm_sequence()
    inputs.status['backend'] = reported
    assert inputs.locked()['reason'] == 'motor_backend_mismatch'


@pytest.mark.parametrize('controller', ['pca9685'], indirect=True)
@pytest.mark.parametrize('missing,reason', [('rc', 'rc_timeout'), ('lq', 'rf_link_stale_or_lost')])
def test_link_recovery_requires_sa_sequence(controller, missing, reason):
    inputs = Inputs(controller, 'pca9685')
    inputs.arm_sequence()
    for _ in range(12):
        state = inputs.tick(**{missing: False})
    assert state['phase'] == 'KILLED' and state['reason'] == reason
    inputs.status.update(armed=False, estop_latched=True)
    for _ in range(15):  # Healthy link with SA still HIGH.
        inputs.locked()
    inputs.arm_sequence()

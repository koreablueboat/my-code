"""Run against the installed ROS executables, with synthetic bytes instead of UART."""
import json
import os
import sys
import time
import unittest
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from ament_index_python.packages import get_package_prefix
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray, String
from std_srvs.srv import Trigger


def installed(package, executable):
    path = Path(get_package_prefix(package)) / 'lib' / package
    sys.path.insert(0, str(path))
    loader = SourceFileLoader(executable, str(path / executable))
    module = module_from_spec(spec_from_loader(executable, loader))
    loader.exec_module(module)
    return module


receiver = installed('boat_rc', 'drive')
control = installed('kaboat_control', 'control')
motor = installed('kaboat_motor', 'boat_motor')


class PreviewTest(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '117')
        rclpy.init(args=[])
        self.addCleanup(rclpy.try_shutdown)
        self.axes = [992] * 16
        self.axes[4:6] = [1792, 997]
        self.frames = True
        self.bad_crc = False
        self.lq = 98
        uart = SimpleNamespace(read=self.read, reset_input_buffer=lambda: None)
        with patch.object(receiver.serial, 'Serial', return_value=uart):
            self.rx = receiver.Drive()
        profile = Path(motor.__file__).with_name('preview_bidirectional.json')
        self.motor = motor.BoatMotorNode(SimpleNamespace(backend='preview', duration=0,
                                           profile=profile, test_estop_gpio=False))
        self.control = control.Control()
        self.probe = rclpy.create_node('preview_probe')
        self.executor = SingleThreadedExecutor()
        for node in (self.rx, self.motor, self.control, self.probe):
            self.executor.add_node(node)
            self.addCleanup(node.destroy_node)
        self.addCleanup(self.executor.shutdown)
        self.assertEqual(self.probe.count_publishers('/boat/cmd_vel'), 1)
        self.auto = self.probe.create_publisher(Twist, '/boat/auto/cmd_vel', 1)
        self.auto_msg = None
        self.probe.create_timer(.04, self.send_auto)
        self.rearm = self.probe.create_client(Trigger, '/boat/control/rearm')
        self.preview = self.cmd = self.status = self.estop = None
        self.frame_count = 0
        self.probe.create_subscription(Float32MultiArray, '/boat/motor_preview',
                                       lambda m: setattr(self, 'preview', list(m.data)), 1)
        self.probe.create_subscription(Twist, '/boat/cmd_vel', lambda m: setattr(self, 'cmd', m), 1)
        self.probe.create_subscription(String, '/boat/motor_status',
                                       lambda m: setattr(self, 'status', json.loads(m.data)), 1)
        self.probe.create_subscription(Bool, '/boat/estop', lambda m: setattr(self, 'estop', m.data), 1)
        self.probe.create_subscription(Int32MultiArray, '/rc_channels', self.observe_rc, 1)
        self.pump(.8)

    def observe_rc(self, msg):
        self.assertEqual(len(msg.data), 16)
        self.frame_count += 1

    def read(self, size):
        if not self.frames:
            return b''
        packed = sum(v << (11 * i) for i, v in enumerate(self.axes))
        data = b'\x16' + packed.to_bytes(22, 'little')
        rc_frame = bytes([200, 24]) + data + bytes([receiver.crc8(data) ^ int(self.bad_crc)])
        if self.lq is None:
            return rc_frame
        stats = bytes([0x14, 60, 60, self.lq, 10, 0, 2, 1, 60, 100, 10])
        return rc_frame + bytes([200, 12]) + stats + bytes([receiver.crc8(stats)])

    def send_auto(self):
        if self.auto_msg is not None:
            self.auto.publish(self.auto_msg)

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=.01)

    def request(self):
        future = self.rearm.call_async(Trigger.Request())
        self.executor.spin_until_future_complete(future, timeout_sec=2)
        return future.result().success

    def arm(self):
        self.axes[4] = 191
        self.pump(1.3)
        self.assertTrue(self.control.core.ready_to_arm, self.control.core.reason)
        self.assertFalse(self.request())  # service cannot replace the deliberate edge
        self.axes[4] = 1792
        end = time.monotonic() + 5
        while self.control.core.phase != 'RUNNING' and time.monotonic() < end:
            self.pump(.02)
        self.assertEqual(self.control.core.phase, 'RUNNING', self.control.core.reason)
        self.assertTrue(self.motor.controller.armed)

    def stopped(self):
        self.pump(.65)
        self.assertTrue(self.estop)
        self.assertTrue(self.status['estop_latched'])
        self.assertFalse(self.status['armed'])
        self.assertEqual(self.preview[:2], [0., 0.])
        self.assertEqual((self.cmd.linear.x, self.cmd.angular.z), (0., 0.))

    def test_switches_and_rearm(self):
        self.stopped()  # started with SA already high
        self.assertFalse(self.request())
        self.axes[4] = 191
        self.pump(.35)
        self.axes[4] = 1792
        self.stopped()
        self.axes[5] = 191
        self.axes[4] = 191
        self.pump(1.2)
        self.assertEqual(self.control.core.mode, 'STANDBY')
        self.assertFalse(self.control.core.ready_to_arm)
        self.axes[5] = 997
        self.arm()
        self.axes[4] = 191
        self.stopped()
        self.pump(.7)
        self.axes[4] = 1792
        self.pump(.12)
        self.axes[1] = 1811
        self.stopped()  # nonzero during RELEASE
        self.axes[1] = 992
        self.stopped()
        self.arm()
        self.axes[5] = 1350
        self.stopped()
        self.assertEqual(self.control.core.mode, 'INVALID')
        self.axes[5] = 997
        self.arm()
        self.axes[4] = 992
        self.stopped()
        self.assertIsNone(self.control.core.permit)

    def test_interrupted_handshake(self):
        for fault in ('mode', 'link'):
            with self.subTest(fault=fault):
                self.axes[4:6] = [191, 997]
                self.lq = 98
                self.pump(1.35)
                self.assertTrue(self.control.core.ready_to_arm)
                self.axes[4] = 1792
                self.pump(.12)
                self.assertEqual(self.control.core.phase, 'RELEASE')
                if fault == 'mode':
                    self.axes[5] = 191
                else:
                    self.lq = 0
                self.stopped()
                self.lq = 98
                self.axes[5] = 997
                self.stopped()
                self.assertFalse(self.request())

    def test_manual_and_motor_reversal(self):
        self.arm()
        self.axes[1] = 1811
        self.pump(.4)
        self.assertAlmostEqual(self.cmd.linear.x, .03)
        self.assertGreater(min(self.preview[:2]), 0.)
        self.axes[1] = 174
        self.pump(.20)
        self.assertAlmostEqual(self.cmd.linear.x, -.009)
        self.assertEqual(self.preview[:2], [0., 0.])
        self.pump(.55)
        self.assertLess(max(self.preview[:2]), 0.)
        self.assertGreaterEqual(min(self.preview[:2]), -.040001)
        for steer, yaw in [(174, .06), (1811, -.06)]:
            self.axes[:2] = [steer, 992]
            self.pump(.8)
            self.assertAlmostEqual(self.cmd.angular.z, yaw)
            left, right = self.preview[:2]
            self.assertLess(left * yaw, 0)
            self.assertGreater(right * yaw, 0)
            self.assertLessEqual(max(abs(left), abs(right)), .040001)
        self.axes[4] = 191
        self.stopped()

    def test_fault_recovery(self):
        self.arm()
        self.frames = False
        self.stopped()
        count = self.frame_count
        self.frames = self.bad_crc = True
        self.pump(.25)
        self.assertEqual(self.frame_count, count)
        self.bad_crc = False
        self.stopped()
        self.arm()
        self.lq = None
        self.pump(1.2)
        self.stopped()
        self.lq = 98
        self.stopped()
        self.arm()
        other = self.probe.create_publisher(Twist, '/boat/cmd_vel', 1)
        self.stopped()
        self.probe.destroy_publisher(other)
        self.stopped()

    def test_auto_transition_timeout(self):
        self.arm()
        self.auto_msg = Twist()
        self.axes[5] = 1792
        self.stopped()
        self.auto_msg.linear.x = .03
        self.axes[4] = 191
        self.pump(1.25)
        self.assertFalse(self.control.core.ready_to_arm)
        self.auto_msg = Twist()
        self.arm()
        self.auto_msg.linear.x = .03
        self.pump(.4)
        self.assertGreater(min(self.preview[:2]), 0.)
        self.auto_msg = None
        self.stopped()
        self.auto_msg = Twist()
        self.stopped()
        self.arm()
        self.frames = False
        self.stopped()  # even with live AUTO
        self.frames = True
        self.stopped()
        self.axes[5] = 997
        self.stopped()
        self.arm()


class ConfirmationTest(unittest.TestCase):
    def test_services_require_new_status_and_neutral_hold(self):
        config = json.loads(Path(control.__file__).with_name('control_config.json').read_text())
        c = control.Supervisor(config)
        axes = [992] * 16
        axes[4:6] = [191, 997]
        state = dict(armed=False, estop_latched=True, backend_ready=True,
                     esc_mode='bidirectional', backend='preview')

        def tick(t, report=False):
            c.set_lq(98, t)
            if report:
                c.set_motor(state, t)
            c.set_rc(axes, t)
            return c.tick(t)

        for i in range(12):
            tick(i / 10, report=True)
        axes[4] = 1792
        self.assertEqual(tick(1.2)['phase'], 'RELEASE')
        self.assertEqual(tick(1.46)['action'], 'reset')
        state['estop_latched'] = False
        tick(1.47, report=True)
        c.service_result('reset', c.epoch, True, 1.48)
        self.assertEqual(tick(1.49)['phase'], 'WAIT_RESET')
        self.assertEqual(tick(1.50, report=True)['phase'], 'ZERO')
        for t in (1.7, 1.9, 2.099):
            self.assertEqual(tick(t)['phase'], 'ZERO')
        self.assertEqual(tick(2.101)['action'], 'arm')
        state['armed'] = True
        tick(2.11, report=True)
        c.service_result('arm', c.epoch, True, 2.12)
        self.assertEqual(tick(2.13)['phase'], 'WAIT_ARM')
        self.assertEqual(tick(2.14, report=True)['phase'], 'RUNNING')


if __name__ == '__main__':
    unittest.main(verbosity=2)

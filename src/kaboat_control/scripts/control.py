#!/usr/bin/env python3
"""ROS adapter for the field v3.4 Supervisor and a bidirectional motor."""
import json
import signal
import time
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String, UInt8
from std_srvs.srv import SetBool, Trigger

from supervisor import ACTIVE_PHASES, AUTO_TIMEOUT, LQ_TIMEOUT, MOTOR_TIMEOUT, RC_TIMEOUT, Supervisor


class Control(Node):
    def __init__(self):
        super().__init__('kaboat_control')
        config = json.loads(Path(__file__).with_name('control_config.json').read_text())
        backend = self.declare_parameter(
            'expected_backend', 'preview',
            ParameterDescriptor(read_only=True, description='Expected motor_status backend: preview, pca9685, bench_neutral, bench_variable left_t200_water or manual_dual'),
        ).value
        self.core = Supervisor(config, expected_backend=backend)
        self.cmd = self.create_publisher(Twist, '/boat/cmd_vel', 1)
        self.stop = self.create_publisher(Bool, '/boat/estop', 1)
        self.reset = self.create_client(Trigger, '/boat/reset_estop')
        self.arm = self.create_client(SetBool, '/boat/arm')
        self.create_subscription(Int32MultiArray, '/rc_channels', self.rc, 1)
        self.create_subscription(UInt8, '/rc_uplink_lq', self.lq, 1)
        self.create_subscription(Twist, '/boat/auto/cmd_vel', self.auto, 1)
        self.create_subscription(String, '/boat/motor_status', self.motor, 1)
        self.create_service(Trigger, '/boat/control/rearm', self.rearm)
        self.last_tick = time.monotonic()
        self.last_graph = 0.
        self.last_transition = None
        self.last_auto_rx = None
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish(self, state):
        transition = (state['phase'], state['reason'])
        if transition != self.last_transition:
            self.get_logger().info(json.dumps({
                'phase': state['phase'], 'previous_phase': self.last_transition[0] if self.last_transition else None,
                'reason': state['reason'], 'monotonic_s': time.monotonic(),
                'last_auto_rx_monotonic_s': self.last_auto_rx,
            }))
            self.last_transition = transition
        msg = Twist()
        msg.linear.x, msg.angular.z = state['v'], state['w']
        self.stop.publish(Bool(data=state['estop']))
        self.cmd.publish(msg)

    def check_gap(self, last, limit, now, reason):
        # A returning packet must not hide a timeout between timer callbacks.
        if last is not None and now - last > limit:
            self.core.stop(reason)

    def check_input(self, was_active):
        c = self.core
        if c.phase in ACTIVE_PHASES and c.phase != 'RUNNING':
            if not c.neutral() or (c.mode == 'AUTO' and c.auto != (0., 0.)):
                c.stop('nonzero_during_arming')
        if was_active and c.phase == 'KILLED':
            self.publish(c.result())

    def rc(self, msg):
        c = self.core
        now = time.monotonic()
        was_active = c.phase in ACTIVE_PHASES
        self.check_gap(c.rc_at, RC_TIMEOUT, now, 'rc_timeout')
        c.set_rc(list(msg.data), now)
        self.check_input(was_active)

    def lq(self, msg):
        c = self.core
        now = time.monotonic()
        was_active = c.phase in ACTIVE_PHASES
        self.check_gap(c.lq_at, LQ_TIMEOUT, now, 'rf_link_stale_or_lost')
        c.set_lq(msg.data, now)
        self.check_input(was_active)

    def auto(self, msg):
        c = self.core
        if c.mode != 'AUTO':
            return
        was_active = c.phase in ACTIVE_PHASES
        now = time.monotonic()
        if c.auto_at is not None and now - c.auto_at > AUTO_TIMEOUT:
            self.get_logger().warning(f'AUTO reception gap {now - c.auto_at:.3f}s, previous valid command at {c.auto_at:.6f}')
        self.last_auto_rx = now
        self.check_gap(c.auto_at, AUTO_TIMEOUT, now, 'auto_command_timeout')
        if any(v != 0 for v in (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)):
            c.auto_at = None
            c.stop('nonplanar_auto_command')
        else:
            c.set_auto(msg.linear.x, msg.angular.z, now)
            if c.permit is False and c.auto != (0., 0.):
                c.ready_to_arm = False
                c.safe_since = None
        self.check_input(was_active)

    def motor(self, msg):
        c = self.core
        now = time.monotonic()
        was_active = c.phase in ACTIVE_PHASES
        self.check_gap(c.motor_at, MOTOR_TIMEOUT, now, 'motor_status_timeout')
        try:
            state = json.loads(msg.data)
        except ValueError:
            state = None
        c.set_motor(state, now)
        if c.phase in ACTIVE_PHASES and c.health(now):
            c.stop(c.health(now))
        self.check_input(was_active)

    def rearm(self, req, res):
        # Keep the ROS entry point but only the physical SA edge requests ARM.
        res.success = self.core.phase == 'RUNNING'
        res.message = (f'{self.core.phase}: {self.core.reason}; '
                       f'ready_to_arm={self.core.ready_to_arm}; use neutral SA LOW then HIGH')
        return res

    def dispatch(self, action, epoch):
        if action == 'reset':
            client, request = self.reset, Trigger.Request()
        else:
            client, request = self.arm, SetBool.Request(data=True)
        if not client.service_is_ready():
            self.core.service_result(action, epoch, False, time.monotonic())
            self.publish(self.core.result())
            return

        def done(future):
            success = not future.cancelled() and not future.exception() and future.result().success
            self.core.service_result(action, epoch, success, time.monotonic())

        client.call_async(request).add_done_callback(done)

    def tick(self):
        now = time.monotonic()
        if now - self.last_tick > .25:
            self.core.stop('control_loop_stall')
        self.last_tick = now
        if now - self.last_graph >= .20:
            topics = ('/boat/cmd_vel', '/boat/estop', '/boat/motor_status')
            self.core.set_graph(all(self.count_publishers(t) == 1 for t in topics),
                                self.count_publishers('/boat/auto/cmd_vel') == 1)
            self.last_graph = now
        state = self.core.tick(now)
        self.publish(state)
        if state['action']:
            self.dispatch(state['action'], state['epoch'])


def main():
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    # Finish the in-flight message conversion before tearing down the executor.
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Control()
    executor = rclpy.get_global_executor()
    executor.add_node(node)
    try:
        while not stopping and rclpy.ok():
            executor.spin_once(timeout_sec=.05)
    finally:
        executor.remove_node(node)
        if rclpy.ok():
            node.core.stop('shutdown')
            node.publish(node.core.result())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Pi -> I2C PCA9685 -> two ESCs. Preview by default; verified profile required."""
from contextlib import ExitStack
import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Float32MultiArray
from std_srvs.srv import SetBool, Trigger
from esc_config import controller_for_profile, validate, pulse_us
from reverse_motor_core import TIMEOUT
from pca_backend import PCAOutput
from manual_dual import ManualDualOutput, validate_dual
from bench_pca import (BENCH_PRESCALE, NEUTRAL_COUNTS, FixedNeutralGate, VariableBenchOutput, LeftWaterOutput, validate_left_water, request_oe, shutdown)


class BoatMotorNode(Node):
    def __init__(self,args,bench_gate=None):
        super().__init__('boat_motor_node')
        self.args=args
        self.profile=validate(json.loads(Path(args.profile).read_text()),hardware=args.backend in ('pca9685','left_t200_water','manual_dual'))
        self.controller = controller_for_profile(self.profile)
        self.bridge=None
        if args.backend in ("bench_variable", "left_t200_water", "manual_dual"):
            oe, bus = bench_gate
            adapter = {"bench_variable": VariableBenchOutput, "left_t200_water": LeftWaterOutput,
                       "manual_dual": ManualDualOutput}[args.backend]
            self.bridge = adapter(oe, bus, self.profile)
            bench_gate = None
        self.bench_gate = bench_gate
        self.bridge_ready = args.backend == 'preview' or bench_gate is not None or self.bridge is not None
        self.started=self.previous=time.monotonic()
        self.deadline=self.started+args.duration if args.duration>0 else float('inf')
        self.stop_requested=False
        self.last_status=0.
        self.last_reason=None
        self.ros_estop = bench_gate is not None or self.bridge is not None
        self.last_estop = None
        self.estop_pin = None
        if args.backend == 'pca9685' and self.profile['oe_gpio_bcm'] != 17:
            raise ValueError('OE wiring requires BCM17')
        if args.backend == 'pca9685' or args.test_estop_gpio:
            from gpiozero import DigitalInputDevice
            self.estop_pin = DigitalInputDevice(27, pull_up=True)
        try:
            if args.backend == 'pca9685':
                self.bridge = PCAOutput(self.profile)
                self.bridge_ready = True
            self.read_estop()
        except Exception:
            if self.bridge:
                self.bridge.close()
            if self.estop_pin:
                self.estop_pin.close()
            raise
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Twist,'/boat/cmd_vel',self.command,qos)
        self.create_subscription(Bool,'/boat/estop',self.estop_cb,qos)
        self.create_service(SetBool,'/boat/arm',self.arm_cb)
        self.create_service(Trigger,'/boat/reset_estop',self.reset_cb)
        # Diagnostic only: not a direct hardware interface.
        self.preview_pub=self.create_publisher(Float32MultiArray,'/boat/motor_preview',1)
        self.status_pub=self.create_publisher(String,'/boat/motor_status',1)
        self.create_timer(.05,self.tick)
        self.get_logger().info(f'backend={args.backend}; DISARMED; input=/boat/cmd_vel')

    def command(self,msg):
        values=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        if self.args.backend == 'left_t200_water' and (not 0 <= msg.linear.x <= .03 or msg.angular.z != 0):
            self.controller.disarm('left_water_forward_only')
            self.bridge.disable()
            return
        if not all(math.isfinite(v) for v in values) or any(abs(v)>1e-9 for v in (values[1],values[2],values[3],values[4])):
            self.controller.disarm('invalid_or_nonplanar_Twist')
            if self.bench_gate:
                self.bench_gate.disable()
            if self.args.backend in ('bench_variable', 'left_t200_water', 'manual_dual'):
                self.bridge.disable()
            return
        now = time.monotonic()
        if (self.args.backend in ('left_t200_water','manual_dual') and self.controller.armed and
                (msg.linear.x != 0 or msg.angular.z != 0) and not self.bridge.poll(now)):
            self.controller.disarm('nonzero_during_water_initialization')
            self.bridge.disable()
            return
        self.controller.command(msg.linear.x, msg.angular.z, now)
        if self.bridge and self.controller.armed and not self.bridge.poll(now):
            # Validate the command and feed the watchdog, but discard its target.
            # arm() reset output and reversal history; step() sees only neutral.
            self.controller.target = (0., 0.)
        if self.args.backend in ('bench_variable', 'left_t200_water', 'manual_dual') and self.bridge.enabled and not self.controller.armed:
            self.bridge.disable()
        if self.bench_gate and self.bench_gate.enabled and not self.controller.armed:
            self.bench_gate.disable()

    def read_estop(self):
        # With pull-up, gpiozero's logical "active" means electrical LOW.
        physical_stop = self.estop_pin is not None and not self.estop_pin.is_active
        stop = physical_stop or self.ros_estop
        if self.bench_gate or self.bridge:
            stop = stop or self.last_estop is None or time.monotonic() - self.last_estop > TIMEOUT
            if stop and self.bench_gate and self.bench_gate.enabled:
                self.bench_gate.disable()
        if stop and self.bridge:
            self.bridge.disable()
        self.controller.emergency(stop)

    def estop_cb(self,msg):
        if self.bench_gate or self.bridge:
            self.read_estop()  # Latch a reception gap before accepting a returning heartbeat.
        self.last_estop = time.monotonic()
        self.ros_estop = msg.data
        self.read_estop()
        if self.controller.estop_input:
            self.emit(0., 0.)

    def enable_permitted(self):
        now = time.monotonic()
        return (not self.stop_requested and now < self.deadline and self.controller.armed
                and (self.estop_pin is None or self.estop_pin.is_active)
                and not self.ros_estop and self.last_estop is not None and now - self.last_estop <= TIMEOUT
                and self.controller.command_at is not None and now - self.controller.command_at <= TIMEOUT)

    def arm_cb(self,req,res):
        self.read_estop()
        if not req.data:
            self.controller.disarm()
            self.emit(0.,0.)
            res.success,res.message=True,'disarmed'
        elif self.stop_requested or time.monotonic()>=self.deadline:
            res.success,res.message=False,'trial finished'
        elif not self.bridge_ready:
            res.success,res.message=False,'PCA neutral initialization not finished'
        elif self.count_publishers('/boat/cmd_vel') != 1:
            res.success,res.message=False,'exactly one command publisher required'
        elif self.controller.armed:
            res.success, res.message = True, 'already armed'
        else:
            res.success, res.message = self.controller.arm(time.monotonic())
            if res.success and (self.bench_gate or self.bridge):
                try:
                    if self.bench_gate:
                        self.bench_gate.enable_neutral(self.enable_permitted)
                    else:
                        self.bridge.enable_neutral(self.enable_permitted)
                except Exception:
                    self.controller.disarm('pca_enable_failed')
                    (self.bench_gate or self.bridge).disable()
                    raise
        return res

    def reset_cb(self,req,res):
        self.read_estop()
        res.success=self.controller.reset()
        res.message='reset; still disarmed' if res.success else 'release estop input first'
        return res

    def emit(self,left,right):
        if self.stop_requested or time.monotonic()>=self.deadline:
            left=right=0.
        if self.bench_gate:
            left = right = 0.
            if self.bench_gate.enabled and (not self.controller.armed or self.stop_requested or time.monotonic() >= self.deadline):
                self.bench_gate.disable()
        if self.args.backend == 'left_t200_water':
            right = 0.
        if self.bridge:
            if self.stop_requested or not self.controller.armed or time.monotonic() >= self.deadline:
                self.bridge.disable()
            else:
                if not self.bridge.poll(time.monotonic()):
                    left = right = 0.
                self.bridge.write(left, right)
        # Fields: left/right normalized, left/right reference microseconds.
        if self.args.backend == 'manual_dual':
            counts = self.bridge.read_counts if self.bridge.enabled else (0, 0)
            widths = [count * 1e6 / (4096 * self.bridge.frequency) for count in counts]
        else:
            widths = [pulse_us(left,self.profile,'left'), pulse_us(right,self.profile,'right')]
        self.preview_pub.publish(Float32MultiArray(data=[float(left),float(right),*widths]))
        return left, right

    def tick(self):
        now=time.monotonic()
        self.read_estop()
        if self.bridge:
            self.bridge_ready = not self.bridge.failed
        if self.count_publishers('/boat/cmd_vel')>1:
            self.controller.disarm('multiple_command_publishers')
        if self.stop_requested or now>=self.deadline:
            self.controller.disarm('finished')
        left,right=self.controller.step(now,now-self.previous)
        self.previous=now
        left,right=self.emit(left,right)
        c=self.controller
        if now-self.last_status>=.5 or c.reason!=self.last_reason:
            state={'backend':self.args.backend,'armed':c.armed,'estop_latched':c.estop,
                   'reason':c.reason,
                   'estop_input':c.estop_input,
                   'logical_left':left,'logical_right':right,
                   'backend_ready':self.bridge_ready,'esc_mode':self.profile['mode'],'physical_speed_measured':False}
            if self.bench_gate:
                state.update(fixed_neutral_bench=True, oe_low_requested=self.bench_gate.enabled)
            if self.args.backend in ('bench_variable', 'left_t200_water', 'manual_dual'):
                state.update(oe_low_requested=self.bridge.enabled, oe_request_ns=self.bridge.oe_ns,
                             target_counts=self.bridge.target_counts, read_counts=self.bridge.read_counts,
                             readback_ns=self.bridge.read_ns, deadline_mono=self.deadline)
            if self.args.backend in ('left_t200_water','manual_dual'):
                state.update(neutral_init_complete=self.bridge.poll(now),
                             remaining_s=round(max(0., self.deadline - now), 1))
            self.status_pub.publish(String(data=json.dumps(state)))
            self.get_logger().info(json.dumps(state))
            self.last_status,self.last_reason=now,c.reason

    def stop(self):
        self.stop_requested=True
        self.controller.disarm('shutdown')
        try:
            self.emit(0.,0.)
        finally:
            try:
                if self.bridge:
                    self.bridge.close()
            finally:
                if self.estop_pin:
                    self.estop_pin.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',choices=['preview','pca9685','bench_neutral','bench_variable','left_t200_water','manual_dual'],default='preview')
    p.add_argument('--profile',default=str(Path(__file__).with_name('preview_bidirectional.json')))
    p.add_argument('--duration',type=float,default=0.,help='seconds: preview 0 for unlimited, bench_neutral >0, pca9685 (0,20]')
    p.add_argument('--test-estop-gpio', action='store_true',
                   help='read physical E-stop GPIO in preview mode')
    p.add_argument('--dry-run', action='store_true', help='describe the fixed-neutral bench without opening devices or ROS')
    p.add_argument('--disconnected', action='store_true',
                   help='confirm propulsion battery and both ESC signal leads are disconnected')
    p.add_argument('--check-profile', action='store_true', help='left water approval check without ROS or devices')
    args=p.parse_args()
    dual = args.backend == 'manual_dual'
    water = args.backend == 'left_t200_water' or dual
    if args.check_profile and not water:
        p.error('--check-profile requires left_t200_water')
    if water:
        if args.test_estop_gpio or args.dry_run or args.disconnected:
            p.error('invalid left water trial options')
        try:
            profile = json.loads(Path(args.profile).read_text())
            if dual:
                validate_dual(profile, hardware=True)
            else:
                validate_left_water(profile)
        except (ValueError, OSError) as exc:
            p.error(str(exc))
    variable = args.backend == 'bench_variable'
    if variable != args.disconnected:
        p.error('bench_variable requires explicit --disconnected confirmation')
    if variable:
        if args.profile != p.get_default('profile') or args.test_estop_gpio:
            p.error('bench_variable uses the installed replacement board candidate only')
        args.profile = str(Path(__file__).with_name('replacement_board_candidate.json'))
    bench = args.backend == 'bench_neutral'
    if bench and (args.test_estop_gpio or args.profile != p.get_default('profile')):
        p.error('bench_neutral uses fixed settings and has no BCM27 input')
    if args.dry_run and not bench:
        p.error('--dry-run is for bench_neutral')
    if not math.isfinite(args.duration) or args.duration<0:
        p.error('invalid duration')
    if (water or variable or args.backend=='pca9685') and not 0<args.duration<=20:
        p.error('PCA trial requires --duration in (0,20]')
    if args.check_profile:
        print(f'{args.backend} profile accepted. Physical cutoff and setup require on-site confirmation.')
        return
    if water and not dual:
        if not sys.stdin.isatty():
            p.error('left water trial requires an attended terminal')
        print('왼쪽 T200 수중 고정, 오른쪽 ESC 전원·신호 분리, 독립 차단 담당자 대기. '
              'SB MANUAL, SA KILL, 스틱 중앙. 추진 전원 차단 준비 확인.')
        if input('시험을 시작하려면 LEFT WATER 입력: ').strip() != 'LEFT WATER':
            p.error('left water trial cancelled')
    if args.dry_run:
        print(f'bench_neutral DRY-RUN: OE BCM17 initially HIGH; PRESCALE=0x{BENCH_PRESCALE:02X}; '
              f'CH0/1={NEUTRAL_COUNTS} counts; CH2..15 FULL OFF; BCM27 unused')
        return
    if bench and args.duration <= 0:
        p.error('bench_neutral requires a positive finite --duration in seconds')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=None
    stop=[]
    def interrupt(sig,frame):
        stop.append(sig)
        if node:node.stop_requested=True
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupt)
    try:
        with ExitStack() as stack:
            gate = None
            if bench or variable or water:
                import gpiod
                import smbus
                owner = 'dual-manual' if dual else 'rc-variable' if variable else 'left-water' if water else 'rc-fixed-neutral'
                _, oe = request_oe(stack, gpiod, owner)
                bus = smbus.SMBus(1)
                stack.callback(bus.close)
                stack.callback(shutdown, oe, bus)
                gate = (oe, bus) if variable or water else FixedNeutralGate(oe, bus)
            node = BoatMotorNode(args, gate)
            try:
                while rclpy.ok() and not stop and time.monotonic() < node.deadline:
                    rclpy.spin_once(node, timeout_sec=.05)
                if dual:
                    why = f'signal {stop[0]}' if stop else ('duration' if time.monotonic() >= node.deadline else 'context')
            finally:
                node.stop()
            if dual:
                node.get_logger().info(f'manual_dual stopped: {why}; OE HIGH requested')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':main()

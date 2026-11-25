#!/usr/bin/env python3
"""Pi -> I2C PCA9685 -> two ESCs. Preview by default; verified profile required."""
import argparse
import json
import math
import signal
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
from pca_backend import PCAOutput


class BoatMotorNode(Node):
    def __init__(self,args):
        super().__init__('boat_motor_node')
        self.args=args
        self.profile=validate(json.loads(Path(args.profile).read_text()),hardware=args.backend=='pca9685')
        self.controller = controller_for_profile(self.profile)
        self.bridge=None
        self.bridge_ready=args.backend=='preview'
        self.started=self.previous=time.monotonic()
        self.deadline=self.started+args.duration if args.duration>0 else float('inf')
        self.stop_requested=False
        self.last_status=0.
        self.last_reason=None
        self.ros_estop = False
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
        self.get_logger().info(f'backend={args.backend}; DISARMED; input=/boat/cmd_vel; max command=15%')

    def command(self,msg):
        values=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        if not all(math.isfinite(v) for v in values) or any(abs(v)>1e-9 for v in (values[1],values[2],values[3],values[4])):
            self.controller.disarm('invalid_or_nonplanar_Twist')
            return
        self.controller.command(msg.linear.x,msg.angular.z,time.monotonic())

    def read_estop(self):
        # With pull-up, gpiozero's logical "active" means electrical LOW.
        physical_stop = self.estop_pin is not None and not self.estop_pin.is_active
        stop = physical_stop or self.ros_estop
        if stop and self.bridge:
            self.bridge.disable()
        self.controller.emergency(stop)

    def estop_cb(self,msg):
        self.ros_estop = msg.data
        self.read_estop()
        if self.controller.estop_input:
            self.emit(0., 0.)

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
            if res.success and self.bridge:
                try:
                    self.bridge.enable_neutral()
                except Exception:
                    self.controller.disarm('pca_enable_failed')
                    self.bridge.disable()
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
        if self.bridge:
            if self.stop_requested or not self.controller.armed:
                self.bridge.disable()
            else:
                if not self.bridge.poll(time.monotonic()):
                    left = right = 0.
                self.bridge.write(left, right)
        # Fields: left/right normalized, left/right reference microseconds.
        self.preview_pub.publish(Float32MultiArray(data=[float(left),float(right),float(pulse_us(left,self.profile,'left')),float(pulse_us(right,self.profile,'right'))]))

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
        self.emit(left,right)
        c=self.controller
        if now-self.last_status>=.5 or c.reason!=self.last_reason:
            state={'backend':self.args.backend,'armed':c.armed,'estop_latched':c.estop,
                   'reason':c.reason,
                   'estop_input':c.estop_input,
                   'logical_left':left,'logical_right':right,
                   'backend_ready':self.bridge_ready,'esc_mode':self.profile['mode'],'physical_speed_measured':False}
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
    p.add_argument('--backend',choices=['preview','pca9685'],default='preview')
    p.add_argument('--profile',default=str(Path(__file__).with_name('preview_bidirectional.json')))
    p.add_argument('--duration',type=float,default=0.,help='0 unlimited preview; hardware must be 1..20 s')
    p.add_argument('--test-estop-gpio', action='store_true',
                   help='read physical E-stop GPIO in preview mode')
    args=p.parse_args()
    if not math.isfinite(args.duration) or args.duration<0:
        p.error('invalid duration')
    if args.backend=='pca9685' and not 0<args.duration<=20:
        p.error('PCA trial requires --duration in (0,20]')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=None
    stop=[]
    def interrupt(sig,frame):
        stop.append(sig)
        if node:node.stop_requested=True
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupt)
    try:
        node=BoatMotorNode(args)
        while rclpy.ok() and not stop and time.monotonic()<node.deadline:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        try:
            if node:node.stop()
        finally:
            if node:node.destroy_node()
            if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()

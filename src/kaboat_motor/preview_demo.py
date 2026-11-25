#!/usr/bin/env python3
"""One publisher for a preview-only demonstration; refuses hardware-backend status."""
import json
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool


def main():
    rclpy.init()
    node=Node('boat_motor_preview_demo')
    pub=node.create_publisher(Twist,'/boat/cmd_vel',1)
    arm=node.create_client(SetBool,'/boat/arm')
    status=[]
    node.create_subscription(String,'/boat/motor_status',lambda m:status.append((time.monotonic(),json.loads(m.data))),1)
    def send(v=0.,w=0.):
        msg=Twist();msg.linear.x=v;msg.angular.z=w;pub.publish(msg)
    def check_preview():
        if not status or time.monotonic()-status[-1][0]>1. or status[-1][1].get('backend')!='preview':
            raise RuntimeError('preview backend not confirmed; demo refused')
    def pump(seconds,v=0.,w=0.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            check_preview();send(v,w);rclpy.spin_once(node,timeout_sec=.05);time.sleep(.025)
    try:
        end=time.monotonic()+3
        while not status and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        check_preview()
        if not arm.wait_for_service(timeout_sec=2):raise RuntimeError('arm service unavailable')
        pump(1.)
        req=SetBool.Request();req.data=True;future=arm.call_async(req)
        end=time.monotonic()+2
        while not future.done() and time.monotonic()<end:pump(.05)
        if not future.done() or not future.result().success:raise RuntimeError('arm refused')
        for label,v,w in [('forward',.03,0.),('left',0.,.06),('right',0.,-.06)]:
            print(label,flush=True);pump(3.,v,w)
        pump(1.)
        print('preview finished; no hardware backend used',flush=True)
    finally:
        if rclpy.ok():
            send()
            req=SetBool.Request();req.data=False
            if arm.service_is_ready():
                future=arm.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=.3)
            node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()

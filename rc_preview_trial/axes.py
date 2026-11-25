#!/usr/bin/python3
"""Live, read-only deadband progress from ROS subscriptions."""
import json
import signal
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Bool, String, UInt8
from deadband import TARGET, HOLD, DeadbandHold


def main():
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    # Finish the in-flight message conversion before tearing down the executor.
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("preview_trial_axes")
    hold = DeadbandHold()
    inputs = [
        ("/rc_channels", Int32MultiArray, lambda m:list(m.data)),
        ("/rc_uplink_lq", UInt8, lambda m:m.data),
        ("/boat/estop", Bool, lambda m:m.data),
        ("/boat/motor_status", String, lambda m:json.loads(m.data)),
        ("/boat/cmd_vel", Twist, lambda m:(m.linear.x,m.angular.z)),
        ("/boat/motor_preview", Float32MultiArray, lambda m:list(m.data)),
    ]
    channels = []
    def receive(topic, convert, msg):
        value = convert(msg)
        if topic == "/rc_channels":
            channels[:] = value
        hold.update(topic,time.monotonic(),value)
    for topic,cls,convert in inputs:
        node.create_subscription(cls,topic,lambda m,k=topic,f=convert:receive(k,f,m),10)

    def display():
        hold.advance(time.monotonic())
        current = f"CH1={channels[0]} CH2={channels[1]}" if channels else "RC 대기"
        stage = hold.result or ("측정 중" if hold.since is not None else "범위·무장 대기")
        print(f"{current} | 목표 {TARGET} | {stage} | 유지 {hold.elapsed:.1f}s / "
              f"남음 {max(0,HOLD-hold.elapsed):.1f}s", flush=True)
        if hold.result:
            print("중앙 복귀 후 SA KILL로 기록을 종료하세요.",flush=True)
    node.create_timer(.25, display)
    executor = rclpy.get_global_executor()
    executor.add_node(node)
    try:
        while not stopping and rclpy.ok():
            executor.spin_once(timeout_sec=.05)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Synthetic RC receive/shutdown regression. No receiver, motor, bag or GPIO."""
import os
from pathlib import Path
import runpy
import signal
import subprocess
import sys
from unittest.mock import patch

import pytest


ROOT = Path(__file__).resolve().parents[1]


def child(target, timing, signum):
    import rclpy
    from rclpy.node import Node
    from rclpy.publisher import Publisher
    from std_msgs.msg import Int32MultiArray

    path = (ROOT / "rc_preview_trial/axes.py" if target == "axes"
            else ROOT / "install/kaboat_control/lib/kaboat_control/control")
    sys.path.insert(0, str(path.parent))
    frame = Int32MultiArray(data=[992]*4 + [191, 997] + [992]*10)
    subscribe, construct, publish = Node.create_subscription, Int32MultiArray.__init__, Publisher.publish
    received = 0
    sent = False
    final = {}

    def stop():
        nonlocal sent
        if not sent:
            sent = True
            print("delivering signal", timing, flush=True)
            os.kill(os.getpid(), signum)

    def construct_received(self, **kwargs):
        # The sender reuses 'frame'. Confirm that this construction is a real ROS take.
        if (timing == "take" and received >= 20 and
                sys._getframe(1).f_code.co_name == "_take_subscription"):
            stop()
        construct(self, **kwargs)

    def observed_publish(pub, msg):
        publish(pub, msg)
        if sent and pub.topic_name == "/boat/estop":
            final["estop"] = msg.data
        if sent and pub.topic_name == "/boat/cmd_vel":
            final["cmd"] = (msg.linear.x, msg.angular.z)

    def subscribe_and_feed(node, cls, topic, callback, qos, **kwargs):
        if topic != "/rc_channels":
            return subscribe(node, cls, topic, callback, qos, **kwargs)
        def on_frame(message):
            nonlocal received
            assert message.data == frame.data
            received += 1
            callback(message)
        sub = subscribe(node, cls, topic, on_frame, qos, **kwargs)
        pub = node.create_publisher(cls, topic, 10)
        def feed():
            if timing == "between" and received >= 20:
                stop()
            else:
                pub.publish(frame)
        node.create_timer(.004, feed)
        return sub

    with patch.object(Node, "create_subscription", subscribe_and_feed), \
         patch.object(Int32MultiArray, "__init__", construct_received), \
         patch.object(Publisher, "publish", observed_publish):
        runpy.run_path(str(path), run_name="__main__")
    assert sent and received >= 20
    if target == "control":
        assert final == {"estop": True, "cmd": (0., 0.)}


@pytest.mark.parametrize("target", ["control", "axes"])
@pytest.mark.parametrize("timing,signum", [
    ("between", signal.SIGINT), ("take", signal.SIGINT), ("take", signal.SIGTERM)])
def test_receive_shutdown(target, timing, signum, tmp_path):
    env = dict(os.environ, ROS_DOMAIN_ID="116", ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR=str(tmp_path))
    result = subprocess.run(
        [sys.executable, "-B", __file__, target, timing, str(int(signum))],
        env=env, capture_output=True, text=True, timeout=25)
    assert result.returncode == 0, result.stdout + result.stderr


if __name__ == "__main__":
    child(sys.argv[1], sys.argv[2], int(sys.argv[3]))

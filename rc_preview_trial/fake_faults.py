#!/usr/bin/python3
"""Exercise installed arm/command/tick/adapter methods with memory-only devices and clock."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader

base = Path(__file__).resolve().parents[1]/"install/kaboat_motor/lib/kaboat_motor"
sys.path.insert(0,str(base))
loader=SourceFileLoader("trial_motor",str(base/"boat_motor"))
motor=module_from_spec(spec_from_loader(loader.name,loader))
loader.exec_module(motor)
from pca_backend import PCAOutput
from esc_config import validate,controller_for_profile,duty_from_us
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Channel:
    def __init__(self):
        self.values=[]
        self.fail=False

    @property
    def duty_cycle(self):
        return self.values[-1]

    @duty_cycle.setter
    def duty_cycle(self,value):
        if self.fail:
            raise OSError("injected CH1 fault")
        self.values.append(value)


class Harness:
    # Use original node methods, bypass ROS Node and device-opening constructors only.
    enable_permitted=motor.BoatMotorNode.enable_permitted
    read_estop=motor.BoatMotorNode.read_estop
    emit=motor.BoatMotorNode.emit
    tick=motor.BoatMotorNode.tick
    command=motor.BoatMotorNode.command
    estop_cb=motor.BoatMotorNode.estop_cb
    arm_cb=motor.BoatMotorNode.arm_cb

    def __init__(self):
        self.profile=validate(json.loads((base/"preview_bidirectional.json").read_text()))
        self.controller=controller_for_profile(self.profile)
        self.channels=[Channel(),Channel()]
        self.bridge=object.__new__(PCAOutput)
        self.bridge.profile=self.profile
        self.bridge.pca=NS(channels=self.channels)
        self.bridge.oe=NS(on=lambda:setattr(self,"disabled",True),off=lambda:setattr(self,"disabled",False))
        self.bridge.actual_frequency=50.
        self.bridge.failed=False
        self.bridge_ready=True
        self.bench_gate=None
        self.estop_pin=NS(is_active=True)
        self.stop_requested=self.ros_estop=False
        self.last_estop=self.last_reason=None
        self.deadline=20.
        self.now=self.previous=self.last_status=1.
        self.preview_pub, self.status_pub = Mock(), Mock()
        self.args=NS(backend="pca9685")
        self.count_publishers=lambda topic:1
        self.get_logger=Mock()
        self.disabled=True

    def arm(self):
        self.estop_cb(Bool(data=False))
        assert self.controller.reset()
        self.command(Twist())
        result=self.arm_cb(NS(data=True),NS())
        assert result.success and not self.disabled

    def step(self,keep_estop=True):
        self.now+=.05
        if keep_estop:
            self.estop_cb(Bool(data=False))
        msg=Twist()
        msg.linear.x=.03
        self.command(msg)
        self.tick()


def exercise(case):
    h=Harness()
    with patch.object(motor.time,"monotonic",lambda:h.now):
        h.arm()
        if case=="estop 수신 중단":
            for _ in range(65): h.step()
            for _ in range(12): h.step(False)
            h.estop_cb(Bool(data=False))
            return h.disabled and h.controller.estop and not h.controller.armed, f"estop 공백 0.60초, cmd 계속, armed={h.controller.armed}, OE HIGH 요청={h.disabled}"
        if case=="복귀 heartbeat가 tick보다 먼저 도착":
            h.now += .51
            h.estop_cb(Bool(data=False))
            h.command(Twist())
            res = h.arm_cb(NS(data=True),NS())
            return (h.disabled and h.controller.estop and not res.success), "래치 유지 및 reset 전 재무장 거부"
        if case=="초기화 중 명령 중단":
            for _ in range(12):
                h.now += .05
                h.estop_cb(Bool(data=False))
                h.tick()
            return h.disabled and not h.controller.armed, h.controller.reason
        if case=="초기화 중 비영점":
            neutral=duty_from_us(1500,50)
            while h.now+.05<h.bridge.ready_at:
                h.step()
            held=all(set(ch.values)=={neutral} for ch in h.channels)
            assert h.controller.target == h.controller.output == (0.,0.)
            h.now = h.bridge.ready_at + .001
            h.tick()  # No post-init command yet.
            held = held and h.controller.output == (0.,0.) and set(h.channels[0].values)=={neutral}
            h.step()
            first=h.preview_pub.publish.call_args.args[0].data[0]
            return held and 0<first<=.015001, f"3초 대기 중 양쪽 중립={held}, 개방 첫 논리값={first:.4f}, 50ms 한 단계 상한=0.015"
        if case=="초기화 중 KILL":
            h.step()
            h.estop_cb(Bool(data=True))
            for _ in range(65): h.step()
            return (h.disabled and h.controller.estop and not h.controller.armed
                    and all(set(ch.values)=={duty_from_us(1500,50)} for ch in h.channels)), f"3초 경과 뒤 armed={h.controller.armed}, latch={h.controller.estop}, OE HIGH 요청={h.disabled}"
        for _ in range(65): h.step()
        h.channels[1].fail=True
        left,right=[len(ch.values) for ch in h.channels]
        try:
            h.step()
        except OSError:
            partial=len(h.channels[0].values)==left+1 and len(h.channels[1].values)==right
            try:
                h.bridge.write(0,0)
            except RuntimeError:
                return partial and h.disabled and h.bridge.failed, f"CH0 쓰기 뒤 CH1 오류={partial}, 어댑터 fault={h.bridge.failed}, OE HIGH 요청={h.disabled}, 후속 쓰기 거부"
        return False,"쓰기 오류 격리 확인 실패"


if __name__=="__main__":
    results=[]
    for case in ("estop 수신 중단","초기화 중 비영점","초기화 중 KILL","한쪽 PWM 쓰기 실패",
                 "복귀 heartbeat가 tick보다 먼저 도착","초기화 중 명령 중단"):
        ok,evidence=exercise(case)
        print(case,"PASS" if ok else "FAIL",evidence)
        results.append(ok)
    sys.exit(0 if all(results) else 1)

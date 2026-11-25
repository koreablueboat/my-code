#!/usr/bin/python3
"""Acquire the first disconnected neutral → forward → SA KILL run."""
import argparse
from contextlib import ExitStack
from datetime import datetime
import fcntl
import json
import os
import select
import subprocess
import termios
from pathlib import Path
import signal
import sys
import time

from guidance import DIRECTIONS, matches_direction, Guide, CUES, Incomplete, awaiting_status, check_operator_input

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pwm_ab'))
from run import device_check, launch, require, space, static_check, DRIVE, INSTALL

RESULTS = Path(__file__).resolve().parent / 'results'
MOTOR = INSTALL / 'kaboat_motor/lib/kaboat_motor/boat_motor'
CONTROL = INSTALL / 'kaboat_control/lib/kaboat_control/control'
TOPICS = '/rc_channels /rc_uplink_lq /boat/cmd_vel /boat/estop /boat/motor_status /boat/motor_preview /rosout'.split()


def clock_pair():
    before = time.monotonic_ns()
    wall = time.time_ns()
    return dict(mono_before=before, wall=wall, mono_after=time.monotonic_ns())


def states(path):
    for line in path.read_text(errors='replace').splitlines():
        i = line.find('{')
        if i >= 0:
            try:
                yield json.loads(line[i:])
            except json.JSONDecodeError:
                continue  # the logger may be writing the last line


class Inputs:
    def __init__(self, node):
        from std_msgs.msg import Int32MultiArray, UInt8, String, Float32MultiArray
        sys.path.insert(0, str(CONTROL.parent))
        from supervisor import Supervisor
        config = json.loads((CONTROL.parent/'control_config.json').read_text())
        self.rc = Supervisor(config, expected_backend='bench_variable')
        self.axes = []
        self.motor = None
        self.motor_at = 0.
        self.first_status_at = None
        self.operator_phase = None
        def channels(msg):
            self.axes = list(msg.data)
            now = time.monotonic()
            self.rc.set_rc(self.axes, now)
            if self.rc.rc_at != now:
                self.rc.rc_at = None
            if self.operator_phase:
                check_operator_input(self.operator_phase, self.rc)
        def status(msg):
            self.motor = json.loads(msg.data)
            require(self.motor['backend'] == 'bench_variable', '다른 모터 백엔드 발견')
            self.motor_at = time.monotonic()
            if self.first_status_at is None:
                self.first_status_at = self.motor_at
        node.create_subscription(Int32MultiArray, '/rc_channels', channels, 1)
        node.create_subscription(UInt8, '/rc_uplink_lq',
                                 lambda m: self.rc.set_lq(m.data, time.monotonic()), 1)
        node.create_subscription(String, '/boat/motor_status', status, 1)
        # Advertise the type before motor startup so bag can subscribe ahead of it.
        node.create_subscription(Float32MultiArray, '/boat/motor_preview', lambda _: None, 1)

    def link_ok(self, now):
        rc = self.rc
        return (rc.rc_at is not None and now-rc.rc_at <= .30 and rc.lq_at is not None
                and now-rc.lq_at <= 1. and 1 <= rc.lq <= 100)

    def display(self):
        values = [self.axes[i] if i < len(self.axes) else '-' for i in (0,1,4,5)]
        return 'CH1/2/5/6=' + '/'.join(map(str, values)) + f' LQ={self.rc.lq or 0}'


def acquire(path, meta):
    import gpiod
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    jobs = {}
    def start(stack, name, args, err=None):
        jobs[name] = launch(stack, args, path/(name+'.log'), err, stdin=subprocess.DEVNULL)
    def check():
        space(path)
        require(all(p.poll() is None for p in jobs.values()), '시험 프로세스 조기 종료 또는 기한 만료')
        require(not (path/'gpiomon.err').stat().st_size, 'GPIO 기록 오류')
    try:
        with ExitStack() as stack:
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            stack.callback(rclpy.try_shutdown)
            node = rclpy.create_node('bench_variable_observer')
            stack.callback(node.destroy_node)
            inputs = Inputs(node)
            start(stack, 'edges', ['gpiomon','--line-buffered','--format=%o %e %s.%n','gpiochip0','24','25'], path/'gpiomon.err')
            time.sleep(.3)
            with gpiod.Chip('gpiochip0') as chip:
                require(all(chip.get_line(n).consumer() == 'gpiomon' for n in (24,25)), '계측 점유 실패')
            meta['recorder_start'] = clock_pair()
            start(stack, 'bag', ['ros2','bag','record','--include-unpublished-topics','-o',str(path/'bag'),*TOPICS])
            start(stack, 'drive', [DRIVE,'--ros-args','-p','port:=/dev/ttyAMA1','-p','baud:=420000'])
            start(stack, 'control', [CONTROL,'--ros-args','-p','expected_backend:=bench_variable'])
            discover_by = time.monotonic() + 30
            bag_ready = prompted = False
            safe_since = None
            refresh = next_check = 0.
            while True:
                rclpy.spin_once(node, timeout_sec=.02)
                now = time.monotonic()
                if now < next_check:
                    continue
                next_check = now + .05
                check()
                require(inputs.motor is None and not node.count_publishers('/boat/motor_status'),
                        '기동 전 다른 모터 상태 발행자 발견')
                if not bag_ready:
                    text = (path/'bag.log').read_text(errors='replace')
                    bag_ready = all(f"Subscribed to topic '{t}'" in text for t in TOPICS)
                    require(bag_ready or now < discover_by, '기동 전 bag 구독 준비 시간 초과')
                safe = (bag_ready and inputs.link_ok(now) and inputs.rc.mode == 'MANUAL'
                        and inputs.rc.permit is False and inputs.rc.sticks_neutral)
                if not safe:
                    safe_since = None
                elif safe_since is None:
                    safe_since = now
                ready = safe and now - safe_since >= 1.1
                if ready and not prompted:
                    termios.tcflush(sys.stdin, termios.TCIFLUSH)
                    prompted = True
                if not safe:
                    prompted = False
                if now >= refresh:
                    cue = ('세 곳 분리·조작 준비 확인 후 Enter (기동만 시작, SA KILL·중앙 유지)' if ready else
                           'bag 준비 대기' if not bag_ready else 'MANUAL·SA KILL·중앙·링크 확인')
                    print(f'\r\033[K[기동 전 | 시간 미사용] {inputs.display()} | {cue}', end='', flush=True)
                    refresh = now + .2
                if prompted and select.select([sys.stdin], [], [], 0)[0]:
                    require(sys.stdin.readline() == '\n', '사용자 준비 취소')
                    check()
                    meta['operator_ready'] = clock_pair()
                    break
            print('\n모터 기동. SA 허용 안내까지 중앙·KILL 유지.', flush=True)
            inputs.operator_phase = '모터 기동'
            meta['motor_launch'] = clock_pair()
            launched = time.monotonic()
            start(stack, 'motor', [MOTOR,'--backend','bench_variable','--duration','20','--disconnected'])
            def notify_stop(exc_type, exc, traceback):
                meta['motor_stop_requested'] = clock_pair()
                meta['first_status_rx_mono'] = inputs.first_status_at
                if exc_type:
                    print(f'\n시험 종료 요청 — SA KILL, 스틱 중앙 유지. {exc}', flush=True)
            stack.push(notify_stop)
            guide = Guide(direction=meta['direction'])
            previous = None
            discovered = False
            while True:
                rclpy.spin_once(node, timeout_sec=.02)
                now = time.monotonic()
                if now < next_check:
                    continue
                next_check = now + .05
                check()
                require(inputs.link_ok(now) and inputs.rc.mode == 'MANUAL', 'RC·LQ 또는 MANUAL 상실')
                motor = inputs.motor
                if not discovered and node.count_publishers('/boat/motor_status'):
                    discovered = True
                    meta['first_status_writer_seen'] = clock_pair()
                if not motor:
                    remaining, label = awaiting_status(now, launched, discovered)
                    cue = '조작 대기 — SA KILL·두 스틱 중앙 유지'
                else:
                    require(now-inputs.motor_at < 1., '모터 상태 수신 중단')
                    remaining = motor['deadline_mono'] - now
                    history = list(states(path/'control.log'))
                    transition = history[-1] if history else {}
                    kills = [s for s in history if s.get('previous_phase') == 'RUNNING'
                             and s.get('phase') == 'KILLED']
                    if kills and (guide.phase not in ('SA KILL','차단 관찰')
                                  or kills[-1]['reason'] != 'radio_kill'):
                        raise Incomplete('예정 전 차단 또는 SA 이외 정지')
                    matched = matches_direction(guide.direction, inputs.rc.v, inputs.rc.w)
                    finished = guide.advance(now, motor, transition, inputs.rc, matched)
                    if finished:
                        meta['motor_alive_observation_end'] = clock_pair()
                        check()
                        break
                    label, cue = guide.phase, CUES[guide.phase]
                    if guide.direction != 'forward' and label == 'SA KILL':
                        cue = '두 스틱 중앙 유지, 종료 차단을 위해 지금 SA KILL'
                if now >= refresh or label != previous:
                    if label != previous:
                        print()
                    if label != previous:
                        meta.setdefault('cues', []).append(dict(stage=label, mono=now))
                    counts = ('상태 미수신' if not motor else
                              '레지스터 미확인' if motor['read_counts'] is None else motor['read_counts'])
                    prefix = '' if motor else '추정 '
                    print(f'\r\033[K[{label} | {prefix}{remaining:4.1f}초 남음] {inputs.display()}'
                          f' PWM={counts} | {cue}', end='', flush=True)
                    refresh, previous = now + .2, label
                    inputs.operator_phase = guide.phase if motor else '모터 기동'
            print('\n차단 관찰 완료. SA KILL 유지. 기록 종료 중.', flush=True)
            meta['capture_end'] = clock_pair()
    finally:
        meta['exit_codes'] = {name:p.poll() for name,p in jobs.items()}
    require(all(rc in (0,-signal.SIGINT) for rc in meta['exit_codes'].values()), '프로세스 종료 오류')
    require(jobs['edges'].returncode == jobs['bag'].returncode == 0 and
            (path/'bag/metadata.yaml').is_file() and not (path/'gpiomon.err').stat().st_size, '기록 종료 오류')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--disconnected', action='store_true', required=True)
    parser.add_argument('--direction', choices=DIRECTIONS, default='forward')
    args = parser.parse_args()
    require(sys.stdin.isatty() and sys.stdout.isatty(), '현장 대화형 터미널에서 실행하세요')
    os.environ.update(ROS_DOMAIN_ID='122', ROS_LOCALHOST_ONLY='1', RCUTILS_LOGGING_BUFFERED_STREAM='0')
    static_check(RESULTS)
    RESULTS.mkdir(exist_ok=True)
    with (RESULTS/'.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        device_check()
        path = RESULTS / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + args.direction)
        path.mkdir()
        meta = {'complete':False, 'direction':args.direction, 'ros_domain_id':os.environ['ROS_DOMAIN_ID'],
                'ros_localhost_only':os.environ['ROS_LOCALHOST_ONLY']}
        print(path, 'SB MANUAL, SA KILL, 두 스틱 중앙 유지.', flush=True)
        try:
            acquire(path, meta)
            meta['complete'] = True  # recording complete; PWM verdict requires offline analysis
        except (RuntimeError, ValueError, KeyboardInterrupt) as exc:
            meta['incomplete_reason'] = str(exc) or '사용자 중단'
            print('\n시험 중단. SA KILL·스틱 중앙 유지.', meta['incomplete_reason'], flush=True)
            raise SystemExit(1)
        finally:
            meta['end'] = clock_pair()
            (path/'run.json').write_text(json.dumps(meta, indent=2)+'\n')
        if args.direction != 'forward':
            from direction_report import report
            summary = report(path)
            (path/'analysis.txt').write_text(summary+'\n')
            print(summary)


if __name__ == '__main__':
    def interrupt(sig, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    main()

#!/usr/bin/python3
"""Attended single-left-T200 trial; default action only checks the approval profile."""
import argparse
from contextlib import ExitStack
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'pwm_ab'))
from run import device_check, launch, DRIVE, INSTALL

MOTOR = INSTALL/'kaboat_motor/lib/kaboat_motor/boat_motor'
CONTROL = INSTALL/'kaboat_control/lib/kaboat_control/control'
CANDIDATE = MOTOR.with_name('left_water_candidate.json')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=CANDIDATE)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    motor = [MOTOR,'--backend','left_t200_water','--duration','20','--profile',args.profile.resolve()]
    # The motor's own admission checks run before any device or child controller.
    result = subprocess.run([*motor,'--check-profile'])
    if result.returncode or not args.run:
        return result.returncode
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        parser.error('대화형 현장 터미널 필요')
    os.environ.update(ROS_DOMAIN_ID='123', ROS_LOCALHOST_ONLY='1')
    device_check()
    path = Path(__file__).resolve().parent/'results'/datetime.now().strftime('%Y%m%d_%H%M%S')
    path.mkdir(parents=True, exist_ok=False)
    print('로그', path, flush=True)
    os.environ['ROS_LOG_DIR'] = str(path)
    with ExitStack() as stack:
        rx = launch(stack,[DRIVE,'--ros-args','-p','port:=/dev/ttyAMA1','-p','baud:=420000'],
                    path/'drive.log',stdin=subprocess.DEVNULL)
        control = launch(stack,[CONTROL,'--ros-args','-p','expected_backend:=left_t200_water'],
                         path/'control.log',stdin=subprocess.DEVNULL)
        proc = subprocess.Popen(motor)
        try:
            while proc.poll() is None:
                if rx.poll() is not None or control.poll() is not None:
                    raise RuntimeError('RC/제어기 종료. SA KILL, 독립 추진 전원 차단.')
                time.sleep(.1)
            return proc.returncode
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(2)
                except subprocess.TimeoutExpired:
                    print('모터 프로세스 종료 지연. 즉시 독립 추진 전원을 차단하세요.', flush=True)
                    proc.terminate()
                    proc.wait(3)
            print('SA KILL·중앙 유지. 독립 차단기로 추진 전원 차단 후 접근하세요.', flush=True)


if __name__ == '__main__':
    def interrupt(sig, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    sys.exit(main())

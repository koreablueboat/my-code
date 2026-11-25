#!/usr/bin/python3
"""T200 수중 단발 시험."""
import argparse
from contextlib import ExitStack
import os
import select
import signal
import sys
import termios
import time

from bench_pca import ADDRESS, LED0, NEUTRAL, prepare, request_oe, shutdown

TIMEOUT = 30


def confirmed(fd, side):
    phrase = f'RUN {side.name}'
    expected = (phrase + '\n').encode('ascii')
    # Discard lines typed before the neutral confirmation prompt.
    termios.tcflush(fd, termios.TCIFLUSH)
    deadline = time.monotonic() + TIMEOUT
    print(f"초기화 2음과 정지를 확인하면 {TIMEOUT}초 안에 {phrase} 입력 후 Enter. "
          "그 외 입력 또는 Ctrl+C는 취소.", flush=True)
    remaining = deadline - time.monotonic()
    if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
        return False
    response = os.read(fd, len(expected) + 1)
    return response == expected and time.monotonic() < deadline


def run_once(oe, bus, fd, side):
    register = LED0 + 4 * side.channel
    try:
        prepare(bus, (side.channel,))
        oe.set_value(0)
        if not confirmed(fd, side):
            return False
        bus.write_i2c_block_data(ADDRESS, register, [0, 0, 315 & 0xff, 315 >> 8])
        time.sleep(0.5)
        bus.write_i2c_block_data(ADDRESS, register, NEUTRAL)
        time.sleep(2)
        return True
    finally:
        shutdown(oe, bus)


def terminate(signum, frame):
    raise SystemExit(128 + signum)


def main(side):
    parser = argparse.ArgumentParser(description=f"{side.label} {__doc__}", allow_abbrev=False)
    parser.add_argument('--hardware', action='store_true', help='실물 수중 시험')
    if not parser.parse_args().hardware:
        print(f"DRY-RUN | GPIO/I²C 접근 없음 | PRESCALE=0x8A | {side.inactive} FULL OFF | "
              f"CH{side.channel} 307 → 현장 확인(RUN {side.name}, {TIMEOUT} s) → 315 × 0.5 s → 307 × 2 s → OE HIGH")
        return 0
    if not sys.stdin.isatty():
        parser.error('현장 입력을 위한 대화형 터미널이 필요합니다.')
    fd = sys.stdin.fileno()
    if not termios.tcgetattr(fd)[3] & termios.ICANON:
        parser.error('Enter로 입력을 완료하는 터미널이 필요합니다.')
    import gpiod
    import smbus

    signal.signal(signal.SIGHUP, terminate)
    signal.signal(signal.SIGTERM, terminate)
    with ExitStack() as stack:
        _, oe = request_oe(stack, gpiod, f'{side.name.lower()}-t200-once')
        bus = smbus.SMBus(1)
        stack.callback(bus.close)
        completed = run_once(oe, bus, fd, side)
    print("단발 명령 완료, 출력 차단." if completed else "현장 확인 취소 또는 시간 초과, 출력 차단.")
    return 0 if completed else 1


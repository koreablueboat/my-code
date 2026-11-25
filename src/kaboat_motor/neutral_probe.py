#!/usr/bin/python3
"""Calibrated PCA9685 neutral bench measurement or single ESC neutral initialization."""
import argparse
from contextlib import ExitStack
import select
import signal
from statistics import median
import time

from bench_pca import (ADDRESS, AUTO_INCREMENT, BENCH_PRESCALE, FULL_OFF, LED0, MODE1,
                       MODE2, NEUTRAL, NEUTRAL_COUNTS, PRESCALE, Side, prepare, request_oe, shutdown)

SECOND = 1_000_000_000
MARGIN = SECOND // 5
WIDTH_MIN_NS, WIDTH_MAX_NS = 1_475_000, 1_525_000
PERIOD_NS = 20_000_000  # 50 Hz target
COINCIDENCE_NS = 1_000_000


def capture_until(lines, events, deadline):
    while True:
        remaining = deadline - time.monotonic_ns()
        if remaining <= 0:
            return
        ready, _, _ = select.select(list(lines), [], [], remaining / SECOND)
        for fd in ready:
            ch, line = lines[fd]
            for edge in line.event_read_multiple():
                events[ch].append((edge.sec * SECOND + edge.nsec,
                                   edge.type == edge.RISING_EDGE))


def measure(oe, bus, capture=None, *, side=None):
    try:
        settings = prepare(bus, (side.channel,) if side is not None else (0, 1))
        if side is None:
            capture(time.monotonic_ns() + MARGIN)
        start = time.monotonic_ns()
        oe.set_value(0)
        if side is not None:
            time.sleep(7)
        else:
            capture(start + 2 * SECOND)
    finally:
        stop = shutdown(oe, bus, None if side is not None else lambda t: capture(t + MARGIN))
    return start, stop, settings


def hardware(*, side=None):
    import gpiod
    import smbus

    events = [[], []]
    with ExitStack() as stack:
        chip, oe = request_oe(stack, gpiod, 'neutral-probe')
        if side is None:
            lines = {}
            for ch, pin in enumerate((24, 25)):
                line = chip.get_line(pin)
                line.request(consumer='neutral-probe', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
                stack.callback(line.release)
                lines[line.event_get_fd()] = ch, line
        bus = smbus.SMBus(1)
        stack.callback(bus.close)
        if side is not None:
            start, stop, (prescale, counts) = measure(oe, bus, side=side)
            print(f"{side.label} ESC 중립 | PRESCALE=0x{prescale:02X}, CH{side.channel}={counts[0]} 카운트, {side.inactive} FULL OFF"
                  f" | OE LOW/HIGH 요청 구간 {(stop-start)/SECOND:.6f} s | 펄스 및 ESC 상태 미측정")
            return 0
        start, stop, (prescale, counts) = measure(oe, bus, lambda end: capture_until(lines, events, end))
    print(f"설정 읽기 확인 | PRESCALE=0x{prescale:02X}, CH0={counts[0]}, CH1={counts[1]} 카운트")
    print(f"OE LOW/HIGH 요청 구간 {(stop-start)/SECOND:.6f} s, OE 단자 전압 미측정")
    short = [[], []]
    results = [summarize(ch, trace, start, stop, short[ch]) for ch, trace in enumerate(events)]
    compare_short(short)
    return 0 if all(results) else 1


def inspect_edges(trace, start, stop):
    # Preserve kernel delivery order. An invalid edge breaks the pending pair.
    rises = []
    falls = {}
    issues = []
    pending = None
    latest = None
    for stamp, rising in trace:
        if stamp > stop + MARGIN:
            continue
        if latest is not None and stamp <= latest:
            issues.append(f"시각 순서 이상 {latest} → {stamp} ns")
            pending = None
            continue
        latest = stamp
        if rising:
            if pending is not None:
                issues.append(f"연속 상승 {pending} → {stamp} ns (하강 누락 의심)")
            if rises and start <= rises[-1] < stamp < stop:
                gap = stamp - rises[-1]
                if gap < PERIOD_NS // 2 or gap > 3 * PERIOD_NS // 2:
                    issues.append(f"상승 간격 이상 {rises[-1]} → {stamp} ns ({gap/1000:.3f} µs)")
            rises.append(stamp)
            pending = stamp
        elif pending is None:
            issues.append(f"짝 없는 하강 {stamp} ns")
        else:
            falls[pending] = stamp
            pending = None
    if pending is not None:
        issues.append(f"짝 없는 마지막 상승 {pending} ns")
    return rises, falls, issues


def compare_short(short):
    print("내부 짧은 펄스 시점 비교 (상승 시각 차이 ±1 ms 이내)")
    if not any(short):
        print("비교 대상 없음")
    matched = set()
    for left, t0 in short[0]:
        candidates = [(right, t1 - t0) for right, t1 in short[1]
                      if abs(t1 - t0) <= COINCIDENCE_NS]
        if not candidates:
            print(f"CH0 #{left} 대응 없음")
        for right, delta in candidates:
            print(f"CH0 #{left} ↔ CH1 #{right}, 상승 차이 CH1−CH0 {delta/1000:+.3f} µs")
            matched.add(right)
    for right, _ in short[1]:
        if right not in matched:
            print(f"CH1 #{right} 대응 없음")


def stats(samples):
    if not samples:
        return '측정 불충분'
    return '/'.join(f'{v/1000:.1f}' for v in (min(samples), median(samples), max(samples)))


def summarize(ch, trace, start, stop, short=None):
    rises, falls, issues = inspect_edges(trace, start, stop)
    before = [r for r in falls if r < start]
    after = [r for r in falls if stop <= r]
    active_rises = [r for r in rises if start <= r < stop]
    active = [(r, falls[r]) for r in active_rises if r in falls]
    widths = [fall - rise for rise, fall in active]
    periods = [b - a for a, b in zip(active_rises, active_rises[1:])]
    repeated = '있음' if len(after) >= 2 else '관찰 안 됨'
    if not active and not after:
        repeated = '측정 불충분'
    print(f"CH{ch} 사전 {len(before)}, 출력 중 {len(active)}, 차단 후 200 ms {len(after)}펄스"
          f" (반복 {repeated}) | 최소/중앙/최대 µs 폭 {stats(widths)}, 주기 {stats(periods)}")

    def width_label(rise):
        return f'{(falls[rise]-rise)/1000:.1f}' if rise in falls else '측정 불충분'

    first = width_label(active_rises[0]) if active_rises else '측정 불충분'
    last = width_label(active_rises[-1]) if active_rises else '측정 불충분'
    inside = []
    outliers = []
    for i, rise in enumerate(active_rises[1:-1], start=1):
        if rise not in falls:
            continue
        width = falls[rise] - rise
        inside.append(width)
        if not WIDTH_MIN_NS <= width <= WIDTH_MAX_NS:
            outliers.append(i + 1)
            print(f"CH{ch} #{i+1} 폭 {width/1000:.3f} µs | 커널 상승 {rise} ns, "
                  f"하강 {falls[rise]} ns | 직전/직후 상승 간격 "
                  f"{(rise-active_rises[i-1])/1000:.3f}/{(active_rises[i+1]-rise)/1000:.3f} µs")
            if short is not None and width < WIDTH_MIN_NS:
                short.append((i + 1, rise))
    print(f"CH{ch} 폭 µs | 첫 {first}, 마지막 {last}, "
          f"내부 최소/중앙/최대 {stats(inside)}")
    numbers = ', '.join(f'#{n}' for n in outliers) or '없음'
    print(f"CH{ch} 내부 유효 {len(inside)}개 중 1475~1525 µs 밖 {len(outliers)}개 "
          f"| 활성 상승 기준 번호 {numbers}")
    print(f"CH{ch} 에지 기록 이상 {len(issues)}건")
    for issue in issues:
        print(f"CH{ch} {issue}")
    return len(active) >= 2 and not before and not after and not issues


def terminate(signum, frame):
    raise SystemExit(128 + signum)


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--hardware', action='store_true', help='execute selected mode on hardware')
    modes = parser.add_mutually_exclusive_group()
    for side in Side:
        modes.add_argument(f'--{side.name.lower()}-esc-neutral', action='store_const', const=side,
                           dest='side', help=f'hold CH{side.channel} neutral for 7 s without loopback')
    args = parser.parse_args()
    side = args.side
    if not args.hardware:
        if side is not None:
            print(f"DRY-RUN | GPIO/I²C 접근 없음 | {side.label} ESC 중립 7 s | PRESCALE=0x{BENCH_PRESCALE:02X}, "
                  f"CH{side.channel}={NEUTRAL_COUNTS} 카운트, {side.inactive} FULL OFF | BCM24/25 미사용")
            return 0
        print("DRY-RUN | GPIO/I²C 접근 없음 | CH0/CH1 50 Hz, 1500 µs, 2 s 목표"
              f" | 예정 PRESCALE=0x{BENCH_PRESCALE:02X}, CH0={NEUTRAL_COUNTS}, CH1={NEUTRAL_COUNTS} 카운트"
              " | BCM24/25 펄스 및 OE 전압 미측정")
        return 0
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGHUP, terminate)
    return hardware(side=side)


if __name__ == '__main__':
    raise SystemExit(main())

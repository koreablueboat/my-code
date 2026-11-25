#!/usr/bin/python3
"""Fixed neutral pulse bench test for an isolated PCA9685, with GPIO loopback."""
import argparse
from contextlib import ExitStack
import select
import signal
from statistics import median
import time

ADDRESS = 0x40
MODE1, MODE2, LED0, PRESCALE = 0, 1, 6, 0xFE
SLEEP, AUTO_INCREMENT = 0x10, 0x20
FULL_OFF = [0, 0, 0, 0x10]
NEUTRAL = [0, 0, 0x33, 1]  # 307 counts at nominal 25 MHz, prescale 121
SECOND = 1_000_000_000
MARGIN = SECOND // 5


def prepare(bus):
    if bus.read_byte_data(ADDRESS, MODE1) & 0x40:
        raise RuntimeError("External oscillator selected")
    bus.write_byte_data(ADDRESS, MODE1, AUTO_INCREMENT | SLEEP)
    bus.write_byte_data(ADDRESS, PRESCALE, 121)
    bus.write_byte_data(ADDRESS, MODE2, 4)  # non-inverted, OE HIGH drives low
    for ch in range(16):
        bus.write_i2c_block_data(ADDRESS, LED0 + 4 * ch, FULL_OFF)
    for ch in (0, 1):
        bus.write_i2c_block_data(ADDRESS, LED0 + 4 * ch, NEUTRAL)
    bus.write_byte_data(ADDRESS, MODE1, AUTO_INCREMENT)
    time.sleep(.001)
    for reg, value in ((MODE1, AUTO_INCREMENT), (MODE2, 4), (PRESCALE, 121)):
        if bus.read_byte_data(ADDRESS, reg) != value:
            raise RuntimeError(f"Configuration readback failed at register {reg:#x}")
    for ch in range(16):
        expected = NEUTRAL if ch < 2 else FULL_OFF
        if bus.read_i2c_block_data(ADDRESS, LED0 + 4 * ch, 4) != expected:
            raise RuntimeError(f"CH{ch} readback failed")


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


def measure(oe, bus, capture):
    try:
        prepare(bus)
        capture(time.monotonic_ns() + MARGIN)
        start = time.monotonic_ns()
        oe.set_value(0)
        capture(start + 2 * SECOND)
    finally:
        try:
            oe.set_value(1)
            stop = time.monotonic_ns()
            capture(stop + MARGIN)
        finally:
            # Single-register writes also work if setup failed before AI was set.
            try:
                bus.write_byte_data(ADDRESS, 0xFD, 0x10)  # ALL_LED_OFF_H
            finally:
                bus.write_byte_data(ADDRESS, MODE1, AUTO_INCREMENT | SLEEP)
    return start, stop


def hardware():
    import gpiod
    import smbus

    events = [[], []]
    with ExitStack() as stack:
        chip = stack.enter_context(gpiod.Chip('gpiochip0'))
        if chip.label() != 'pinctrl-bcm2711':
            raise RuntimeError(f"Unexpected gpiochip0 controller: {chip.label()}")
        oe = chip.get_line(17)
        oe.request(consumer='neutral-probe', type=gpiod.LINE_REQ_DIR_OUT, default_val=1)
        stack.callback(oe.release)
        lines = {}
        for ch, pin in enumerate((24, 25)):
            line = chip.get_line(pin)
            line.request(consumer='neutral-probe', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
            stack.callback(line.release)
            lines[line.event_get_fd()] = ch, line
        bus = smbus.SMBus(1)
        stack.callback(bus.close)
        start, stop = measure(oe, bus, lambda end: capture_until(lines, events, end))
    print(f"OE LOW/HIGH 요청 구간 {(stop-start)/SECOND:.6f} s, OE 단자 전압 미측정")
    results = [summarize(ch, trace, start, stop) for ch, trace in enumerate(events)]
    return 0 if all(results) else 1


def pulses(trace):
    rise = None
    pairs = []
    for stamp, rising in trace:
        if rising:
            rise = stamp
        elif rise is not None:
            pairs.append((rise, stamp))
            rise = None
    return pairs


def stats(samples):
    if not samples:
        return '측정 불충분'
    return '/'.join(f'{v/1000:.1f}' for v in (min(samples), median(samples), max(samples)))


def summarize(ch, trace, start, stop):
    before = pulses([(t, r) for t, r in trace if t < start])
    active = pulses([(t, r) for t, r in trace if start <= t < stop])
    after = pulses([(t, r) for t, r in trace if stop <= t <= stop + MARGIN])
    widths = [fall - rise for rise, fall in active]
    periods = [b[0] - a[0] for a, b in zip(active, active[1:])]
    repeated = '있음' if len(after) >= 2 else '관찰 안 됨'
    if not active and not after:
        repeated = '측정 불충분'
    print(f"CH{ch} 사전 {len(before)}, 출력 중 {len(active)}, 차단 후 200 ms {len(after)}펄스"
          f" (반복 {repeated}) | 최소/중앙/최대 µs 폭 {stats(widths)}, 주기 {stats(periods)}")
    return bool(periods) and not before and not after


def terminate(signum, frame):
    raise SystemExit(128 + signum)


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--hardware', action='store_true', help='run isolated PCA bench measurement')
    if not parser.parse_args().hardware:
        print("DRY-RUN | GPIO/I²C 접근 없음 | CH0/CH1 50 Hz, 1500 µs, 2 s 목표"
              " | BCM24/25 펄스 및 OE 전압 미측정")
        return 0
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGHUP, terminate)
    return hardware()


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/python3
"""Offline analysis of the 30 s fixed-neutral bench windows."""
import argparse
import bisect
import json
import math
from pathlib import Path
import sqlite3
import statistics

ROOT = Path(__file__).resolve().parent / "results"
ORDER = "A1 B1 B2 A2 B3 A3 A4 B4 C1 C2 C3 C4".split()
SECOND = 1_000_000_000
PINS = (24, 25)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load_edges(path):
    edges = {pin: [] for pin in PINS}
    for number, line in enumerate(path.read_text().splitlines(), 1):
        pin, level, stamp = line.split()
        sec, ns = stamp.split(".")
        pin, level = int(pin), int(level)
        require(pin in edges and level in (0, 1) and sec.isdigit()
                and ns.isdigit() and len(ns) <= 9, f"잘못된 에지 행 {number}")
        t = int(sec) * SECOND + int(ns)  # gpiomon %n is an integer nanosecond field
        require(not edges[pin] or edges[pin][-1][0] < t, f"BCM{pin} 시각 역전")
        edges[pin].append((t, level))
    return edges


def channel(edges, lo, hi):
    rises = []
    widths = {}
    bad = 0
    for i, (t, level) in enumerate(edges):
        if not lo <= t < hi:
            continue
        if level:
            rises.append(t)
            if i + 1 < len(edges) and edges[i + 1][1] == 0:
                widths[t] = (edges[i + 1][0] - t) / 1000
            else:
                bad += 1
        elif i == 0 or edges[i - 1][1] == 0:
            bad += 1
    gaps = sum(not 10_000_000 <= v-u <= 30_000_000 for u, v in zip(rises, rises[1:]))
    covered = (bool(rises) and rises[0] - lo <= 30_000_000 and hi - rises[-1] <= 30_000_000
               and edges[0][0] < lo and edges[-1][0] >= hi)
    return widths, bad, gaps, covered


def load_bag(path, start, end):
    files = list(path.glob("*.db3"))
    require((path / "metadata.yaml").is_file() and len(files) == 1, "bag 종료 자료 누락")
    with sqlite3.connect(files[0].resolve().as_uri() + "?mode=ro", uri=True) as db:
        require(db.execute("pragma quick_check").fetchone() == ("ok",), "bag DB 손상")
        for topic in ("/rc_channels", "/rc_uplink_lq"):
            times = [r[0] for r in db.execute(
                "select timestamp from messages join topics on topic_id=topics.id "
                "where name=? and timestamp between ? and ? order by timestamp",
                (topic, start, end))]
            require(times and max(v-u for u, v in zip([start]+times, times+[end])) < SECOND,
                    f"{topic} 부하 누락 또는 1초 이상 공백")


def cpu(s):
    return list(map(int, s["stat"].splitlines()[0].split()[1:9]))


def evaluate(path):
    m = json.loads((path / "window.json").read_text())
    require(m["run"] == path.name and m["complete"] and m["gpiomon_rc"] == 0
            and m["prescale"] == 138 and m["counts"] == [307, 307], "완료·설정 확인 실패")
    require((path / "gpiomon.err").stat().st_size == 0, "gpiomon stderr 확인 필요")
    s, e = m["start"], m["end"]
    lo, hi = s["mono"], s["mono"] + 30*SECOND
    dt = (e["mono"] - lo) / SECOND
    require(29.99 <= dt <= 30.1, "CPU/IRQ 표본 시각 편차")
    if path.name.startswith("B"):
        require(abs((e["wall"]-s["wall"]) - (e["mono"]-lo)) < 50_000_000, "실시간 시계 점프")
        load_bag(path / "bag", s["wall"], s["wall"] + 30*SECOND)
    if path.name.startswith("C"):
        require(not (path/"bag").exists() and e["drive_rchar"] > s["drive_rchar"],
                "C 조건 또는 drive 읽기 활동 확인 실패")
    data = load_edges(path / "edges.log")
    require(all(not any(t > m["shutdown_ns"] + 20_000_000 for t, _ in events)
                for events in data.values()), "OE 차단 요청 20 ms 뒤에도 에지 관측")
    results = [channel(data[p], lo, hi) for p in PINS]
    left, right = [r[0] for r in results]
    keys = sorted(right)
    pairs = []
    used = set()
    paired_left = set()
    for t, w in left.items():
        j = bisect.bisect_left(keys, t)
        nearby = [v for v in keys[max(0,j-1):j+1] if abs(v-t) <= 1_000_000 and v not in used]
        if nearby:
            v = min(nearby, key=lambda x: abs(x-t))
            pairs.append((w,right[v]))
            used.add(v)
            paired_left.add(t)
    rates = []
    report = []
    for pin, (widths, bad, gaps, covered) in zip(PINS, results):
        n = len(widths)
        out = sum(not 1475 <= w <= 1525 for w in widths.values())
        rate = 100*out/n if n else math.nan
        rates.append(rate)
        report.append(f"BCM{pin} {out}/{n}({rate:.2f}%) 짝이상={bad} 간격이상={gaps}")
    r = math.nan
    if len(pairs) >= 2:
        x, y = zip(*pairs)
        if min(x) < max(x) and min(y) < max(y):
            r = statistics.correlation(x, y)
    delta = [v-u for u,v in zip(cpu(s),cpu(e))]
    total = sum(delta)
    require(total > 0, "CPU 표본 오류")
    def intr(snap):
        return int(next(l for l in snap["stat"].splitlines() if l.startswith("intr ")).split()[1])
    report.append(f"대응={len(pairs)} r={r:.3f} CPU={100*(total-delta[3]-delta[4])/total:.1f}% "
                  f"IRQ/s={(intr(e)-intr(s))/dt:.1f} hard/softIRQ CPU={100*delta[5]/total:.2f}/{100*delta[6]/total:.2f}%")
    valid = all(covered and not bad and not gaps for _,bad,gaps,covered in results)
    # At most one unmatched edge per window boundary. Interior unmatched pairs need review.
    missing = (left.keys()-paired_left) | (right.keys()-used)
    valid = valid and all(min(t-lo,hi-t)<=1_000_000 for t in missing)
    return valid, rates, " | ".join(report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    valid = {}
    for run in ORDER:
        path = args.root / run
        if not (path / "window.json").is_file():
            print(run, "미실행" if not path.exists() else "불완전 (error.txt 및 로그 확인)")
            continue
        try:
            ok, rates, report = evaluate(path)
            print(run, "유효" if ok else "기록 이상·비교 제외", report)
            if ok:
                valid[run] = rates
        except (ValueError, OSError, sqlite3.Error, KeyError, IndexError, TypeError, StopIteration) as exc:
            print(run, "불완전·비교 제외", exc)
    for i in range(1, 5):
        a, b = f"A{i}", f"B{i}"
        if a in valid and b in valid:
            print(f"{b}−{a} 이탈률 차이 CH0/CH1 (%p):",
                  " / ".join(f"{v-u:+.2f}" for u,v in zip(valid[a],valid[b])))
        else:
            print(f"{a}/{b} 판정 보류")

    for condition in "ABC":
        group = [rates for run, rates in valid.items() if run.startswith(condition)]
        if len(group) == 4:
            print(condition, "4회 이탈률 단순평균 CH0/CH1 (%):",
                  " / ".join(f"{statistics.mean(values):.3f}" for values in zip(*group)))


if __name__ == "__main__":
    main()

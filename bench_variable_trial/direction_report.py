"""Compare closed bag samples and monotonic GPIO events for one direction run."""
import json
from pathlib import Path
import sqlite3
from statistics import median
import sys

from guidance import DIRECTIONS, matches_direction

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pwm_ab'))
from analyze import load_edges, channel, require


def stable_window(states, start, end, expected):
    groups = [[]]
    for s in states:
        ns = s['readback_ns']
        if ns is None or not start <= ns <= end:
            continue
        if not (s['armed'] and s['oe_low_requested'] and s['backend'] == 'bench_variable'
                and s['target_counts'] == s['read_counts'] == list(expected)):
            groups.append([])
            continue
        if groups[-1] and ns - groups[-1][-1] > 750_000_000:
            groups.append([])
        groups[-1].append(ns)
    intervals = [(g[0],g[-1]) for g in groups if g and g[-1]-g[0] >= 1_000_000_000]
    require(intervals, f'{expected} 되읽기 1초 연속 구간 부족')
    return max(intervals, key=lambda pair: pair[1]-pair[0])


def evaluate(path):
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Int32MultiArray, String
    sys.path.insert(0, str(ROOT/'install/kaboat_control/lib/kaboat_control'))
    from supervisor import Supervisor
    run = json.loads((path/'run.json').read_text())
    require(run['complete'] and (path/'bag/metadata.yaml').is_file(), '완료된 기록 필요')
    direction = run['direction']
    action, target = DIRECTIONS[direction]
    require(direction != 'forward', '방향·복귀 회차 전용')
    cues = {c['stage']:round(c['mono']*1e9) for c in run['cues']}
    config = json.loads((ROOT/'install/kaboat_control/lib/kaboat_control/control_config.json').read_text())
    rc = Supervisor(config)
    files = list((path/'bag').glob('*.db3'))
    require(len(files) == 1, '단일 종료 bag 필요')
    with sqlite3.connect(files[0].resolve().as_uri()+'?mode=ro&immutable=1', uri=True) as db:
        require(db.execute('pragma integrity_check').fetchone() == ('ok',), 'bag 무결성 오류')
        def messages(topic, cls):
            for stamp, blob in db.execute('select timestamp,data from messages join topics on topics.id=topic_id where name=? order by timestamp', (topic,)):
                yield stamp, deserialize_message(blob, cls)
        states = [json.loads(msg.data) for _,msg in messages('/boat/motor_status', String)]
        frames = [(t,list(msg.data)) for t,msg in messages('/rc_channels', Int32MultiArray)]
    def offset(pair):
        return pair['wall'] - (pair['mono_before']+pair['mono_after'])//2
    offsets = [offset(run[key]) for key in ('recorder_start','capture_end')]
    require(abs(offsets[0]-offsets[1]) < 1_000_000, 'bag/GPIO 시계 대응 불확실')
    edges = load_edges(path/'edges.log')
    phases = [('초기 중립','중립 초기화',action,(307,307)),(action,action,'중립 복귀',target),
              ('복귀 중립','중립 복귀','SA KILL',(307,307))]
    lines, widths = [], []
    valid = True
    for label, begin, end, expected in phases:
        lo,hi = stable_window(states,cues[begin],cues[end],expected)
        window = [(t,a) for t,a in frames if lo+max(offsets) <= t <= hi+min(offsets)]
        times = [lo+max(offsets)] + [t for t,_ in window] + [hi+min(offsets)]
        require(window and max(b-a for a,b in zip(times,times[1:])) < 300_000_000,'RC 기록 공백')
        rc_ok = True
        for t,axes in window:
            rc.set_rc(axes,t/1e9)
            match = matches_direction(direction, rc.v, rc.w) if label == action else rc.sticks_neutral
            rc_ok &= (rc.rc_at == t/1e9 and rc.mode == 'MANUAL' and rc.permit is True and match)
        valid &= rc_ok
        raw = ' '.join(f'CH{i+1}={min(a[i] for _,a in window)}~{max(a[i] for _,a in window)}' for i in (0,1))
        lines.append(f'{label}: RC {"PASS" if rc_ok else "FAIL"} {raw}, 목표·되읽기 {expected}')
        medians = []
        for pin in (24,25):
            samples,bad,gaps,covered = channel(edges[pin],lo,hi)
            widths_us = [w for t,w in samples.items() if t+round(w*1000)<=hi]
            require(covered and not bad and not gaps and len(widths_us)>=45, f'BCM{pin} 펄스 기록 불완전')
            medians.append(median(widths_us))
            lines.append(f'  BCM{pin}: {len(widths_us)}펄스, 폭 중앙값 {medians[-1]:.2f} µs')
        widths.append(medians)
    tick = 139 / 28.437
    for idx,counts in enumerate(target):
        expected = (counts-307)*tick
        change = widths[1][idx]-widths[0][idx]
        residual = widths[2][idx]-widths[0][idx]
        reverse_change = widths[2][idx]-widths[1][idx]
        ok = abs(change-expected)<=tick and abs(residual)<=tick and abs(reverse_change+expected)<=tick
        valid &= ok
        lines.append(f'CH{idx} {"PASS" if ok else "FAIL"}: 방향 변화 {change:+.2f} (기대 {expected:+.2f}), 복귀 변화 {reverse_change:+.2f}, 최초 중립과 차이 {residual:+.2f} µs')
    stop = next(s for s in states if s['estop_latched'] and not s['armed'] and not s['oe_low_requested']
                and s['oe_request_ns'] >= cues['SA KILL'])
    off = stop['oe_request_ns']
    end = run['motor_alive_observation_end']['mono_before']
    require(end < stop['deadline_mono']*1e9 and end-off>=1e9,'기한 내 종료 차단 관찰시간 부족')
    for pin in (24,25):
        extra = sum(off < t <= end for t,_ in edges[pin])
        valid &= not extra
        lines.append(f'BCM{pin} 종료 차단 후 {(end-off)/1e9:.3f}초 추가 에지 {extra}개')
    lines.append('방향·중립 복귀 및 종료 차단: '+('PASS' if valid else 'FAIL'))
    return '\n'.join(lines)


def report(path):
    try:
        return evaluate(path)
    except (OSError, KeyError, ValueError, sqlite3.Error, StopIteration) as exc:
        return f'판정 보류 — 기록 또는 안정 구간 확인 불가 ({exc})'


if __name__ == '__main__':
    print(report(Path(sys.argv[1])))

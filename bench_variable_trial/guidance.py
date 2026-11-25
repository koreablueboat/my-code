"""Operator cues only. No device, command publisher, or arm/reset client."""
from dataclasses import dataclass


class Incomplete(RuntimeError):
    pass


DIRECTIONS = {'forward': ('직진', (315, 315)), 'reverse': ('후진', (304, 304)),
              'left': ('좌회전', (304, 310)), 'right': ('우회전', (310, 304))}


def matches_direction(direction, v, w):
    if direction == 'reverse':
        return v <= -.0087 and w == 0.
    if direction == 'left':
        return v == 0. and w >= .058
    if direction == 'right':
        return v == 0. and w <= -.058
    return v >= .029 and w == 0.


START_RESERVE = 10.0  # time for the existing arming, initialization, forward and KILL stages


def awaiting_status(now, launched, discovered):
    remaining = 20.0 - (now - launched)
    stage = 'DDS 발행자 발견·첫 상태 대기' if discovered else '모터 기동·DDS 발견 대기'
    if remaining <= START_RESERVE:
        raise Incomplete(f'{stage} 시간 부족: 후속 단계에 {START_RESERVE:g}초 필요')
    return remaining, stage


def check_operator_input(phase, rc):
    if phase in ('모터 기동', '중앙·KILL 유지') and rc.permit is not False:
        raise Incomplete('SA 허용 안내 전 CH5 이탈')
    if phase in ('모터 기동', '중앙·KILL 유지', 'SA 허용', '재무장 확인', '중립 초기화') and not rc.sticks_neutral:
        raise Incomplete('방향 조작 안내 전 스틱 중앙 이탈')


@dataclass
class Guide:
    phase: str = '중앙·KILL 유지'
    since: float | None = None
    last_read: float | None = None
    direction: str = "forward"

    def advance(self, now, motor, control, rc, direction_match):
        remaining = motor['deadline_mono'] - now
        counts = motor['read_counts']
        enabled = motor['oe_low_requested'] and motor['armed']
        neutral = counts == [307, 307]
        stamp = motor['oe_request_ns'] / 1e9
        read = (motor['readback_ns'] or 0) / 1e9
        phase = self.phase
        action, target = DIRECTIONS[self.direction]
        needs_return = self.direction != 'forward'
        if control.get('previous_phase') == 'RUNNING' and control.get('phase') == 'KILLED':
            if phase not in ('SA KILL', '차단 관찰') or control['reason'] != 'radio_kill':
                raise Incomplete('예정 전 차단 또는 SA 이외 정지')
        if phase == '중앙·KILL 유지':
            if rc.permit is not False or motor['armed'] or motor['oe_low_requested']:
                raise Incomplete('안내 전 SA 허용 또는 무장')
            safe = (rc.permit is False and rc.sticks_neutral and not enabled
                    and control.get('reason') == 'radio_kill' and not motor['armed']
                    and not motor['oe_low_requested'])
            if safe:
                if self.since is None:
                    self.since = now
                if now - self.since >= 1.1:
                    self.phase = 'SA 허용'
            else:
                self.since = None
        elif phase == 'SA 허용':
            if rc.permit is True:
                self.phase = '재무장 확인'
        elif phase == '재무장 확인':
            if enabled and neutral and control.get('phase') == 'RUNNING':
                self.phase = '중립 초기화'
        elif phase == '중립 초기화':
            if not (enabled and neutral and rc.sticks_neutral and rc.permit):
                raise Incomplete('중립 초기화 조건 이탈')
            if read - stamp >= 3.0:
                self.phase = action
        elif phase in (action, '중립 복귀'):
            returning = phase == '중립 복귀'
            expected = [307, 307] if returning else list(target)
            aligned = rc.sticks_neutral if returning else direction_match
            if (enabled and rc.permit and aligned and counts == expected and
                    motor.get('target_counts', counts) == expected and now-read < .75):
                if self.last_read is None or read-self.last_read > .75:
                    self.since = read
                self.last_read = read
                if read - self.since >= 1.1:
                    self.phase = '중립 복귀' if needs_return and not returning else 'SA KILL'
                    self.last_read = None
            else:
                self.last_read = None
        elif phase == 'SA KILL' and needs_return and rc.permit and not rc.sticks_neutral:
            raise Incomplete('종료 차단 전 중앙 이탈')
        elif phase == 'SA KILL':
            if (rc.permit is False and control.get('phase') == 'KILLED'
                    and control.get('reason') == 'radio_kill'
                    and not motor['armed'] and not motor['oe_low_requested']):
                self.phase = '차단 관찰'
        elif phase == '차단 관찰':
            if rc.permit is not False or motor['oe_low_requested'] or motor['armed']:
                raise Incomplete('차단 관찰 중 허용 또는 출력 재개')
            if now-stamp >= 1.4 and remaining > .15:
                return True
        # Conservative time budget for remaining operations, including a reaction margin.
        reserve = {'중앙·KILL 유지':START_RESERVE, 'SA 허용':9., '재무장 확인':7.5,
                   '중립 초기화':max(0., 3.-(now-stamp))+4.5,
                   action:3.8, '중립 복귀':3.8, 'SA KILL':2., '차단 관찰':max(0., 1.4-(now-stamp))+.15}
        extra = 2. if needs_return and self.phase not in ('중립 복귀', 'SA KILL', '차단 관찰') else 0.
        if remaining <= reserve[self.phase] + extra:
            raise Incomplete(f'{self.phase} 단계에서 남은 시간 {remaining:.1f}초 부족')
        return False


CUES = {'중앙·KILL 유지':'SA KILL, 양쪽 스틱 중앙 유지',
        'SA 허용':'지금 SA 허용, 양쪽 스틱 중앙 유지',
        '재무장 확인':'스틱 중앙 유지, 무장 확인 대기',
        '중립 초기화':'스틱 중앙 유지, 초기화 대기',
        '직진':'조향 중앙, 스로틀을 전진 끝까지 올려 유지',
        '후진':'조향 중앙, 스로틀 후진 끝까지 유지',
        '좌회전':'스로틀 중앙, 조향 왼쪽 끝까지 유지',
        '우회전':'스로틀 중앙, 조향 오른쪽 끝까지 유지',
        '중립 복귀':'지금 두 스틱 중앙 복귀 후 유지',
        'SA KILL':'직진 상태에서 지금 SA KILL',
        '차단 관찰':'SA KILL 유지'}

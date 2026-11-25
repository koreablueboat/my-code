"""Bidirectional logical thruster commands with per-side neutral dwell."""
import math

LIMIT = 0.15  # normalized absolute command, not % measured speed or force
REVERSE_LIMIT = 0.04  # smaller absolute maximum for ANY negative propeller command
SLEW = 0.30  # normalized output units / second
TIMEOUT = 0.50  # motor command reception watchdog
NEUTRAL_DWELL = 0.40  # seconds each side must stay at neutral across a sign reversal
EPS = 1e-8


def sign(value):
    return (value > EPS) - (value < -EPS)


def mix(v, w, mode='bidirectional', reverse_limit=REVERSE_LIMIT):
    """Use existing mixer conventions: x forward, z positive yaw left.

    Reverse cap is imposed AFTER mixing to also cover reverse side of pivot turns.
    """
    if mode != 'bidirectional':
        raise ValueError('bidirectional mode required')
    if not all(math.isfinite(q) for q in (v, w)):
        raise ValueError('non-finite command')
    if not -0.009000001 <= v <= .030000001 or abs(w) > .060000001:
        raise ValueError('command out of verified PC v2 envelope')
    surge = v / .30
    yaw = w / .60
    left, right = surge-yaw, surge+yaw
    scale = max(1., max(abs(left), abs(right))/LIMIT)
    left, right = left/scale, right/scale
    # Symmetric counter-thrust for zero-surge pivot despite reverse cap.
    if abs(v) <= EPS and abs(w) > EPS:
        left = max(-reverse_limit, min(reverse_limit, left))
        right = max(-reverse_limit, min(reverse_limit, right))
    return tuple(max(-reverse_limit, min(LIMIT, q)) for q in (left, right))


class Controller:
    def __init__(self, mode='bidirectional', *, reverse_limit=REVERSE_LIMIT, neutral_dwell=NEUTRAL_DWELL):
        if mode != 'bidirectional':
            raise ValueError('bidirectional only')
        self.reverse_limit = reverse_limit
        self.neutral_dwell = neutral_dwell
        self.mode = mode
        self.armed = False
        self.estop = False
        self.estop_input = False
        self.target = (0., 0.)
        self.output = (0., 0.)
        self.command_at = None
        self.reason = 'disarmed'
        self.last_driven_sign = [0, 0]
        self.neutral_since = [None, None]

    def disarm(self, reason='disarmed'):
        self.armed = False
        self.target = self.output = (0., 0.)
        self.command_at = None
        self.reason = reason
        self.last_driven_sign = [0, 0]
        self.neutral_since = [None, None]

    def emergency(self, value):
        self.estop_input = bool(value)
        if value:
            self.estop = True
            self.disarm('estop_latched')

    def reset(self):
        if self.estop_input or self.armed:
            return False
        self.estop = False
        self.disarm()
        return True

    def command(self, v, w, now):
        if self.armed and now - self.command_at > TIMEOUT:
            self.disarm('command_timeout_rearm_required')
            return
        try:
            self.target = mix(v, w, self.mode, self.reverse_limit)
        except (ValueError, OverflowError):
            self.disarm('invalid_command_rearm_required')
            return
        self.command_at = now

    def arm(self, now):
        if self.estop:
            return False, 'release estop and reset first'
        if self.command_at is None or now-self.command_at > TIMEOUT or self.target != (0., 0.):
            return False, 'fresh zero command required before arm'
        self.armed = True
        self.output = self.target = (0., 0.)
        self.command_at = now
        self.reason = 'armed_neutral'
        self.last_driven_sign = [0, 0]
        self.neutral_since = [None, None]
        return True, 'armed; waiting for new command'

    def step(self, now, dt):
        if not self.armed or self.estop:
            self.output = (0., 0.)
            return self.output
        if self.command_at is None or now-self.command_at > TIMEOUT:
            self.disarm('command_timeout_rearm_required')
            return self.output
        next_output = []
        reversal_wait = False
        delta = SLEW * min(.10, max(0., dt))
        for idx, (out, goal) in enumerate(zip(self.output, self.target)):
            desired_sign = sign(goal)
            if desired_sign == 0:
                # Zero bypasses slew (for stops / soft KILL / AUTO STOP).
                if sign(out):
                    self.neutral_since[idx] = now
                elif self.neutral_since[idx] is None and self.last_driven_sign[idx]:
                    self.neutral_since[idx] = now
                next_output.append(0.)
                continue
            if self.last_driven_sign[idx] not in (0, desired_sign):
                # Cross-direction command forces zero, then holds it for at least dwell.
                if sign(out):
                    self.neutral_since[idx] = now
                    next_output.append(0.)
                    reversal_wait = True
                    continue
                if self.neutral_since[idx] is None:
                    self.neutral_since[idx] = now
                if now-self.neutral_since[idx] < self.neutral_dwell:
                    next_output.append(0.)
                    reversal_wait = True
                    continue
            # Either same sign or the new sign waited long enough at neutral.
            self.last_driven_sign[idx] = desired_sign
            self.neutral_since[idx] = None
            next_output.append(out + max(-delta, min(delta, goal-out)))
        self.output = tuple(next_output)
        self.reason = 'neutral_dwell' if reversal_wait else 'running' if any(sign(x) for x in self.output) else 'neutral'
        return self.output

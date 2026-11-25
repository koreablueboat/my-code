"""Pure mixer and safety state; no ROS, GPIO or serial dependency."""
import math

LIMIT = 0.15  # fraction of PWM command span, NOT measured thrust or speed
TIMEOUT = 0.5
SLEW = 0.30   # normalized command change per second


def mix(v, w, mode="bidirectional"):
    """ROS x forward, z positive left. Two aft, parallel thrusters."""
    if not math.isfinite(v) or not math.isfinite(w):
        raise ValueError('nonfinite command')
    if mode not in ('forward_only','bidirectional'):raise ValueError('unknown ESC mode')
    if mode=='forward_only' and v<0:raise ValueError('reverse requested for forward-only ESC')
    surge = max(-1., min(1., v / 0.30))
    yaw = max(-1., min(1., w / 0.60))
    left, right = surge - yaw, surge + yaw
    if mode=='forward_only':left,right=max(0.,left),max(0.,right)
    scale = max(1., max(abs(left), abs(right)) / LIMIT)
    return left / scale, right / scale



class Controller:
    def __init__(self, mode="bidirectional"):
        self.mode = mode
        self.armed = False
        self.estop = False
        self.estop_input = False
        self.target = (0., 0.)
        self.output = (0., 0.)
        self.command_at = None
        self.reason = 'disarmed'

    def disarm(self, reason='disarmed'):
        self.armed = False
        self.output = self.target = (0., 0.)
        self.command_at = None
        self.reason = reason

    def emergency(self, value):
        self.estop_input = value
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
            self.target = mix(v, w, self.mode)
        except ValueError:
            self.disarm('invalid_command')
            return
        self.command_at = now

    def arm(self, now):
        if self.estop:
            return False, 'clear estop input and call reset first'
        if self.command_at is None or now-self.command_at > TIMEOUT or self.target != (0., 0.):
            return False, 'send fresh zero Twist before arming'
        self.armed = True
        self.output = (0., 0.)
        self.target = (0., 0.)
        # Only commands arriving after arm can move the boat.
        self.command_at = now
        self.reason = 'armed_neutral'
        return True, 'armed; waiting for new command'

    def step(self, now, dt):
        if not self.armed or self.estop:
            self.output = (0., 0.)
            return self.output
        if self.command_at is None or now-self.command_at > TIMEOUT:
            self.disarm('command_timeout_rearm_required')
            return self.output
        if self.target == (0., 0.):
            self.output = (0., 0.)  # stop bypasses ramp
            self.reason = 'neutral'
            return self.output
        out = []
        delta = SLEW * min(max(dt, 0.), 0.10)
        for current, goal in zip(self.output, self.target):
            # Go through neutral before reversal; hardware calibration still required.
            if current * goal < 0:
                goal = 0.
            out.append(current + max(-delta, min(delta, goal-current)))
        self.output = tuple(out)
        self.reason = 'running'
        return self.output

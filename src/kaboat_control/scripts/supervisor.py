"""Pure control supervisor; does not output PWM or change the motor mixer.

CH5 LOW = KILL, HIGH = permit. LOW + neutral + stable active mode for 1s,
then a deliberate HIGH edge starts reset/neutral/arm handshake. A fault or
mode change requires this sequence again. AUTO always requires live RC.
"""
import math

RC_TIMEOUT = 0.30
LQ_TIMEOUT = 1.0
MOTOR_TIMEOUT = 1.5  # existing motor_status is published every 0.5s
AUTO_TIMEOUT = 0.30
SAFE_HOLD = 1.0
ZERO_HOLD = 0.60
SERVICE_TIMEOUT = 2.0
ACTIVE_PHASES = ('RELEASE', 'RESET', 'WAIT_RESET', 'ZERO', 'ARM', 'WAIT_ARM', 'RUNNING')


def validate_config(c):
    c = dict(c)
    names = ('steering_channel', 'throttle_channel', 'permit_channel', 'mode_channel')
    channels = [c[n] for n in names]
    if any(type(n) is not int or not 1 <= n <= 16 for n in channels) or len(set(channels)) != 4:
        raise ValueError('four distinct channel numbers in 1..16 required')
    if c['permit_channel'] != 5:
        raise ValueError('v2 uses CH5/AUX1 as KILL/permit')
    for name in ('mapping_verified', 'throttle_reverse', 'steering_reverse'):
        if type(c[name]) is not bool:
            raise ValueError(name + ' must be boolean')
    for n in ('raw_low', 'raw_center', 'raw_high', 'deadband', 'max_linear', 'max_angular'):
        if type(c[n]) not in (int, float) or not math.isfinite(c[n]):
            raise ValueError('invalid numeric setting: ' + n)
    if not 100 <= c['raw_low'] < c['raw_center'] < c['raw_high'] <= 1900:
        raise ValueError('invalid calibrated channel endpoints')
    if c['throttle_idle'] != 'center':
        raise ValueError('reverse v2 requires center-neutral throttle; field mapping is not verified')
    if not 0 < c['deadband'] <= .20 or not 0 < c['max_linear'] <= .03 or not 0 < c['max_angular'] <= .06:
        raise ValueError('v2 command limits: linear <= .03, angular <= .06')
    r = c.get('max_reverse_linear')
    if type(r) not in (int, float) or not math.isfinite(r) or not 0 < r <= .009 or r > c['max_linear']:
        raise ValueError('reverse v2 requires max_reverse_linear in (0,0.009] and <= max_linear')
    return c


class Supervisor:
    def __init__(self, config, expected_backend='preview'):
        self.c = validate_config(config)
        if expected_backend not in ('preview', 'pca9685', 'bench_neutral', 'bench_variable', 'left_t200_water', 'manual_dual'):
            raise ValueError('unknown motor backend')
        self.backend = expected_backend
        self.phase, self.reason = 'KILLED', 'startup_rearm_required'
        self.mode, self.selected_mode = 'STANDBY', 'STANDBY'
        self.rc_at = self.lq_at = self.motor_at = self.auto_at = None
        self.rc_seq = self.motor_seq = 0
        self.run_rc_seq = 0
        self.lq = 0
        self.motor = {}
        self.v = self.w = 0.
        self.sticks_neutral = False
        self.auto = (0., 0.)
        self.permit = None
        self.safe_since = None
        self.ready_to_arm = False
        self.external_kill = False
        self.epoch = 0
        self.phase_at = 0.
        self.pending = None
        self.confirm_seq = 0
        self.graph_ok = True
        self.auto_graph_ok = False

    def stop(self, reason):
        self.phase, self.reason = 'KILLED', reason
        self.safe_since = None
        self.ready_to_arm = False
        self.pending = None
        self.epoch += 1  # invalidate late service replies

    def set_external_kill(self, value):
        self.external_kill = bool(value)
        if value:
            self.stop('external_kill')

    def set_graph(self, okay, auto_okay):
        self.graph_ok, self.auto_graph_ok = bool(okay), bool(auto_okay)
        if not okay:
            self.stop('publisher_or_service_conflict')

    def set_motor(self, state, now):
        keys = ('armed', 'estop_latched', 'backend_ready')
        if not isinstance(state, dict) or any(type(state.get(k)) is not bool for k in keys):
            self.stop('invalid_motor_status')
            return
        self.motor, self.motor_at = dict(state), now
        self.motor_seq += 1

    def set_lq(self, value, now):
        self.lq, self.lq_at = value, now
        if type(value) is not int or not 1 <= value <= 100:
            self.stop('rf_link_lost_or_invalid')

    def set_auto(self, v, w, now):
        if not all(math.isfinite(x) for x in (v, w)) or not 0 <= v <= self.c['max_linear'] or abs(w) > self.c['max_angular']:
            self.auto_at = None
            if self.mode == 'AUTO':
                self.stop('invalid_auto_command')
            return
        self.auto, self.auto_at = (v, w), now

    def _axis(self, raw):
        c = self.c
        span = c['raw_high'] - c['raw_center'] if raw >= c['raw_center'] else c['raw_center'] - c['raw_low']
        return max(-1., min(1., (raw-c['raw_center']) / span))

    def _deadband(self, value):
        d = self.c['deadband']
        return 0. if abs(value) <= d else math.copysign((abs(value)-d)/(1-d), value)

    def neutral(self):
        return self.sticks_neutral

    def health(self, now):
        if not self.c['mapping_verified']:
            return 'channel_mapping_unverified'
        if not self.graph_ok:
            return 'publisher_or_service_conflict'
        if self.external_kill:
            return 'external_kill'
        if self.rc_at is None or now-self.rc_at > RC_TIMEOUT:
            return 'rc_timeout'
        if self.lq_at is None or now-self.lq_at > LQ_TIMEOUT:
            return 'rf_link_stale_or_lost'
        if type(self.lq) is not int or not 1 <= self.lq <= 100:
            # Preserve the distinct invalid-LQ fault across later timer ticks.
            return 'rf_link_lost_or_invalid'
        if self.motor_at is None or now-self.motor_at > MOTOR_TIMEOUT:
            return 'motor_status_timeout'
        if self.motor.get('backend') != self.backend:
            return 'motor_backend_mismatch'
        if not self.motor.get('backend_ready') or self.motor.get('esc_mode') != 'bidirectional':
            return 'motor_not_ready_or_wrong_profile'
        return None

    def auto_health(self, now):
        if self.mode != 'AUTO':
            return None
        if self.backend in ('bench_variable', 'left_t200_water', 'manual_dual'):
            return 'variable_bench_manual_only'
        if not self.auto_graph_ok:
            return 'auto_publisher_required'
        if self.auto_at is None or now-self.auto_at > AUTO_TIMEOUT:
            return 'auto_command_timeout'
        return None

    def set_rc(self, values, now):
        c = self.c
        if len(values) != 16 or any(type(v) is not int or not 0 <= v <= 2047 for v in values):
            self.stop('invalid_channels')
            return
        used = [values[c[n]-1] for n in ('steering_channel', 'throttle_channel', 'permit_channel', 'mode_channel')]
        if any(not 100 <= x <= 1900 for x in used):
            self.stop('control_channel_out_of_range')
            return
        steer, throttle, permit_raw, mode_raw = used
        steer = self._axis(steer) * (-1 if c['steering_reverse'] else 1)
        if c['throttle_idle'] == 'low':
            throttle = max(0., min(1., (throttle-c['raw_low'])/(c['raw_high']-c['raw_low'])))
            if c['throttle_reverse']:
                throttle = 1-throttle
        else:
            throttle = self._axis(throttle) * (-1 if c['throttle_reverse'] else 1)
        self.sticks_neutral = abs(throttle) <= c['deadband'] and abs(steer) <= c['deadband']
        # v2: center-neutral bidirectional RC. Never treat reverse as a safe arming neutral.
        # Negative command is capped independently from the forward speed command.
        throttle = self._deadband(throttle)
        self.v = throttle * (c['max_reverse_linear'] if throttle < 0 else c['max_linear'])
        self.w = -self._deadband(steer)*c['max_angular']  # stick right -> ROS yaw negative
        p = self._axis(permit_raw)
        permit = False if p < -.5 else True if p > .5 else None
        m = self._axis(mode_raw)
        mode = 'STANDBY' if m < -.6 else 'MANUAL' if abs(m) < .25 else 'AUTO' if m > .6 else 'INVALID'
        previous_permit, previous_mode = self.permit, self.mode
        self.permit, self.mode = permit, mode
        self.rc_at, self.rc_seq = now, self.rc_seq+1
        if mode != previous_mode:
            self.auto_at = None  # never reuse a previous mode's autonomous command
            self.stop('mode_changed_rearm_required')
        if permit is None or mode == 'INVALID':
            self.stop('invalid_switch_position')
            return
        if not permit:
            if previous_permit is not False or self.phase != 'KILLED':
                self.stop('radio_kill')
            else:
                self.reason = 'radio_kill'
            if not self.neutral():
                self.safe_since, self.ready_to_arm = None, False
            return
        if mode == 'STANDBY':
            self.stop('standby')
            return
        if previous_permit is False:
            ready = self.ready_to_arm and self.safe_since is not None and now-self.safe_since >= SAFE_HOLD
            error = self.health(now) or self.auto_health(now)
            if not ready or error or not self.neutral() or (mode == 'AUTO' and self.auto != (0., 0.)):
                self.stop(error or 'neutral_low_hold_then_high_required')
            else:
                self.selected_mode = mode
                self.phase, self.phase_at, self.reason = 'RELEASE', now, 'releasing_estop_input'
                self.safe_since, self.ready_to_arm = None, False

    def result(self, action=None):
        run = self.phase == 'RUNNING'
        cmd = self.auto if run and self.selected_mode == 'AUTO' else (self.v, self.w) if run else (0., 0.)
        # Require another live RC frame after arm confirmation, and a new AUTO command.
        if run and (self.rc_seq <= self.run_rc_seq or (self.mode == 'AUTO' and (self.auto_at is None or self.auto_at <= self.phase_at))):
            cmd = (0., 0.)
        return {'phase': self.phase, 'reason': self.reason,
                'estop': self.phase not in ACTIVE_PHASES,
                'mode': self.selected_mode if run else 'E_STOP' if self.phase == 'KILLED' and self.reason != 'standby' else 'STANDBY',
                'selected_mode': self.mode, 'ready_to_arm': self.ready_to_arm,
                'v': cmd[0], 'w': cmd[1], 'action': action, 'epoch': self.epoch}

    def tick(self, now):
        error = self.health(now)
        if error:
            self.stop(error)
            return self.result()
        if self.permit is False:
            if self.phase != 'KILLED':
                self.stop('radio_kill')
            safe = self.mode in ('MANUAL', 'AUTO') and self.neutral() and not self.motor['armed']
            safe = safe and not self.auto_health(now) and (self.mode != 'AUTO' or self.auto == (0., 0.))
            if safe:
                if self.safe_since is None:
                    self.safe_since = now
                self.ready_to_arm = now-self.safe_since >= SAFE_HOLD
            else:
                self.safe_since, self.ready_to_arm = None, False
            return self.result()
        if self.phase not in ACTIVE_PHASES:
            return self.result()
        error = self.auto_health(now)
        if error or self.mode != self.selected_mode or self.permit is not True:
            self.stop(error or 'mode_or_permit_changed')
            return self.result()
        if self.phase != 'RUNNING' and (not self.neutral() or (self.mode == 'AUTO' and self.auto != (0., 0.))):
            self.stop('nonzero_during_arming')
            return self.result()
        if self.phase in ('RESET', 'WAIT_RESET', 'ARM', 'WAIT_ARM') and now-self.phase_at > SERVICE_TIMEOUT:
            self.stop('service_or_confirmation_timeout')
            return self.result()
        action = None
        if self.phase == 'RELEASE' and now-self.phase_at >= .25:
            # Allow estop=false to reach the existing motor before reset service.
            self.phase, self.phase_at = 'RESET', now
            self.pending = ('reset', self.epoch)
            action = 'reset'
        elif self.phase == 'WAIT_RESET' and self.motor_seq > self.confirm_seq:
            if not self.motor['armed'] and not self.motor['estop_latched']:
                self.phase, self.phase_at, self.reason = 'ZERO', now, 'fresh_zero_before_arm'
        elif self.phase == 'ZERO':
            if self.motor['estop_latched'] or self.motor['armed']:
                self.stop('unexpected_motor_state')
            elif now-self.phase_at >= ZERO_HOLD:
                self.phase, self.phase_at = 'ARM', now
                self.pending = ('arm', self.epoch)
                action = 'arm'
        elif self.phase == 'WAIT_ARM' and self.motor_seq > self.confirm_seq:
            if self.motor['armed'] and not self.motor['estop_latched']:
                self.phase, self.phase_at, self.reason = 'RUNNING', now, 'running'
                self.run_rc_seq = self.rc_seq
        elif self.phase == 'RUNNING' and (not self.motor['armed'] or self.motor['estop_latched']):
            self.stop('motor_disarmed_rearm_required')
        return self.result(action)

    def service_result(self, kind, epoch, success, now):
        if self.pending != (kind, epoch) or epoch != self.epoch:
            return  # late success may never restart the supervisor
        self.pending = None
        if not success:
            self.stop(kind+'_service_failed')
        else:
            self.phase = 'WAIT_RESET' if kind == 'reset' else 'WAIT_ARM'
            self.phase_at = now
            self.confirm_seq = self.motor_seq


"""Two-channel Basic ESC R3 mapping for the scoped manual-only backend."""
import math
import time
from bench_pca import _RegisterOutput, _check_register_profile, NEUTRAL_COUNTS
from esc_config import validate
from reverse_motor_core import LIMIT

# Bounds of this low-output implementation; not a measured thrust envelope.
LOW, HIGH = 298, 319
COUNT_RATE = 24.0  # counts/second, ~117 us/s on the calibrated board


def validate_dual(p, hardware=False):
    if not isinstance(p, dict) or not all(isinstance(p.get(k), dict) for k in ('left','right','count_mapping')):
        raise ValueError('expected channel and count_mapping objects')
    validate(p, hardware=hardware)
    if p.get('trial_scope') != 'manual_dual':
        raise ValueError('manual_dual installation scope required')
    _check_register_profile(p, allow_inversion=True)
    for side in ('left', 'right'):
        curve = p.get('count_mapping', {}).get(side, {})
        keys = ('reverse_limit', 'reverse_start', 'forward_start', 'forward_limit')
        if not isinstance(curve, dict) or not all(type(curve.get(k)) is int for k in keys):
            raise ValueError(f'{side}: integer count endpoints required')
        rev, rev_start, fwd_start, fwd = (curve[k] for k in keys)
        # 301 = 1471.3 us, 313 = 1530.0 us, outside the nominal deadband.
        if not LOW <= rev < rev_start <= 301 or not 313 <= fwd_start < fwd <= HIGH:
            raise ValueError(f'{side}: invalid calibrated count envelope')
    if hardware:
        if p.get('dual_mapping_verified') is not True:
            raise ValueError('actual dual ESC mapping is unverified')
        for key in ('propulsion_disconnect_model', 'approval_record'):
            if not isinstance(p.get(key), str) or not p[key].strip():
                raise ValueError(f'missing {key} for the installed dual system')
    return p


class ManualDualOutput(_RegisterOutput):
    active_channels = (0, 1)
    min_count, max_count = LOW, HIGH
    allow_inversion = True

    def __init__(self, oe, bus, profile):
        super().__init__(oe, bus, validate_dual(profile, hardware=True))

    def enable_neutral(self, permit):
        super().enable_neutral(permit)
        self.position = [float(NEUTRAL_COUNTS)] * 2
        self.last_step = time.monotonic()

    def counts_for(self, left, right):
        now = time.monotonic()
        delta = COUNT_RATE * max(0., min(.1, now - self.last_step))
        self.last_step = now
        for i, (side, u) in enumerate((('left', left), ('right', right))):
            limit = self.profile['reverse_limit'] if u < 0 else LIMIT
            if not math.isfinite(u) or abs(u) > limit + 1e-9:
                raise ValueError('command outside manual mixer limits')
            if u == 0:
                self.position[i] = NEUTRAL_COUNTS
                continue
            curve = self.profile['count_mapping'][side]
            direction = 'forward' if u * self.profile[side]['sign'] > 0 else 'reverse'
            start, end = curve[direction+'_start'], curve[direction+'_limit']
            target = start + (end-start)*min(1., abs(u)/limit)
            self.position[i] += max(-delta, min(delta, target-self.position[i]))
        return tuple(round(pos) for pos in self.position)

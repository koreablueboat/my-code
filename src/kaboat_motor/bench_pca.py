"""Shared PCA9685 bench setup and output cutoff."""
from enum import Enum
import time

ADDRESS = 0x40
MODE1, MODE2, LED0, PRESCALE = 0, 1, 6, 0xFE
SLEEP, AUTO_INCREMENT = 0x10, 0x20
FULL_OFF = [0, 0, 0, 0x10]
# Replacement board: 4096 * (0x79 + 1) / 17.5726 ms from the first capture.
BENCH_CLOCK_HZ = 28_437_000
BENCH_PRESCALE = round(BENCH_CLOCK_HZ / (4096 * 50)) - 1
NEUTRAL_COUNTS = 307
NEUTRAL = [0, 0, NEUTRAL_COUNTS & 0xff, NEUTRAL_COUNTS >> 8]


class Side(Enum):
    LEFT = (0, '왼쪽', 'CH1~15')
    RIGHT = (1, '오른쪽', 'CH0, CH2~15')

    def __init__(self, channel, label, inactive):
        self.channel = channel
        self.label = label
        self.inactive = inactive


def prepare(bus, neutral_channels=(0, 1)):
    if bus.read_byte_data(ADDRESS, MODE1) & 0x40:
        raise RuntimeError("External oscillator selected")
    bus.write_byte_data(ADDRESS, MODE1, AUTO_INCREMENT | SLEEP)
    bus.write_byte_data(ADDRESS, PRESCALE, BENCH_PRESCALE)
    bus.write_byte_data(ADDRESS, MODE2, 4)  # non-inverted, OE HIGH drives low
    for ch in range(16):
        bus.write_i2c_block_data(ADDRESS, LED0 + 4 * ch, FULL_OFF)
    for ch in neutral_channels:
        bus.write_i2c_block_data(ADDRESS, LED0 + 4 * ch, NEUTRAL)
    bus.write_byte_data(ADDRESS, MODE1, AUTO_INCREMENT)
    time.sleep(.001)
    prescale = bus.read_byte_data(ADDRESS, PRESCALE)
    if prescale != BENCH_PRESCALE:
        raise RuntimeError("PRESCALE readback failed")
    for reg, value in ((MODE1, AUTO_INCREMENT), (MODE2, 4)):
        if bus.read_byte_data(ADDRESS, reg) != value:
            raise RuntimeError(f"Configuration readback failed at register {reg:#x}")
    counts = []
    for ch in range(16):
        expected = NEUTRAL if ch in neutral_channels else FULL_OFF
        registers = bus.read_i2c_block_data(ADDRESS, LED0 + 4 * ch, 4)
        if registers != expected:
            raise RuntimeError(f"CH{ch} readback failed")
        if ch in neutral_channels:
            counts.append(registers[2] | (registers[3] << 8))  # verified ON = 0
    return prescale, counts


def request_oe(stack, gpiod, consumer):
    chip = stack.enter_context(gpiod.Chip('gpiochip0'))
    if chip.label() != 'pinctrl-bcm2711':
        raise RuntimeError(f"Unexpected gpiochip0 controller: {chip.label()}")
    oe = chip.get_line(17)
    oe.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT, default_val=1)
    stack.callback(oe.release)
    return chip, oe


def shutdown(oe, bus, after_disable=None):
    try:
        oe.set_value(1)
        stop = time.monotonic_ns()
        if after_disable is not None:
            after_disable(stop)
        return stop
    finally:
        # Single-register writes also work if setup failed before AI was set.
        try:
            bus.write_byte_data(ADDRESS, 0xFD, 0x10)  # ALL_LED_OFF_H
        finally:
            bus.write_byte_data(ADDRESS, MODE1, AUTO_INCREMENT | SLEEP)


class FixedNeutralGate:
    """The only operation that lowers OE prepares and verifies both neutral channels."""
    def __init__(self, oe, bus):
        self.oe = oe
        self.bus = bus
        self.enabled = False

    def enable_neutral(self, permit):
        self.disable()
        prepare(self.bus)
        if not permit():
            raise RuntimeError("Bench enable expired during register verification")
        self.oe.set_value(0)
        self.enabled = True

    def disable(self):
        self.oe.set_value(1)
        self.enabled = False


def _check_register_profile(profile, allow_inversion=False):
    if (profile['reference_clock_hz'], profile['frequency_hz'], profile['i2c_address'],
            profile['oe_gpio_bcm'], profile['mode'], profile['neutral_init_seconds']) != (
                BENCH_CLOCK_HZ, 50, ADDRESS, 17, 'bidirectional', 3):
        raise ValueError('unexpected calibrated PCA profile')
    for index, side in enumerate(('left', 'right')):
        polarity = profile[side]['sign'] if allow_inversion else 1
        if profile[side] != dict(channel=index, sign=polarity, neutral_us=1500,
                                 forward_us=1900, reverse_us=1100):
            raise ValueError('unexpected calibrated PCA channel configuration')


class _RegisterOutput:
    """The motor node owns the bus and OE through its ExitStack."""
    allow_inversion = False

    def __init__(self, oe, bus, profile):
        _check_register_profile(profile, allow_inversion=self.allow_inversion)
        self.oe, self.bus, self.profile = oe, bus, profile
        self.enabled = self.failed = False
        self.ready_at = float('inf')
        self.target_counts = self.read_counts = None
        self.read_ns = self.oe_ns = None
        self.frequency = BENCH_CLOCK_HZ / (4096 * (BENCH_PRESCALE + 1))
        self.disable()

    def disable(self):
        self.oe.set_value(1)
        if self.enabled or self.oe_ns is None:
            self.oe_ns = time.monotonic_ns()
        self.enabled = False
        self.ready_at = float('inf')

    close = disable

    def enable_neutral(self, permit):
        self.disable()
        if self.failed:
            raise RuntimeError('variable bench fault latched')
        try:
            _, counts = prepare(self.bus, self.active_channels)
            self.read_counts = self.target_counts = tuple(counts)
            self.read_ns = time.monotonic_ns()
            if not permit():
                raise RuntimeError('Bench enable expired during register verification')
            self.oe.set_value(0)
            self.enabled = True
            self.oe_ns = time.monotonic_ns()
            self.ready_at = time.monotonic() + self.profile['neutral_init_seconds']
        except Exception:
            self.failed = True
            self.disable()
            raise

    def poll(self, now):
        return self.enabled and not self.failed and now >= self.ready_at

    def counts_for(self, left, right):
        from esc_config import duty_from_us, pulse_us
        return tuple(duty_from_us(pulse_us(value, self.profile, side), self.frequency) >> 4
                     for side, value in self.outputs(left, right))

    def write(self, left, right):
        try:
            if not self.enabled or self.failed:
                raise RuntimeError('variable bench output disabled')
            if not self.poll(time.monotonic()) and (left or right):
                raise RuntimeError('non-neutral output during initialization')
            counts = self.counts_for(left, right)
            if not all(self.min_count <= count <= self.max_count for count in counts):
                raise ValueError('variable bench count outside envelope')
            self.target_counts = counts
            observed = []
            for index, count in enumerate(counts):
                reg = LED0 + 4 * index
                data = [0, 0, count & 0xff, count >> 8]
                if count != self.read_counts[index]:
                    self.bus.write_i2c_block_data(ADDRESS, reg, data)
                raw = self.bus.read_i2c_block_data(ADDRESS, reg, 4)
                if raw != data:
                    raise RuntimeError(f'CH{index} variable PWM readback failed')
                observed.append(raw[2] | (raw[3] << 8))
            if self.active_channels == (0,) and self.bus.read_i2c_block_data(ADDRESS, LED0+4, 4) != FULL_OFF:
                raise RuntimeError('inactive CH1 not FULL OFF')
            if self.bus.read_byte_data(ADDRESS, PRESCALE) != BENCH_PRESCALE:
                raise RuntimeError('variable PWM prescale readback failed')
            self.read_counts = tuple(observed)
            self.read_ns = time.monotonic_ns()
        except Exception:
            self.failed = True
            self.disable()
            raise


class VariableBenchOutput(_RegisterOutput):
    """Disconnected bench only."""
    active_channels = (0, 1)
    min_count, max_count = 304, 319

    @staticmethod
    def outputs(left, right):
        return (('left', left), ('right', right))


def validate_left_water(profile):
    from esc_config import validate
    validate(profile, hardware=True)
    if profile.get('trial_scope') != 'left_t200_water':
        raise ValueError('hardware approval must cover left_t200_water')
    for key in ('propulsion_disconnect_model', 'approval_record'):
        if not isinstance(profile.get(key), str) or not profile[key].strip():
            raise ValueError(f'missing approved installation detail: {key}')
    _check_register_profile(profile)
    return profile


class LeftWaterOutput(_RegisterOutput):
    active_channels = (0,)
    min_count, max_count = 307, 315

    def __init__(self, oe, bus, profile):
        super().__init__(oe, bus, validate_left_water(profile))

    @staticmethod
    def outputs(left, right):
        if right != 0 or not 0 <= left <= .100000001:
            raise ValueError('left water trial accepts only limited CH0 forward output')
        return (('left', left),)

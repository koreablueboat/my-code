import pytest
from unittest.mock import Mock
import neutral_probe as probe


class Bus:
    def __init__(self, log, fail_channel):
        self.registers = {probe.MODE1: 0x11}
        self.log = log
        self.fail_channel = fail_channel

    def read_byte_data(self, address, reg):
        assert address == probe.ADDRESS
        return self.registers[reg]

    def write_byte_data(self, address, reg, value):
        assert address == probe.ADDRESS
        self.registers[reg] = value
        self.log.append(('reg', reg, value))

    def write_i2c_block_data(self, address, reg, value):
        ch = (reg - probe.LED0) // 4
        self.log.append(('channel', ch, tuple(value)))
        if ch == self.fail_channel and value == probe.NEUTRAL:
            raise OSError('partial write')
        self.registers[reg] = list(value)

    def read_i2c_block_data(self, address, reg, size):
        return self.registers[reg][:size]


@pytest.mark.parametrize('fault', [None, 'left', 'right', 'ctrl-c', 'capture'])
def test_gate_order(fault, monkeypatch):
    log = [('oe', 1)]  # initial output request in hardware()
    bus = Bus(log, {'left': 0, 'right': 1}.get(fault))
    oe = Mock()
    oe.set_value.side_effect = lambda v: log.append(('oe', v))
    monkeypatch.setattr(probe.time, 'monotonic_ns', lambda: 100)
    def capture(end):
        log.append(('capture', end))
        if end == 100 + 2 * probe.SECOND:
            if fault == 'ctrl-c':
                raise KeyboardInterrupt
            if fault == 'capture':
                raise RuntimeError
    if fault:
        with pytest.raises({'ctrl-c': KeyboardInterrupt, 'capture': RuntimeError}.get(fault, OSError)):
            probe.measure(oe, bus, capture)
    else:
        probe.measure(oe, bus, capture)
    assert log[-4:] == [('oe', 1), ('capture', 100 + probe.MARGIN),
                        ('reg', 0xFD, 0x10), ('reg', probe.MODE1, 0x30)]
    if fault in ('left', 'right'):
        assert ('oe', 0) not in log
        assert log[-5][0] == 'channel'
    else:
        enable = log.index(('oe', 0))
        assert log[enable-1] == ('capture', 100 + probe.MARGIN)
        assert log[enable+1] == ('capture', 100 + 2 * probe.SECOND)
        for ch in range(16):
            assert ('channel', ch, tuple(probe.FULL_OFF)) in log[:enable]
        for ch in (0, 1):
            assert ('channel', ch, tuple(probe.NEUTRAL)) in log[:enable]
        assert ('reg', probe.MODE1, probe.AUTO_INCREMENT) in log[:enable]


def test_statistics_and_cutoff(capsys):
    us = 1000
    trace = [(t * us, r) for t in (1000, 21000, 41000, 61000)
             for t, r in ((t, True), (t+1500, False))]
    assert not probe.summarize(0, trace, 0, 40000 * us)
    out = capsys.readouterr().out
    for value in ('출력 중 2', '반복 있음', '1500.0/1500.0/1500.0', '20000.0/20000.0/20000.0'):
        assert value in out
    assert not probe.summarize(1, [], 0, 40000 * us)
    assert '측정 불충분' in capsys.readouterr().out


@pytest.mark.parametrize('bad_register', [None, probe.PRESCALE, probe.LED0, probe.LED0 + 4])
def test_calibrated_readback_before_enable(bad_register):
    bus = Bus([], None)
    if bad_register == probe.PRESCALE:
        read = bus.read_byte_data
        bus.read_byte_data = lambda addr, reg: 121 if reg == bad_register else read(addr, reg)
    elif bad_register is not None:
        read = bus.read_i2c_block_data
        bus.read_i2c_block_data = lambda addr, reg, size: (
            [0, 0, 0x32, 1] if reg == bad_register else read(addr, reg, size))
    oe = Mock()
    if bad_register is None:
        _, _, settings = probe.measure(oe, bus, Mock())
        assert settings == (0x8A, [307, 307])
    else:
        with pytest.raises(RuntimeError, match='readback'):
            probe.measure(oe, bus, Mock())
        oe.set_value.assert_called_once_with(1)


def test_boundary_widths(capsys):
    widths = [1181, 1500, 1510, 1490, 1182]
    trace = [(stamp * 1000, rising) for i, width in enumerate(widths)
             for stamp, rising in ((i * 20000, True), (i * 20000 + width, False))]
    assert probe.summarize(1, trace, 0, 81181 * 1000)
    out = capsys.readouterr().out
    assert '출력 중 5, 차단 후 200 ms 0펄스' in out
    assert '첫 1181.0, 마지막 1182.0, 내부 최소/중앙/최대 1490.0/1500.0/1510.0' in out


def test_internal_outliers_and_coincidence(capsys):
    base = 987654321000
    short = [[], []]
    for ch, widths in enumerate((
            [1181, 1475, 1438.3, 1525, 1525.001, 1500, 1181],
            [1181, 1500, 1435.7, 1500, 1500, 1474.999, 1181])):
        trace = []
        for i, width in enumerate(widths):
            rise = base + i * probe.PERIOD_NS + ch * 2345
            trace.extend([(rise, True), (rise + round(width * 1000), False)])
        assert probe.summarize(ch, trace, base, base + len(widths) * probe.PERIOD_NS, short[ch])
    probe.compare_short(short)
    out = capsys.readouterr().out
    assert 'CH0 내부 유효 5개 중 1475~1525 µs 밖 2개 | 활성 상승 기준 번호 #3, #5' in out
    assert 'CH1 내부 유효 5개 중 1475~1525 µs 밖 2개 | 활성 상승 기준 번호 #3, #6' in out
    assert '커널 상승 987694321000 ns, 하강 987695759300 ns' in out
    assert '직전/직후 상승 간격 20000.000/20000.000 µs' in out
    assert 'CH0 #3 ↔ CH1 #3, 상승 차이 CH1−CH0 +2.345 µs' in out
    assert 'CH1 #6 대응 없음' in out


def test_discontinuous_trace(capsys):
    trace = [(t * 1000, r) for t, r in [
        (100, False), (1000, True), (21000, True), (22400, False),
        (41000, True), (40999, False), (42500, False),
        (81000, True), (82500, False), (101000, True)]]
    assert not probe.summarize(0, trace, 0, 102000 * 1000)
    out = capsys.readouterr().out
    for label in ('짝 없는 하강', '연속 상승', '시각 순서 이상', '상승 간격 이상', '짝 없는 마지막 상승'):
        assert label in out
    assert '출력 중 2' in out
    assert '첫 측정 불충분, 마지막 측정 불충분' in out
    assert '활성 상승 기준 번호 #2' in out
    assert '직전/직후 상승 간격 20000.000/20000.000 µs' in out


def test_compare_by_timestamp_despite_number_offset(capsys):
    probe.compare_short([[(35, 700_000_000), (98, 1_950_000_000)], [(34, 700_010_000)]])
    out = capsys.readouterr().out
    assert 'CH0 #35 ↔ CH1 #34' in out
    assert 'CH0 #98 대응 없음' in out


@pytest.mark.parametrize('fault', [None, 'ctrl-c', 'hold', 'readback'])
@pytest.mark.parametrize('name,ch', [('left', 0), ('right', 1)])
def test_single_esc_mode(monkeypatch, fault, name, ch):
    import sys
    from unittest.mock import MagicMock

    log = []
    bus = Bus(log, None)
    bus.close = Mock()
    if fault == 'readback':
        read = bus.read_i2c_block_data
        bus.read_i2c_block_data = lambda addr, reg, size: (
            probe.NEUTRAL if reg == probe.LED0 + 4 * (1 - ch) else read(addr, reg, size))
    oe = Mock()
    oe.request.side_effect = lambda **kw: log.append(('request', kw['default_val']))
    oe.set_value.side_effect = lambda v: log.append(('oe', v))
    chip = MagicMock()
    chip.label.return_value = 'pinctrl-bcm2711'
    chip.get_line.return_value = oe
    gpio = MagicMock()
    gpio.Chip.return_value.__enter__.return_value = chip
    monkeypatch.setitem(sys.modules, 'gpiod', gpio)
    monkeypatch.setitem(sys.modules, 'smbus', Mock(SMBus=Mock(return_value=bus)))
    monkeypatch.setattr(probe.signal, 'signal', Mock())
    monkeypatch.setattr(sys, 'argv', ['neutral_probe', '--hardware', f'--{name}-esc-neutral'])

    def sleep(seconds):
        log.append(('sleep', seconds))
        if seconds == 7 and fault in ('ctrl-c', 'hold'):
            raise KeyboardInterrupt if fault == 'ctrl-c' else OSError('hold')

    monkeypatch.setattr(probe.time, 'sleep', sleep)
    if fault:
        with pytest.raises({'ctrl-c': KeyboardInterrupt, 'hold': OSError, 'readback': RuntimeError}[fault]):
            probe.main()
    else:
        assert probe.main() == 0
    chip.get_line.assert_called_once_with(17)
    assert log[0] == ('request', 1)
    assert log[-3:] == [('oe', 1), ('reg', 0xFD, 0x10), ('reg', probe.MODE1, 0x30)]
    writes = [entry[1:] for entry in log if entry[0] == 'channel']
    assert writes == [(ch, tuple(probe.FULL_OFF)) for ch in range(16)] + [(ch, (0, 0, 0x33, 1))]
    assert ('reg', probe.PRESCALE, 0x8A) in log
    if fault == 'readback':
        assert ('oe', 0) not in log
    else:
        assert log[-5:-3] == [('oe', 0), ('sleep', 7)]
    oe.release.assert_called_once()
    bus.close.assert_called_once()

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

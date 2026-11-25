from unittest.mock import Mock

import pytest

import t200_once as once
import bench_pca as pca
from test_neutral_probe import Bus


@pytest.mark.parametrize('fault', [None, 'cancel', 'interrupt', 'write', 'return', 'hold', 'prepare'])
@pytest.mark.parametrize('side', list(pca.Side))
def test_one_shot(fault, monkeypatch, side):
    log = []
    oe = Mock()
    oe.set_value.side_effect = lambda v: log.append(('oe', v))
    bus = Bus(log, side.channel if fault == 'prepare' else None)
    write = bus.write_i2c_block_data

    def record_write(address, reg, value):
        write(address, reg, value)
        if ('oe', 0) in log and fault == ('return' if value == pca.NEUTRAL else 'write'):
            raise OSError

    def confirm(fd, selected):
        assert selected is side
        log.append(('confirm', fd))
        if fault == 'interrupt':
            raise KeyboardInterrupt
        return fault != 'cancel'

    def sleep(seconds):
        log.append(('sleep', seconds))
        if seconds == 0.5 and fault == 'hold':
            raise RuntimeError

    monkeypatch.setattr(once, 'confirmed', confirm)
    monkeypatch.setattr(once.time, 'sleep', sleep)
    bus.write_i2c_block_data = record_write
    if fault in (None, 'cancel'):
        assert once.run_once(oe, bus, 9, side) == (fault is None)
    else:
        with pytest.raises({'interrupt': KeyboardInterrupt, 'hold': RuntimeError}.get(fault, OSError)):
            once.run_once(oe, bus, 9, side)
    assert log[-3:] == [('oe', 1), ('reg', 0xFD, 0x10), ('reg', pca.MODE1, 0x30)]
    if fault == 'prepare':
        assert ('oe', 0) not in log
        return
    enabled = log.index(('oe', 0))
    assert ('reg', pca.PRESCALE, 0x8A) in log[:enabled]
    channels = [e[1:] for e in log[:enabled] if e[0] == 'channel']
    assert channels == [(i, tuple(pca.FULL_OFF)) for i in range(16)] + [(side.channel, (0, 0, 51, 1))]
    active = [('confirm', 9), ('channel', side.channel, (0, 0, 59, 1)), ('sleep', 0.5),
              ('channel', side.channel, (0, 0, 51, 1)), ('sleep', 2)]
    length = {'cancel': 1, 'interrupt': 1, 'write': 2, 'hold': 3, 'return': 4}.get(fault, 5)
    assert log[enabled + 1:-3] == active[:length]


@pytest.mark.parametrize('ready,line,end,accept', [
    (True, b'RUN LEFT\n', 29, True), (True, b'RUN LEFT\n', 30, False),
    (True, b'RUN LEFT', 1, False), (True, b'RUN OTHER\n', 1, False),
    (True, b'', 1, False), (False, b'', 30, False)])
@pytest.mark.parametrize('side', list(pca.Side))
def test_fresh_timed_confirmation(ready, line, end, accept, monkeypatch, side):
    if side is pca.Side.RIGHT:
        line = line.replace(b'LEFT', b'RIGHT').replace(b'OTHER', b'LEFT')
    else:
        line = line.replace(b'OTHER', b'RIGHT')
    flush = Mock()
    monkeypatch.setattr(once.termios, 'tcflush', flush)
    monkeypatch.setattr(once.time, 'monotonic', iter([0, 0, end]).__next__)
    poll = Mock(return_value=([7] if ready else [], [], []))
    monkeypatch.setattr(once.select, 'select', poll)
    read = Mock(return_value=line)
    monkeypatch.setattr(once.os, 'read', read)
    assert once.confirmed(7, side) is accept
    flush.assert_called_once_with(7, once.termios.TCIFLUSH)
    poll.assert_called_once_with([7], [], [], 30)
    if not ready:
        read.assert_not_called()


@pytest.mark.parametrize('channel', range(16))
def test_right_readback_mismatch_keeps_gate_closed(channel, monkeypatch):
    bus = Bus([], None)
    read = bus.read_i2c_block_data
    def corrupt(address, reg, size):
        if reg == pca.LED0 + 4 * channel:
            return pca.FULL_OFF if channel == 1 else pca.NEUTRAL
        return read(address, reg, size)

    monkeypatch.setattr(bus, 'read_i2c_block_data', corrupt)
    oe = Mock()
    with pytest.raises(RuntimeError, match=f'CH{channel} readback'):
        once.run_once(oe, bus, 9, pca.Side.RIGHT)
    oe.set_value.assert_called_once_with(1)

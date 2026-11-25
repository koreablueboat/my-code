"""Exercise the installed launch sequence with child processes that only write files."""
import json
from pathlib import Path

import pytest

import manual


@pytest.mark.parametrize('motor_code', [0, 7])
def test_monitor_fake_children(tmp_path, monkeypatch, motor_code):
    stub = tmp_path/'stub'
    stub.write_text("""#!/usr/bin/python3
import os, signal, sys, time
from pathlib import Path
role = Path(sys.argv[0]).name
root = Path(os.environ['ROS_LOG_DIR'])
def stop(*args):
    if role == 'ros2':
        (root/'bag').mkdir()
        (root/'bag/metadata.yaml').touch()
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
if role == 'ros2':
    for topic in sys.argv[5:]:
        print(f"Subscribed to topic '{topic}'", flush=True)
if role == 'boat_motor':
    assert sys.argv[1:] == ['--backend','manual_dual','--profile',str(root/'profile.json'),'--duration','20']
    assert os.environ['ROS_DOMAIN_ID'] == '124'
    if int(os.environ['MOTOR_CODE']):
        sys.exit(int(os.environ['MOTOR_CODE']))
    (root/'STOP').touch()
while True:
    time.sleep(.02)
""")
    stub.chmod(0o755)
    for name in ('ros2', 'drive', 'control', 'boat_motor'):
        (tmp_path/name).symlink_to(stub)
    monkeypatch.setenv('PATH', str(tmp_path))
    monkeypatch.setenv('MOTOR_CODE', str(motor_code))
    monkeypatch.setattr(manual, 'binary', lambda pkg, exe: tmp_path/exe)
    monkeypatch.setattr(manual, 'device_check', lambda: None)
    if motor_code:
        with pytest.raises(RuntimeError, match='모터 종료코드 7'):
            manual.monitor(tmp_path, tmp_path/'profile.json', 20)
    else:
        manual.monitor(tmp_path, tmp_path/'profile.json', 20)
    result = json.loads((tmp_path/'result.json').read_text())
    assert result['ended_by'] == ('error' if motor_code else 'operator_stop')
    assert result['bag_closed'] and not result['forced_stop']
    assert result['exit_codes'] == dict(rx=0, control=0, motor=motor_code, bag=0)

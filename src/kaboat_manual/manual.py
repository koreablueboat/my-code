#!/usr/bin/python3
"""Supervise one attended manual_dual session. No arm/reset clients."""
import argparse
import ctypes
from contextlib import ExitStack
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix

ROOT = Path.home()/'.local/state/kaboat-manual'
TOPICS = '/rc_channels /rc_uplink_lq /boat/cmd_vel /boat/estop /boat/motor_status /boat/motor_preview /rosout'.split()


def binary(pkg, exe):
    return Path(get_package_prefix(pkg))/'lib'/pkg/exe


def tail(path, size=4096):
    if not path.is_file():
        return ''
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size-size))
        return stream.read().decode(errors='replace')


def device_check():
    for dev in ('/dev/ttyAMA1','/dev/i2c-1'):
        if not os.access(dev, os.R_OK | os.W_OK):
            raise ValueError(f'{dev} 접근 불가')
        usage = subprocess.run(['fuser',dev], capture_output=True)
        if usage.returncode != 1 or usage.stdout.strip() or usage.stderr.strip():
            raise ValueError(f'{dev} 점유 또는 점유 확인 실패')
    if Path('/sys/bus/i2c/devices/1-0040/driver').exists():
        raise ValueError('0x40 kernel driver busy')


def active(path):
    with (path/'lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
    return False


def inspect_session(action, path):
    running = active(path)
    state = '실행 중' if running else ('종료됨' if (path/'result.json').is_file() else '실행기 없음·정상 종료 미확인')
    print(state, path)
    if action == 'stop':
        if running:
            (path/'STOP').touch()
            until = time.monotonic()+12
            while active(path) and time.monotonic()<until:
                time.sleep(.1)
            if active(path):
                raise ValueError('종료 확인 지연. 독립 추진 전원을 차단하고 diagnose를 확인하세요.')
            print('실행기 종료 확인')
        return
    for name in ('control','motor'):
        lines = tail(path/(name+'.log')).splitlines()
        if lines:
            print(name, lines[-1])
    if action == 'diagnose':
        print(tail(path/'result.json') or '완료 기록 없음')
        for name in ('rx','control','motor','bag'):
            print(f'--- {name} ---\n{tail(path/(name+".log"), 2500)}')


def monitor(path, profile, duration):
    env = dict(os.environ, ROS_DOMAIN_ID='124', ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(path), RCUTILS_LOGGING_BUFFERED_STREAM='0')
    jobs = {}
    forced = []
    result = {'ended_by':'startup_error','duration':duration,'domain':124}
    parent_pid = os.getpid()
    libc = ctypes.CDLL(None, use_errno=True)
    def stop_on_parent_exit():
        # Linux PR_SET_PDEATHSIG. Also close the fork/parent-exit race.
        if libc.prctl(1, signal.SIGTERM, 0, 0, 0) or os.getppid() != parent_pid:
            os._exit(125)

    def spawn(name, cmd):
        with (path/(name+'.log')).open('x') as log:
            jobs[name] = subprocess.Popen(cmd,env=env,stdout=log,stderr=log,
                                          stdin=subprocess.DEVNULL,start_new_session=True,preexec_fn=stop_on_parent_exit)
    try:
        device_check()
        if shutil.disk_usage(path).free < 128*1024**2:
            raise ValueError('기록 공간 128 MiB 미만')
        spawn('bag',['ros2','bag','record','-o',path/'bag',*TOPICS])
        spawn('rx',[binary('boat_rc','drive'),'--ros-args','-p','port:=/dev/ttyAMA1','-p','baud:=420000'])
        spawn('control',[binary('kaboat_control','control'),'--ros-args','-p','expected_backend:=manual_dual'])
        deadline = time.monotonic()+25
        last_line = ''
        while not (path/'STOP').exists():
            if any(p.poll() is not None for name,p in jobs.items() if name!='motor'):
                raise RuntimeError('수신기·제어기·bag 조기 종료')
            if 'motor' not in jobs:
                ready = tail(path/'bag.log')
                if all(f"Subscribed to topic '{t}'" in ready for t in TOPICS[:4]):
                    spawn('motor',[binary('kaboat_motor','boat_motor'),'--backend','manual_dual',
                                   '--profile',profile,'--duration',str(duration)])
                elif time.monotonic() >= deadline:
                    raise RuntimeError('수신기·제어기 토픽 기록 준비 시간 초과')
            else:
                lines = tail(path/'motor.log').splitlines()
                if lines and lines[-1] != last_line:
                    print(lines[-1], flush=True)
                    last_line = lines[-1]
                code = jobs['motor'].poll()
                if code is not None:
                    if code:
                        raise RuntimeError(f'모터 종료코드 {code}')
                    result['ended_by'] = 'motor_exit'
                    break
            time.sleep(.1)
        else:
            result['ended_by'] = 'operator_stop'
    except KeyboardInterrupt:
        result['ended_by'] = 'operator_interrupt'
    except Exception as exc:
        result['error'] = str(exc)
        result['ended_by'] = 'error'
        raise
    finally:
        # Motor/OE first; keep the recorder alive through motor shutdown.
        for name,proc in reversed(list(jobs.items())):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(2 if name=='motor' else 6)
                except subprocess.TimeoutExpired:
                    print(f'{name} 종료 지연. 독립 추진 전원을 차단하세요.', flush=True)
                    forced.append(name)
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
        result['exit_codes'] = {name:p.returncode for name,p in jobs.items()}
        result['forced_stop'] = forced
        result['bag_closed'] = (path/'bag/metadata.yaml').is_file()
        (path/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        print('SA KILL 유지, 독립 차단기로 추진 전원 차단. 기록:',path)
    if forced or not result['bag_closed'] or any(v not in (0,-signal.SIGINT) for v in result['exit_codes'].values()):
        raise ValueError('비정상 종료 또는 bag 마감 오류. diagnose 확인')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    for name in ('check','start'):
        opt = sub.add_parser(name)
        opt.add_argument('--profile',type=Path,default=binary('kaboat_motor','manual_dual_candidate.json'))
        if name=='start':
            opt.add_argument('--duration',type=int,choices=range(1,21),default=20)
    for name in ('stop','status','diagnose'):
        opt = sub.add_parser(name)
        if name=='diagnose':
            opt.add_argument('--session',type=Path)
    args = parser.parse_args()
    if args.action in ('stop','status','diagnose'):
        path = getattr(args,'session',None) or ROOT/'latest'
        if not path.is_dir():
            print('저장된 세션 없음')
            return
        inspect_session(args.action,path.resolve())
        return
    sys.path.insert(0,str(binary('kaboat_motor','manual_dual').parent))
    from manual_dual import validate_dual
    settings = json.loads(args.profile.read_text())
    validate_dual(settings)
    print('양방향 PWM 설정 구조 확인', flush=True)
    validate_dual(settings, hardware=True)
    if args.action=='check':
        print('승인 프로파일 선언 확인. 실제 차단기와 배선은 현장 확인 대상.')
        return
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise ValueError('현장 터미널 필요')
    print('양쪽 T200 수중 고정, 독립 전원 차단 담당자 준비. SB MANUAL·SA KILL·스틱 중앙.')
    if input('시작하려면 DUAL MANUAL 입력: ').strip() != 'DUAL MANUAL':
        raise ValueError('취소')
    ROOT.mkdir(mode=0o700,parents=True,exist_ok=True)
    with ExitStack() as stack:
        lock = stack.enter_context((ROOT/'launcher.lock').open('a'))
        fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
        path = ROOT/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path.mkdir(mode=0o700)
        lock = stack.enter_context((path/'lock').open('a'))
        fcntl.flock(lock,fcntl.LOCK_EX)
        profile = path/'profile.json'
        profile.write_text(json.dumps(settings,indent=2))
        link = ROOT/'.latest.tmp'
        link.unlink(missing_ok=True)
        link.symlink_to(path)
        link.replace(ROOT/'latest')
        print('세션',path,flush=True)
        monitor(path,profile,args.duration)


if __name__ == '__main__':
    def interrupt(*_):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM,signal.SIGHUP):
        signal.signal(sig, interrupt)
    try:
        main()
    except (ValueError, OSError, RuntimeError, KeyError) as exc:
        sys.exit(str(exc))

#!/usr/bin/python3
"""Single-owner fixed-neutral A/B/C bench. --check never opens hardware."""
import argparse
from contextlib import ExitStack
import fcntl
import importlib.util
import inspect
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

sys.dont_write_bytecode = True
from analyze import ORDER, ROOT, evaluate, require

INSTALL = Path(__file__).resolve().parents[1] / "install"
MODULE = INSTALL / "kaboat_motor/lib/kaboat_motor/bench_pca.py"
DRIVE = INSTALL / "boat_rc/lib/boat_rc/drive"
RESERVE = 128 * 1024**2


def static_check(root):
    import gpiod
    import smbus
    spec = importlib.util.spec_from_file_location("bench_pca", MODULE)
    bench = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bench)
    for name, args in (("prepare", (None,)), ("request_oe", (None,None,"test")),
                       ("shutdown", (None,None))):
        inspect.signature(getattr(bench,name)).bind(*args)
    require(bench.BENCH_PRESCALE == 138 and bench.NEUTRAL_COUNTS == 307, "벤치 상수 불일치")
    require(DRIVE.is_file() and os.access(DRIVE, os.X_OK), "설치된 boat_rc drive 필요")
    require(shutil.which("gpiomon") and shutil.which("fuser"), "gpiomon/fuser 필요")
    version = subprocess.check_output(["gpiomon","--version"], text=True)
    require("v1." in version, "gpiomon 1.x 필요")
    require(hasattr(gpiod.Line,"is_used") and hasattr(smbus,"SMBus"), "GPIO/I²C API 불일치")
    parent = next(p for p in (root,*root.parents) if p.exists())
    require(parent.is_dir() and os.access(parent,os.W_OK | os.X_OK), "결과 경로 쓰기 불가")
    space(parent)
    return bench


def space(path):
    require(shutil.disk_usage(path).free >= RESERVE, "저장공간 여유 128 MiB 미만")


def idle_device(path):
    require(os.access(path,os.R_OK | os.W_OK), f"{path} 접근 불가")
    p = subprocess.run(["fuser",path], capture_output=True, text=True)
    require(p.returncode == 1 and not p.stderr.strip() and not p.stdout.strip(),
            f"{path} 점유 또는 조회 실패: {p.stdout} {p.stderr}")


def device_check():
    import gpiod
    idle_device("/dev/i2c-1")
    idle_device("/dev/ttyAMA1")
    require(not Path("/sys/bus/i2c/devices/1-0040/driver").exists(), "0x40 커널 드라이버 점유")
    with gpiod.Chip("gpiochip0") as chip:
        require(chip.label()=="pinctrl-bcm2711", "gpiochip0 라벨 불일치")
        for pin in (17,24,25):
            require(not chip.get_line(pin).is_used(), f"BCM{pin} 사용 중")
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            argv = path.read_bytes().split(b"\0")[:3]
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        names = {os.path.basename(v) for v in argv}
        require(not names.intersection({b"boat_motor",b"neutral_probe",b"control",b"left_t200_once",
                                        b"right_t200_once",b"drive",b"gpiomon"})
                and not (b"ros2" in names and b"bag" in argv), f"다른 시험 프로세스: {path}")


def drive_sample(pid):
    proc = Path("/proc") / str(pid)
    require(any(fd.resolve() == Path("/dev/ttyAMA1").resolve() for fd in (proc/"fd").iterdir()),
            "drive UART 점유 확인 실패")
    io = dict(line.split(": ") for line in (proc/"io").read_text().splitlines())
    return int(io["rchar"])


def snapshot():
    begin = time.monotonic_ns()
    fields = {k:Path("/proc",k).read_text() for k in ("stat","interrupts","softirqs")}
    fields["wall"] = time.time_ns()
    fields["mono"] = time.monotonic_ns()
    fields["span_ns"] = fields["mono"]-begin
    return fields


def launch(stack, argv, stdout, stderr=None, *, stdin=None):
    out = stack.enter_context(stdout.open("x"))
    err = stack.enter_context(stderr.open("x")) if stderr else out
    p = subprocess.Popen(argv, stdin=stdin, stdout=out, stderr=err, start_new_session=True)
    def stop():
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:
                p.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL)
                p.wait()
                raise RuntimeError(f"{argv[0]} 정상 종료 실패")
    stack.callback(stop)
    return p


def run(path, bench):
    import gpiod
    import smbus
    stop_signal = []
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
        signal.signal(sig, lambda signum, frame: stop_signal.append(signum))
    children = []
    def check():
        if stop_signal:
            raise InterruptedError("사용자 중단")
        require(all(p.poll() is None for p in children), "기록기 또는 부하 프로세스 조기 종료")
        err = path/"gpiomon.err"
        require(not err.exists() or err.stat().st_size == 0, "GPIO 기록 오류: gpiomon.err 확인")
        space(path)
    def until(deadline):
        while True:
            check()
            remaining = deadline-time.monotonic_ns()
            if remaining <= 0:
                return
            time.sleep(min(.25,remaining/1e9))

    with ExitStack() as jobs:
        if path.name.startswith(("B", "C")):
            require(shutil.which("ros2"), "ROS 환경을 먼저 source 하세요")
            require(os.environ.get("ROS_DOMAIN_ID")=="119" and os.environ.get("ROS_LOCALHOST_ONLY")=="1",
                    "B/C는 ROS_DOMAIN_ID=119, ROS_LOCALHOST_ONLY=1 필요")
            if path.name.startswith("B"):
                bag = launch(jobs,["ros2","bag","record","-o",str(path/"bag"),
                                   "/rc_channels","/rc_uplink_lq"],path/"bag.log")
                children.append(bag)
            drive = launch(jobs,[str(DRIVE),"--ros-args","-p","port:=/dev/ttyAMA1",
                                 "-p","baud:=420000"],path/"drive.log")
            children.append(drive)
            until(time.monotonic_ns()+10_000_000_000)
        mon = launch(jobs,["gpiomon","--line-buffered","--format=%o %e %s.%n",
                           "gpiochip0","24","25"],path/"edges.log",path/"gpiomon.err")
        children.append(mon)
        until(time.monotonic_ns()+1_000_000_000)
        with gpiod.Chip("gpiochip0") as chip:
            for pin in (24,25):
                require(chip.get_line(pin).consumer()=="gpiomon", f"BCM{pin} 기록기 준비 실패")
        with ExitStack() as hardware:
            _,oe = bench.request_oe(hardware,gpiod,"pwm-ab")
            bus = smbus.SMBus(1)
            hardware.callback(bus.close)
            shutdown_times = []
            def cutoff():
                shutdown_times.append(bench.shutdown(oe, bus))
            hardware.callback(cutoff)
            prescale, counts = bench.prepare(bus)
            require((prescale,counts)==(138,[307,307]), "중립 설정 되읽기 실패")
            check()
            oe.set_value(0)
            until(time.monotonic_ns()+1_000_000_000)
            start = snapshot()
            if path.name.startswith("C"):
                start["drive_rchar"] = drive_sample(drive.pid)
            until(start["mono"]+30_000_000_000)
            end = snapshot()
            if path.name.startswith("C"):
                end["drive_rchar"] = drive_sample(drive.pid)
                require(end["drive_rchar"] > start["drive_rchar"], "drive 읽기 활동 없음")
            until(time.monotonic_ns()+1_000_000_000)
        until(time.monotonic_ns()+1_000_000_000)
    require(not stop_signal and mon.returncode==0, "중단 또는 GPIO 기록기 종료 오류")
    if path.name.startswith("B"):
        require(bag.returncode == 0, "bag 종료 오류")
    if path.name.startswith(("B", "C")):
        require(drive.returncode in (0, -signal.SIGINT), "drive 종료 오류")
    return dict(run=path.name,complete=True,start=start,end=end,gpiomon_rc=mon.returncode,
                counts=counts,prescale=prescale,shutdown_ns=shutdown_times[0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--check",action="store_true",help="장치를 열지 않는 정적 검사")
    action.add_argument("--run",choices=ORDER)
    parser.add_argument("--disconnected",action="store_true",help="추진 배터리 및 양쪽 ESC 신호선 분리 확인")
    parser.add_argument("--root",type=Path,default=ROOT)
    args = parser.parse_args()
    root = args.root.resolve()
    bench = static_check(root)
    if args.check:
        print("정적 검사 통과:",MODULE,"/ GPIO·UART·I²C 점유는 --run에서 검사")
        return
    require(args.disconnected, "추진 배터리와 양쪽 ESC 신호선을 분리한 뒤 --disconnected 지정")
    root.mkdir(parents=True,exist_ok=True)
    with (root/".lock").open("a") as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        device_check()
        path = root/args.run
        path.mkdir(exist_ok=False)
        try:
            record = run(path,bench)
            tmp = path/"window.tmp"
            tmp.write_text(json.dumps(record))
            tmp.replace(path/"window.json")
            ok, _, report = evaluate(path)
            print(report)
            require(ok, "기록 이상·비교 제외")
        except BaseException as exc:
            (path/"error.txt").write_text(f"{type(exc).__name__}: {exc}\n")
            raise
        print("유효 회차:",path)


if __name__=="__main__":
    try:
        main()
    except (ValueError,OSError,RuntimeError) as exc:
        sys.exit(str(exc))

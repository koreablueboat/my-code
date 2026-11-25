#!/usr/bin/python3
"""Real UART, exclusively PREVIEW. Run after sourcing the installed workspace."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from contextlib import ExitStack
from datetime import datetime

BASE = Path(__file__).resolve().parent
BIN = BASE.parent / "install"
TOPICS = "/rc_channels /rc_uplink_lq /boat/cmd_vel /boat/estop /boat/motor_status /boat/motor_preview /rosout".split()


def record():
    os.environ.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="121", PYTHONDONTWRITEBYTECODE="1")
    if not os.access("/dev/ttyAMA1", os.R_OK | os.W_OK):
        sys.exit("수신기 UART 접근 불가")
    print("미완료 항목: 중립 데드밴드만 측정합니다. SB MANUAL, SA KILL, 두 스틱 중앙.")
    if input("조작할 준비가 되면 준비 입력: ").strip() != "준비":
        sys.exit("시험 시작 전 취소")
    import rclpy
    rclpy.init(args=[])
    probe = rclpy.create_node("preview_trial_preflight")
    try:
        time.sleep(2)
        if any(probe.count_publishers(t) for t in TOPICS[:-1]) or any(
                name in ("boat_motor_node", "boat_rc", "kaboat_control") for name in probe.get_node_names()):
            sys.exit("시험 도메인 점유")
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    path = BASE/"results"/datetime.now().strftime("%Y%m%d_%H%M%S")
    path.mkdir(parents=True,exist_ok=False)
    print(path, flush=True)
    children = []
    errors = []
    with ExitStack() as stack:
        def launch(name, args):
            log = stack.enter_context((path/(name+".log")).open("x"))
            proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            def stop():
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(15)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGTERM)
                        proc.wait(5)
                        errors.append(name+" 종료 지연")
                if proc.returncode not in (0,-signal.SIGINT):
                    errors.append(f"{name} 종료코드 {proc.returncode}")
            stack.callback(stop)
            children.append(proc)
        launch("bag", ["ros2","bag","record","-o",str(path/"bag"),*TOPICS])
        launch("motor",[BIN/"kaboat_motor/lib/kaboat_motor/boat_motor","--backend","preview"])
        launch("rx",[BIN/"boat_rc/lib/boat_rc/drive","--ros-args","-p","port:=/dev/ttyAMA1","-p","baud:=420000"])
        launch("control",[BIN/"kaboat_control/lib/kaboat_control/control","--ros-args","-p","expected_backend:=preview"])
        discovery_end = time.monotonic()+30
        feedback = None
        while True:
            if any(proc.poll() is not None for proc in children):
                errors.append("시험 프로세스 조기 종료")
                break
            if feedback is None:
                baglog = (path/"bag.log").read_text(errors="replace")
                if all(f"Subscribed to topic '{t}'" in baglog for t in TOPICS):
                    launch("axes",[sys.executable,"-B",str(BASE/"axes.py")])
                    feedback = stack.enter_context((path/"axes.log").open())
                    print("준비 완료. 중앙에서 SA 허용 후 CH1만 1020~1050에 유지하세요.",flush=True)
                elif time.monotonic()>discovery_end:
                    errors.append("기록기 토픽 연결 시간 초과")
                    break
            if feedback is not None:
                lines = feedback.readlines()
                if lines:
                    print("".join(lines),end="",flush=True)
            logs = (path/"control.log").read_text(errors="replace")
            if any('"previous_phase": "RUNNING"' in line and '"phase": "KILLED"' in line for line in logs.splitlines()):
                time.sleep(1.5)
                break
            time.sleep(.25)
    if not (path/"bag"/"metadata.yaml").is_file():
        errors.append("bag 종료 메타데이터 누락: 저장된 표본만 분석")
    result = subprocess.run([sys.executable,"-B",str(BASE/"analyze.py"),"--partial","--deadband-only",str(path/"bag")],
                            check=False, capture_output=True, text=True)
    (path/"analysis.txt").write_text(result.stdout)
    print(result.stdout, end="")
    if result.returncode:
        errors.append(result.stderr.strip() or "중립 데드밴드 미통과")
    if errors:
        (path/"errors.txt").write_text("\n".join(errors)+"\n")
        sys.exit("; ".join(errors))


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: sys.exit("기록 종료 요청"))
    record()

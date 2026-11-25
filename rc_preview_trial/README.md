# 실제 RC 가변속 PREVIEW 보완

진행 대상은 판정 보류였던 중립 데드밴드 한 항목이다. 추진 배터리와 양쪽 ESC 신호선을 분리하고 SB는 MANUAL 가운데, SA는 KILL, 두 스틱은 중앙에 둔다. 실행기는 로컬 도메인 121에서 설치된 UART 수신기, 제어기, PREVIEW 백엔드와 bag을 사용한다.

```bash
cd ~/kaboat_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -B rc_preview_trial/record.py
```

1. 직접 조작할 준비가 됐을 때 터미널에 **준비**를 입력한다. 기록기 구독 완료 안내까지 SA KILL·중립을 유지한다.
2. 중립 KILL을 2초 이상 유지한 뒤 SA 허용으로 전환하고 중앙에서 무장을 기다린다. 화면의 CH1을 보며 조향만 살짝 오른쪽으로 움직인다.
3. **CH1 1020~1050, CH2 980~1004**에서 5초 연속 유지한다. 실시간 표시는 현재값, 목표, 경과시간과 남은 시간이다. 입력 범위 이탈이나 오래된 RC·LQ·모터 상태는 해당 유지 시간만 초기화한다. 시간제한에 맞춰 조작할 필요가 없다.
4. PASS 또는 FAIL 표시 후 중앙으로 복귀하고 SA KILL로 마친다. 최종 판정은 종료된 bag의 수신 시각으로 같은 규칙을 적용한다. 범위 진입 후 첫 0.12초는 명령 전달 안정화 시간이다. 이후 cmd_vel 영값과 양쪽 논리 0·목표 1500 µs를 검사한다. 조기 종료 시 5초 연속 표본이 없으면 판정 보류다.

실시간 진행은 ROS 구독에서 읽는다. 운영 중인 SQLite 파일을 열지 않는다. 결과는 새 회차 디렉터리에 보관한다. 정상 종료 및 파일 점유 해제 후에만 분석한다. 부분 기록도 종료 확인 후 저장 표본만 분석한다.

```bash
/usr/bin/python3 -B rc_preview_trial/analyze.py --deadband-only rc_preview_trial/results/회차/bag
/usr/bin/python3 -B rc_preview_trial/fake_faults.py
```

기존 회차 전체 분석은 같은 분석기에서 `--deadband-only`를 생략한다. `deadband_samples.py`는 저장된 실제 RC 프레임을 감독기 입력 매핑으로 검사한다. 가짜 어댑터 시험은 설치된 모터의 콜백과 PCAOutput 메서드를 호출하고 시계·핀·채널은 메모리 모형을 사용한다.

# RC 보트 수동 제어

ROS 2 Humble에서 양쪽 T200을 RC로 조종하는 프로그램은 `kaboat_manual`의 `manual` 실행기다. CRSF 수신기와 MANUAL 감독기, 차동 믹서, PCA9685 모터 노드를 함께 관리한다. 전진·후진·제자리 회전·복합 조향·중립·SA KILL을 지원한다.

**실물 운항은 미승인이다.** 배포한 `manual_dual` 설정은 보정 후보이며 기본 `check`와 `start`는 승인 검사에서 종료된다. 독립 추진 전원 차단, ESC 신호 상실 정지, 양쪽 실물 매핑 등 현장 승인 조건과 모터 기동 후 최대 20초 제한이 적용된다. [수동 제어 사용 설명서](src/kaboat_manual/README.md)

## 설치된 프로그램 사용

각 터미널에서 환경을 먼저 불러온다.

```bash
source /opt/ros/humble/setup.bash
source ~/kaboat_ws/install/setup.bash
ros2 run kaboat_manual manual check
```

현장 승인 프로파일이 발행되면 해당 경로로 검사하고 현장 터미널에서 시작한다. 아래 경로는 예시다.

```bash
ros2 run kaboat_manual manual check --profile ~/approved/manual_dual.json
ros2 run kaboat_manual manual start --profile ~/approved/manual_dual.json --duration 20
```

같은 사용자의 다른 터미널에서 상태를 확인한다. 정상 종료는 스틱 중앙 복귀, SA KILL, 종료 요청, 독립 추진 전원 차단 순서다. 회차별 로그와 종료코드는 `~/.local/state/kaboat-manual/`에 기록된다.

```bash
ros2 run kaboat_manual manual status
ros2 run kaboat_manual manual stop
ros2 run kaboat_manual manual diagnose
```

## 빌드와 기록

소스에서 다시 설치하려면 워크스페이스 루트에서 실행한다.

```bash
cd ~/kaboat_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to kaboat_manual
source install/setup.bash
```

Git에는 소스·시험 도구·판정 기록을 보관한다. 원본 bag·GPIO 로그·로컬 백업은 Pi 또는 별도 아카이브에서 원래 경로로 복원해야 기존 분석기의 입력과 기록 문서의 원본 링크를 사용할 수 있다. [소프트웨어 시험 결과](docs/records/manual-dual-software.txt) · [벤치 검증 기록](docs/records/bench-stop.md) · [개발 이력](docs/records/WORKLOG.md)

PREVIEW 및 향후 AUTO 연동에는 감독기의 후보 명령 토픽 `/boat/auto/cmd_vel`을 사용한다. 최종 `/boat/cmd_vel`은 제어기가 단독 발행한다. [제어기 운용 방법](src/kaboat_control/README.md)

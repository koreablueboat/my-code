# 양쪽 T200 수동 제어

Pi에서 실제 RC → CRSF 수신기 → MANUAL 감독기 → 차동 믹서 → 단일 모터 노드 → PCA9685 CH0(좌)·CH1(우)를 실행한다. 추진기 두 대를 사용하는 실물 경로 이름은 `manual_dual`이다. ROS 2 Humble, 설치된 boat_rc/kaboat_control/kaboat_motor, gpiod 1.x, smbus, fuser, rosbag2 SQLite가 필요하다. 로컬 ROS 도메인 124, UART /dev/ttyAMA1(420000 baud), I²C-1 0x40, OE BCM17을 쓴다. BCM27, MOSFET, 상태등, GPS를 사용하지 않는다.

## 먼저 확인할 사항

배포 파일은 **미승인 보정 후보**다. ESC 사양, OE 기본 차단, Pi/호스트 정지까지 포함한 독립 정지, ESC 신호 상실 정지의 기존 네 승인 조건에 더해 양쪽 실제 전진/후진 기동점·상한·추력 방향·중립 정지 매핑 확인이 필요하다. 실물 승인본은 해당 배선의 manual_dual 범위, 검증된 차단기 모델 및 승인 기록을 참조해야 한다. 좌측 단발 시험 승인이나 단순 키 입력은 이 증거를 대신할 수 없다. 모든 승인 플래그가 false인 후보를 설치한다. 프로파일 검사는 현장 배선이나 물리 스위치 상태를 감지하지 않는다.

XT90은 무전원 분리점으로 사용한다. 추진 전원만 끊을 수 있는 DC 부하 개방 정격의 독립 차단 수단과 현장 차단 담당자가 필요하다. Pi 정지·전원 상실, 복전 시 재기동 방지, ESC 신호 상실, 물리 정지 시간 등은 설치별 승인 기록에 남긴다. OE HIGH는 소프트웨어 요청값이다. 안전 승인 전에는 check/status/diagnose까지만 사용한다.

## 명령

모든 터미널에서 `source /opt/ros/humble/setup.bash`와 `source ~/kaboat_ws/install/setup.bash`를 먼저 실행한다.

```bash
ros2 run kaboat_manual manual check
ros2 run kaboat_manual manual check --profile ~/approved/manual_dual.json
ros2 run kaboat_manual manual start --profile ~/approved/manual_dual.json --duration 20
ros2 run kaboat_manual manual status
ros2 run kaboat_manual manual stop
ros2 run kaboat_manual manual diagnose
ros2 run kaboat_manual manual diagnose --session ~/.local/state/kaboat-manual/회차폴더
```

후보에서 check와 start가 오류로 종료되는 것이 정상이다. 두 번째 경로는 검증 후 발행할 현장 승인본의 예시다. start는 현장 터미널 전경에서 돌아가고 현재 모터 상태를 출력한다. 두 번째 터미널에서 같은 사용자로 status/stop을 호출한다. start는 승인 및 작업자 확인 → 장치 점유·공간 검사 → bag·수신기·제어기 → 모터 순으로 시작한다. 실행시간은 **모터 기동부터 1~20초**이며 자동 재시작은 없다. 이 소프트웨어는 현행 유한 실행시간 조건을 유지한다.

## RC 조작

1. 양쪽 T200 수중 고정 및 회전체 주변 이격, 독립 차단 담당자 대기. SB MANUAL, SA KILL, 두 스틱 중앙에서 시작한다. 구성과 주변을 확인한 후 시작 프롬프트에 DUAL MANUAL을 입력한다.
2. 상태가 정상이고 모터가 미무장인 동안 중앙·SA KILL을 1초 이상 유지한 뒤 SA를 허용한다. 중립에서 초기화 2음, 실제 정지, 양쪽 중립 되읽기와 `armed=true`, `neutral_init_complete=true`를 확인한다. 초기화 중 비영점 입력은 무장 해제한다. 준비 시간이 부족하면 회차를 종료한다.
3. 스로틀 전후로 전진·후진, 스로틀 중앙에서 조향 좌우로 제자리 회전, 두 스틱을 함께 움직여 복합 조향한다. 양쪽 스틱 중앙은 좌우 307카운트 중립이다. 방향 반전은 각 채널의 영점 및 반전 대기(최소 0.4초)를 거친다. 실제 프로펠러 방향·추력은 현장 기록으로 판정한다.
4. SA KILL은 OE 차단·래치를 요청하며 프로그램은 남은 시간 동안 상태를 보고한다. RF 단절 후 링크가 복구되어도 자동 재무장하지 않는다. 재무장은 건강 상태 회복 후 중앙·SA LOW 유지와 새 HIGH 전환이 필요하다. 정상 종료는 중앙 → SA KILL → stop(또는 전경 Ctrl+C) → 독립 추진 전원 차단 순서다. 이상 회전이나 소프트웨어 응답 상실 시 즉시 독립 차단기를 사용한다.

## 데드밴드 반영 방식

Basic ESC R3 명목 정지대는 1475~1525 µs다. 교체 보드의 보정 후보 28.437 MHz, 50 Hz 요청, PRESCALE 0x8A에서 1카운트는 약 4.888 µs다. RC 스틱 데드밴드(0.08)와 기존 믹서의 정규화 출력·슬루·반전 대기는 그대로다. 다음 변환은 **믹서의 각 채널 출력**에 적용한다.

- 0은 즉시 307(약 1500.62 µs)이다. 비영점은 채널별 sign과 방향별 시작점·상한 사이에서 크기에 따라 선형 보간한다. 크기 1은 기존 논리 전진 상한 +0.15 또는 프로파일 후진 상한 −0.04에 대응한다.
- 미검증 후보 양쪽은 전진 313~319(약 1530~1559 µs), 후진 301~298(약 1471~1457 µs)이다. 누적 소수 위치를 초당 최대 24카운트로 이동시키고 최종 레지스터에서 반올림한다. 비영점이 생기자마자 경계 밖으로 뛰지 않으며 이동 중에는 정지대 안의 값도 거친다. 중앙 복귀·차단은 이 추가 슬루를 기다리지 않는다.
- 후보의 정착 카운트 CH0/CH1은 직진 끝 317/317, 후진 끝 299/299, 좌회전 298/315, 우회전 315/298, 전진+좌 조향 끝 307/319, 중립 307/307이다. 수중 RPM·추력의 검증값이 아니다. 채널별로 독립된 후보 범위를 저장하며 변경 시 해당 설치의 보정과 승인이 필요하다. [제조사 Basic ESC R3 사양](https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/)

## 상태·오류 분석

각 회차는 ~/.local/state/kaboat-manual/ 아래에 유일한 폴더를 만든다. latest는 최근 회차다. profile.json은 실행 시점 복사본이다. rx.log, control.log, motor.log, bag.log, bag/, result.json을 함께 확인한다. status는 실행기 잠금과 최근 로그를 보여준다. 기록된 마지막 armed/OE 상태는 실시간 전압이나 현재 회전을 보증하지 않는다. stop은 해당 회차의 실행기에 종료를 요청하고 종료를 기다린다. 종료 지연 시 터미널 응답을 기다리지 말고 독립 추진 전원을 차단한다.

| 관찰 | 확인할 기록 및 조치 |
|---|---|
| hardware refused / mapping unverified | 승인 범위와 설치 증거 확인. 후보로는 기동 불가 |
| UART/I²C 점유, startup timeout | 다른 모터·기록기 세션과 포트 접근권한, rx.log 및 bag.log 확인 |
| radio_kill / rc_timeout / rf_link_stale_or_lost | control.log의 전이와 bag의 /rc_channels, /rc_uplink_lq 확인 |
| estop_latched / command_timeout / backend mismatch | /boat/estop·/boat/cmd_vel 발행 상태, 실행 중 단일 모터/제어기와 backend 확인 |
| 채널 되읽기 또는 PRESCALE 오류 | OE 차단 후 종료. 전원 차단 상태에서 배선·보드·프로파일 점검 |
| 종료 사유 | motor.log의 duration/signal, result.json의 ended_by·exit_codes·forced_stop·bag_closed를 함께 확인 |
| 비정상 종료로 완료 기록 없음 | 물리 차단 확인 후 diagnose로 마지막 상태 및 남은 프로세스를 확인. 임의 재시작 금지 |

종료된 bag은 `ros2 bag info ~/.local/state/kaboat-manual/latest/bag`로 요약할 수 있다. 기록 중 SQLite를 열지 않는다. 실제 수동 운항 승인은 별도이며 기존 무부하 GPIO 폭 정밀도 FAIL도 유지한다.

# RC 단절과 Pi 정지 벤치 측정

RC 단절·복구는 실제 조종기와 PREVIEW에서 확인했다. RC 연동 PCA 파형과 Pi 정지 시 물리 차단은 측정 대기다. 추진 전원을 분리하고 수신기, 제어 전원과 계측기만 연결한다. 실제 ESC 모델과 보드 배선, 펌웨어, 예상 차단 시간의 근거를 측정 전에 기록한다. 출력 차단 시간은 Pi 밖의 계측기로 사건 발생부터 OE와 두 PWM 신호가 안전 상태에 도달할 때까지 측정한다.

| 시험 | 관찰 방법 | 판정 기준 | 실측과 판정 |
|---|---|---|---|
| 송신기 단절 및 복구 | 실제 수신기와 PREVIEW에서 송신기를 끄고 RC 프레임·LQ·estop·모터 상태 시각을 기록한 뒤 켠다. | 단절 시 정지하고 링크 복구 후에도 의도적인 중립·재무장 전까지 출력이 0이다. | PREVIEW 통과. 복구 후 22.742초 관찰. 아래 bag 근거 참조 |
| Pi 제어 프로세스 정지 | 독립 차단 회로와 제조사 사양을 확인한 벤치에서 모터 프로세스를 일시 정지한다. 외부 계측기로 OE 및 CH0/CH1을 연속 관찰한다. | 외부 차단기가 사전에 정한 허용 시간 내 동작하고 프로세스 재개만으로 출력이 재개되지 않는다. | 대기 |
| Pi 전원 상실 및 복구 | 제어 전원만 끊었다가 복구하며 외부 계측기로 OE와 PWM을 관찰한다. | 정지·재부팅 중 출력이 차단되고 명시적 재무장까지 차단 상태를 유지한다. | 대기 |

기록할 값은 시험 시각, 장비·프로파일, 고장 발생 시각, 마지막 RC 및 LQ, OE와 좌우 PWM 변화 시각, 복구 후 무장 상태, 파형 파일과 판정이다. RF PREVIEW 시험과 PCA 출력 시험의 결과를 각각 적는다. 제어기는 설정한 expected_backend와 모터 상태의 backend가 일치해야 한다. 버튼 없는 시험에는 PREVIEW와 무부하 고정 중립 전용 bench_neutral을 사용한다. 실제 ESC의 신호 상실 반응과 추진 전원 차단 성능은 별도로 실측한다.

## 2026-11-25 교체 PCA와 왼쪽 Basic ESC R3 중립 시험

- 작업자 실측. 교체 PCA에서 PRESCALE 0x8A, CH0·CH1 각 307카운트로 3회 측정했다. 총 600펄스 모두 1475~1525 µs 범위였고, 매회 차단 후 200 ms 동안 두 채널 모두 0펄스였다.
- 작업자 현장 관찰. 왼쪽 Basic ESC R3 한 대에서 전원 투입 시 3음, CH0 중립 명령 후 초기화 2음을 들었고 지속 회전은 관찰되지 않았다.

## 왼쪽 T200 첫 수중 단발 시험

- 작업자가 ESC 초기화 2음, 왼쪽 추진기의 짧은 회전, 중립 복귀 후 정지를 모두 확인했다.
- 터미널에서 `RUN LEFT`를 입력한 뒤 `단발 명령 완료, 출력 차단.`이 표시됐다.

## 오른쪽 CH1 실물 시험

| 시험 | 터미널 기록 | 작업자 관찰 |
|---|---|---|
| Basic ESC R3 중립 | PRESCALE=0x8A, CH1=307카운트, CH0·CH2~15 FULL OFF. OE LOW/HIGH 요청 구간 7.007227초 | 초기화 2음 확인, 지속 회전 없음 |
| T200 수중 단발 | `RUN RIGHT` 입력 후 `단발 명령 완료, 출력 차단.` 표시 | 초기화 2음, 오른쪽 추진기의 짧은 회전, 중립 복귀 후 정지 확인 |

7.007227초는 프로그램이 OE를 요청한 두 시각의 차이다. OE 핀의 전압 전환과 RF 단절 후 차단 시간은 이번 실측에 포함되지 않았다.

## 2026-11-25 버튼 없는 RC PREVIEW 결과

도메인 118, 실제 수신기 → 제어기 → PREVIEW. 시각은 KST bag 기록 시각이다. 좌우 값은 `/boat/motor_preview.data[0:2]`, 채널은 `/rc_channels`의 CH2(인덱스 1)·CH5(인덱스 4)다.

| 항목과 판정 | bag (현장 Pi) | 근거 표본 |
|---|---|---|
| 비영점 출력 중 SA KILL 통과 | [rc_sa_kill_loaded_preview_20261125_180232](/home/ubuntu/rc_sa_kill_loaded_preview_20261125_180232) | 18:05:43.085291 CH5 1792→191을 기준으로 −24.006 ms에 좌우 각각 +0.013734, +14.474 ms에 estop=true, +21.571 ms에 좌우 0, +32.515 ms에 armed=false·estop_latched=true. 첫 0 출력 앞뒤(+18.259/+30.704 ms)의 CH2는 모두 1157이었다. |
| 링크 단절 정지 통과 | [rc_link_loss_preview_20261125_181232](/home/ubuntu/rc_link_loss_preview_20261125_181232) | 18:14:45.020437 armed=true, .048271 마지막 RC는 CH5=1792·CH2=1189. .359999 좌우 각각 +0.01745, .370203 RUNNING→KILLED(`rc_timeout`), .370683 estop=true, .374786 좌우 0, .414530 armed=false·estop_latched=true. 첫 LQ=0은 .537718이었다. 채널 프레임 공백은 32.280초였고 실제 송신기 전원 OFF 시각은 미기록이다. |
| 링크 복구 후 22.742초 미무장 유지 통과 | [같은 링크 단절 bag](/home/ubuntu/rc_link_loss_preview_20261125_181232) | 18:15:17.328755 RC 복귀, 17.398810 첫 유효 LQ=25부터 18:15:40.141174 기록 끝까지 CH5=1792 (5,544개), armed=false·estop_latched=true (43개), 좌우 0 (1,566개). RUNNING 재진입 없음. 이 bag에는 종료용 SA KILL이 없다. |
| 새 SA 전환에 의한 중립 재무장 통과 | [rc_explicit_rearm_preview_20261125_182116](/home/ubuntu/rc_explicit_rearm_preview_20261125_182116) | 초기 KILL에서 18:21:22.615424 좌우 0, .813255 armed=false·estop_latched=true. 18:21:43.207443 CH1·CH2=992에서 CH5 191→1792, 18:21:44.161983 armed=true·estop_latched=false, .186895 RUNNING. 허용 중 좌우 983표본 모두 0. |
| 종료용 KILL 통과 | [같은 명시적 재무장 bag](/home/ubuntu/rc_explicit_rearm_preview_20261125_182116) | 18:22:32.318364 CH5 1792→191, .332326 RUNNING→KILLED(`radio_kill`), .333079 estop=true, .335650 좌우 0, .362769 armed=false·estop_latched=true. |

## 실제 수신기 조종기 시험 절차

**현재 도달 범위는 실제 RC 입력 → 제어기 → 좌우 PREVIEW 목표값이다.** 설치된 `drive`는 CRC가 유효한 CRSF 채널 프레임과 링크 통계를 발행한다. `control`의 기본 기대 백엔드는 preview다. `boat_motor --backend preview`는 PCA를 열지 않는다. 물리 CH0·CH1로 RC 명령을 내보내는 단계는 보류한다.

설치된 양방향 예제에서 `esc_spec_verified`, `oe_default_disabled_verified`, `independent_stop_verified`, `signal_loss_stop_verified`가 모두 false다. 소스의 실물 템플릿도 미설정이다. `expected_backend`는 모터 상태와의 일치 여부만 검사한다. 좌우 단발 성공, 예제의 1500 µs, 벤치 도구의 0x8A·307카운트를 운항 프로파일 승인으로 사용하지 않는다.

### 현장 기동과 기록

추진 배터리와 양쪽 ESC 신호선을 분리하고 수신기와 Pi 제어 전원을 유지한다. SA는 위(KILL), SB는 가운데(MANUAL), 두 스틱은 중앙에서 시작한다. 다음 명령은 **현장 Pi의 각 SSH 터미널**에서 실행한다. 아래 118은 이 시험에 고른 로컬 ROS 도메인이다. 시험 전에 이 도메인에 다른 제어·모터 노드가 없는지 확인한다.

```bash
source ~/kaboat_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=118
ros2 node list
```

각각 별도 터미널에서 위 환경을 설정하고 다음 순서로 기동한다.

1. 수신기 UART를 읽는다. 실제 채널과 LQ가 들어오는지 먼저 확인한다.

   ```bash
   ros2 run boat_rc drive --ros-args -p port:=/dev/ttyAMA1 -p baud:=420000
   ```

2. 버튼 없는 프로젝트 배선(물리 13번 미배선)에 맞춰 모터를 PREVIEW로 시작한다. 이 구성에서 정지 요청은 제어기의 /boat/estop으로 전달된다. 이전 BCM27 진단 시험에서 사용한 --test-estop-gpio는 개방 입력을 풀업 HIGH로 읽어 정지를 유지할 수 있다.

   ```bash
   ros2 run kaboat_motor boat_motor --backend preview
   ```

3. 제어기를 시작한다. 이 터미널의 전이 로그에 phase, reason과 단조 시계 시각이 나온다.

   ```bash
   ros2 run kaboat_control control --ros-args -p expected_backend:=preview
   ```

4. 기록용 터미널에서 bag을 시작한다. 구독이 연결된 뒤 시험을 시작하고 사건 순번과 실제 스위치 조작을 적는다.

   ```bash
   ros2 bag record -o "$HOME/rc_bench_$(date +%Y%m%d_%H%M%S)" /rc_channels /rc_uplink_lq /boat/estop /boat/cmd_vel /boat/motor_status /boat/motor_preview /rosout
   ```

5. 관찰용 터미널에서 다음을 확인한다. `cmd_vel`·`estop` 발행자는 각각 제어기 한 개, `motor_status` 발행자는 PREVIEW 모터 한 개여야 한다. 두 RC 토픽도 각각 실제 수신기 노드 한 개만 발행해야 한다. `--once`는 한 메시지를 기다리므로 단절 중에는 연속 기록한 bag과 전이 로그를 사용한다.

   ```bash
   for t in /rc_channels /rc_uplink_lq /boat/cmd_vel /boat/estop /boat/motor_status; do ros2 topic info -v "$t"; done
   ros2 topic echo --once /rc_channels
   ros2 topic echo --once /rc_uplink_lq
   ros2 topic echo --once /boat/motor_status
   ros2 topic echo --once /boat/motor_preview
   ros2 service call /boat/control/rearm std_srvs/srv/Trigger '{}'
   ```

   마지막 서비스는 phase·reason·ready_to_arm을 **조회**한다. 실제 재무장 요청은 조종기의 SA 전환으로만 시작한다. `motor_preview.data`의 순서는 좌·우 논리 출력, 좌·우 목표 µs다. 설치된 양방향 PREVIEW에서 정지 목표는 `[0, 0, 1500, 1500]`이다. 항상 `motor_status.backend=preview`와 `armed`를 함께 읽는다. 중립 값만으로 무장 여부를 구별할 수 없다.

### 현장 조작과 판정

| 순서 | 조작 | 확인할 기록 |
|---|---|---|
| 1. 초기 KILL | SB 가운데(수신기 CH6 약 997), SA 위(CH5 약 191), CH1 조향·CH2 스로틀은 중앙(약 992) | RC·LQ 갱신, KILLED, estop=true, armed=false, 좌우 논리값 0. 수신기 CH 번호와 PCA 출력 CH 번호는 별개다. |
| 2. 명시적 PREVIEW 재무장 | RC·LQ와 모터 상태가 정상 갱신되면 KILL·중립을 1초 이상 유지한다. 조회에서 ready_to_arm=True를 확인한 뒤 SA를 아래(CH5 약 1792)로 한 번 전환하고 스틱을 중앙에 둔다. | 제어기가 정지 해제 요청 → reset → 새 모터 상태 → 0 명령 0.60초 → PREVIEW arm → 새 상태 확인을 거쳐 RUNNING이 된다. armed=true, estop_latched=false를 확인한다. 정지 입력이 남거나 실패하면 해당 단계와 사유를 기록한다. |
| 3. SA KILL | RUNNING에서 스로틀을 중앙에서 조금 움직여 두 논리값이 실제로 0을 벗어난 상태에서 SA를 위로 올린다. | CH5 전환, KILLED/radio_kill, estop=true, cmd_vel=0, armed=false 및 좌우 논리값 0 복귀를 기록한다. 스틱을 중앙으로 복귀한다. |
| 4. 송신기 단절 | 2번으로 PREVIEW만 다시 무장하고 두 논리값이 0을 벗어난 것을 확인한다. SA는 허용 위치에 두고 송신기 전원을 끈다. | 마지막 RC·LQ와 이후 발행 내용을 기록한다. 수신기가 채널값을 계속 보내는지, LQ가 0이 되는지 또는 발행이 끊기는지 구분한다. KILLED의 실제 사유와 armed=false, 좌우 0을 확인한다. |
| 5. 링크 복구 | 송신기 스틱을 중앙으로 두고 새 KILL→허용 조작 없이 전원을 복구한다. 송신기 시작 조건으로 SA 이동이 필요하면 실제 CH5 전환도 기록한다. | RC·LQ 정상 갱신 뒤에도 KILLED, armed=false, 좌우 0이어야 한다. 뜻밖의 RUNNING이나 출력 재개가 있으면 SA를 KILL에 두고 시험을 종료한다. |
| 6. 명시적 재무장 | 복구 상태에서 2번의 SA 위·중립 유지와 새 SA 아래 전환을 수행한다. | 해당 전환 뒤에만 RUNNING으로 복귀하고 새 RC 입력에 좌우 PREVIEW가 응답하는지 확인한다. 마지막에 SA KILL, armed=false, 좌우 0을 확인한 뒤 bag과 노드를 Ctrl+C로 종료한다. |

설치 제어기의 RC 만료 기준은 마지막 유효 프레임 후 0.30초 초과, LQ 만료는 1.0초 초과다. LQ 0 또는 1~100 밖 값도 KILL 조건이다. 오래된 값과 정상 범위 LQ를 수신기가 계속 새 프레임으로 보내면 만료 조건이 성립하지 않을 수 있다. 단절 후 제어기가 계속 RUNNING이면 SA KILL로 시험을 끝내고 실패로 기록한다. 이 숫자는 소프트웨어 판정 기준이며 차단 시간 실측값으로 사용하지 않는다.

### BCM24/25 되읽기와 외부 계측

| 관찰 경로 | 관찰값과 판정 범위 |
|---|---|
| Pi 입력 에지 | 이전과 같이 PCA CH0→BCM24, CH1→BCM25, 공통 GND로 연결하면 각 채널의 상승·하강 시각, 펄스 폭·주기, 마지막 펄스 및 정해진 관찰창의 추가 펄스 수를 기록할 수 있다. 펄스가 있으면 중립 PWM과 변화된 PWM을 구별할 수 있다. PREVIEW는 두 핀에 PWM을 만들지 않으므로 여기서 에지 0개는 RC 차단 성공의 증거가 아니다. |
| 설치된 벤치 도구 | `neutral_probe`의 2채널 캡처는 자신이 OE·PCA를 구동하는 고정 중립 시험이다. 단독 중립 모드와 단발 도구도 RC를 구독하지 않는다. 조종기 시험과 병행하지 않는다. |
| 외부 동시 계측 | 물리 차단 시간을 판정할 때는 별도의 로직 분석기·오실로스코프에서 SA 조작 또는 송신기 RF 차단의 독립 사건 표식, 실제 PCA OE 단자, CH0, CH1을 같은 시간축에 기록한다. 사건부터 OE HIGH까지와 각 채널의 마지막 펄스 종료까지를 각각 보고하고 이후 관찰창의 무펄스를 확인한다. 마지막 펄스가 사건 전이면 그대로 표시한다. 수신기 CRSF/LQ 변화를 시작점으로 삼았다면 수신기 검출 이후 지연으로 명시한다. Pi 로그와 bag 시각만으로 물리적인 RF 단절부터의 지연, OE 전압, Pi 정지 중 독립 차단 시간을 확정할 수 없다. |

현장 Pi에서 에지만 수동 관찰하려면 `gpiodetect`로 gpiochip0의 라벨이 `pinctrl-bcm2711`인지 확인한 뒤 별도 터미널에서 아래 입력 감시를 사용한다. 각 줄은 BCM 번호, 에지(1 상승·0 하강), 커널 시각이다. 감시 시작·종료 시각과 연결 상태를 함께 기록한다.

```bash
gpiodetect
gpiomon --line-buffered --format='%o %e %s.%n' gpiochip0 24 25
```

## 교체 보드의 운항 경로 점검과 다음 무부하 벤치

2026-11-25에 소스와 `install/<패키지>/lib/<패키지>/`의 설치 파일을 비교했다. 아래 모터 노드, PCA 어댑터, 프로파일 검증기, 양방향 예제, 벤치 초기화 모듈과 제어기·감독기 코드는 각각 일치했다.

| 경로 | 설치본에서 확인한 동작 |
|---|---|
| 운항 모터의 정지 입력 | `boat_motor --backend pca9685`는 PREVIEW용 옵션과 무관하게 BCM27을 풀업 입력으로 연다. 전기적 HIGH 또는 마지막 ROS estop=true이면 정지 입력이 활성이고, PCA가 열려 있으면 OE 차단을 요청한다. reset은 정지 입력이 활성이거나 이미 무장된 경우 거부된다. 프로젝트 배선에서 물리 13번은 개방이므로 운항 노드의 버튼 전제가 맞지 않는다. [모터 노드](../../src/kaboat_motor/boat_motor_node.py) · [양방향 모터 코어](../../src/kaboat_motor/reverse_motor_core.py) |
| 운항 PCA 주파수와 승인 조건 | 노드와 `PCAOutput` 모두 실물 프로파일을 검증한다. 어댑터는 `reference_clock_hz`를 Adafruit 드라이버에 전달하고 `frequency_hz`를 설정한다. 이후 드라이버가 기준 클럭과 PRESCALE로 계산한 주파수로 목표 µs를 duty로 바꾼다. 이 주파수는 파형 실측값이 아니다. 설치 예제의 25 MHz·50 Hz를 적용하면 계산상 PRESCALE은 0x79다. 예제의 검증 플래그 4개는 false이므로 이 프로파일은 하드웨어 접근 전에 거부된다. [PCA 어댑터](../../src/kaboat_motor/pca_backend.py) · [검증기](../../src/kaboat_motor/esc_config.py) · [예제](../../src/kaboat_motor/preview_bidirectional.json) |
| 교체 보드 벤치 보정 | 고정 벤치 도구는 0x79에서 측정한 주기 17.5726 ms로부터 기준 클럭을 28,437,000 Hz로 추정했다. 50 Hz용 PRESCALE 0x8A를 직접 쓰고 CH0·CH1 중립 307카운트를 되읽었다. 이 상수는 운항 PCA 경로로 전파되지 않는다. [벤치 초기화](../../src/kaboat_motor/bench_pca.py) · [양쪽 중립 실측 로그](/home/ubuntu/neutral_probe_20261125_160550.log) |

### RC 연동 고정 중립 벤치 (설치본)

실제 수신기 → 기존 제어기 → `boat_motor --backend bench_neutral` 한 세트를 사용한다. 모터 상태의 `backend=bench_neutral`, `fixed_neutral_bench=true`, `oe_low_requested`를 기록한다. 마지막 값은 소프트웨어가 요청한 OE 상태다. 기존 SA KILL, RC 0.30초·LQ 1.0초 만료, 중립 확인 및 새 SA 전환에 의한 재무장 절차가 적용된다. 이 백엔드의 고정 중립 게이트는 cmd_vel로부터 PWM 레지스터를 쓰는 경로가 없다. 무장마다 OE HIGH 상태로 설정과 16개 채널을 되읽은 뒤 CH0·CH1의 307카운트만 활성화한다. BCM27은 사용하지 않는다.

설치본 dry-run 명령은 `ros2 run kaboat_motor boat_motor --backend bench_neutral --dry-run`이다. 모의 시험 7건과 설치 제어기의 백엔드 선택 시험 9건을 통과했다. 이 검증에서 실제 GPIO·I²C 접근과 ROS arm 서비스 호출은 수행하지 않았다.

추진 배터리와 양쪽 ESC 신호선을 분리한다. PCA 로직 전원과 공통 GND, OE→BCM17, CH0→BCM24, CH1→BCM25 연결을 확인한다. SA KILL·SB MANUAL·스틱 중앙에서 시작한다. 아래 명령은 현장 Pi의 **각 터미널**에서 공통 환경을 설정한 뒤 실행한다.

```bash
source ~/kaboat_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=118
```

| 순서 | 터미널별 명령 |
|---|---|
| 1. 에지 기록 | `gpiomon --line-buffered --format='%o %e %s.%n' gpiochip0 24 25 > "$HOME/rc_neutral_edges_$(date +%Y%m%d_%H%M%S).log"` (gpiochip0 라벨 pinctrl-bcm2711 확인). 두 입력만 요청한다. |
| 2. bag | `ros2 bag record -o "$HOME/rc_fixed_neutral_$(date +%Y%m%d_%H%M%S)" /rc_channels /rc_uplink_lq /boat/estop /boat/cmd_vel /boat/motor_preview /boat/motor_status /rosout` |
| 3. 실제 수신기 | `ros2 run boat_rc drive --ros-args -p port:=/dev/ttyAMA1 -p baud:=420000` |
| 4. 제어기 | `ros2 run kaboat_control control --ros-args -p expected_backend:=bench_neutral` |
| 5. 모터 | `ros2 run kaboat_motor boat_motor --backend bench_neutral --duration 180` (기동 시점부터 180초). 같은 도메인의 모터와 PCA·OE 구동기는 각각 이 한 개만 사용한다. |
| 6. 상태 확인 | `ros2 topic echo /boat/motor_status` |

1. 초기 KILL에서 armed=false와 두 채널 무펄스를 확인한다. 정상 RC·LQ에서 중립 KILL을 1초 이상 유지한 뒤 SA를 허용으로 전환한다. RUNNING, armed=true, 두 채널 약 20 ms 주기와 약 1500 µs 폭을 기록한다.
2. 작은 전진 스틱 입력으로 bag의 cmd_vel이 비영점인 구간에도 양쪽 펄스가 같은 중립 폭을 유지하는지 비교한다. SA KILL 후 estop=true, 무장 해제·래치와 두 채널의 마지막 펄스, 이후 200 ms 동안 추가 펄스 수를 기록한다.
3. 같은 중립 절차로 재무장한 뒤 링크를 끊고 실제 KILLED reason과 무펄스를 기록한다. SA 허용 상태에서 링크만 복구했을 때 차단 유지, 중립 KILL 유지 후 새 허용 전환에서만 중립 펄스 재개를 확인한다.
4. **종료**는 SA KILL → armed=false·estop_latched=true 및 차단 관찰창 확보 → 모터 Ctrl+C → 제어기·수신기 Ctrl+C → bag Ctrl+C 후 저장 완료 → 입력 감시·상태 관찰 Ctrl+C 순서다. 종료 경로는 OE HIGH, 전체 FULL OFF, SLEEP을 요청한 뒤 장치를 닫는다. 기한이 먼저 만료되면 해당 시각을 기록한다. 물리 차단 시간은 위의 외부 동시 계측 방법으로 측정한다.

### 2026-11-25 RC 연동 고정 중립 벤치 결과

사용 기록은 `rc_neutral_edges_20261125_190749.log`, `rc_neutral_edges_20261125_190749.log.times`, `rc_fixed_neutral_20261125_190719_0.db3`이다.

| 항목 | 판정 | 관측 근거 |
|---|---|---|
| 실제 양쪽 중립 PWM 발생 | 통과 | CH0·CH1에서 약 20.02 ms 주기의 완전한 펄스를 각각 1,078개 관측했다. 폭 중앙값은 각각 1498.725 µs와 1496.391 µs였다. |
| SA KILL 후 양쪽 반복 펄스 차단 | 통과 | CH5가 1792→191로 바뀌고 제어기가 RUNNING→KILLED (`radio_kill`)를 기록했다. CH5 전환 뒤 양쪽에 완전한 펄스가 하나씩 더 있었다. 마지막 하강 에지는 각각 19:10:08.991520, 19:10:08.991535 KST였다. 이후 GPIO 기록 종료까지 각각 약 62.582초 동안 추가 에지 0개였고 기록기 종료코드는 0이었다. |
| 개별 내부 펄스 폭 1475~1525 µs | 실패 | CH0에서 22개, CH1에서 32개 이탈했다. 비영점 명령 구간에서도 각각 6개와 7개가 이탈했다. |

CH0 상승 에지마다 가장 가까운 CH1 상승 에지를 찾아 중복·누락 없이 1,078쌍을 대응시켰다. CH1 상승은 CH0보다 14.000~33.630 µs 늦게 기록됐다. 첫 쌍과 끝 쌍을 제외한 내부 1,076쌍 중 양쪽 모두 범위 안은 1,043쌍, 양쪽 모두 짧음은 10쌍, 양쪽 모두 김은 11쌍, 한쪽만 이탈은 12쌍(CH0만 김 1쌍, CH1만 짧음 11쌍), 반대 방향 이탈은 0쌍이었다.

아래 번호는 첫 완전한 펄스 쌍을 1로 센 값이다. 에지 차이는 CH1 시각에서 CH0 시각을 뺀 값이고, 직전·직후 간격은 해당 상승 에지를 기준으로 했다. 모든 값의 단위는 µs이다.

| 쌍 | 폭 CH0 / CH1 | 상승 차이 | 하강 차이 | CH0 직전 / 직후 간격 | CH1 직전 / 직후 간격 |
|---|---|---:|---:|---|---|
| 756 (양쪽 최단 폭) | 1448.030 / 1447.882 | +17.648 | +17.500 | 20069.619 / 19974.080 | 20063.526 / 19978.321 |
| 771 (양쪽 최장 폭) | 1565.105 / 1563.364 | +20.111 | +18.370 | 20015.136 / 20022.118 | 20008.858 / 20022.618 |

756번에서 CH0의 직전·직후 간격은 주기 중앙값 대비 +49.383 / −46.156 µs, CH1은 +43.920 / −41.285 µs로 대부분 상쇄됐다. 771번에서도 편차 부호가 바뀌었지만, 두 간격 합과 중앙값 두 배의 차이는 CH0 −3.218 µs, CH1 −7.736 µs였다. **폭 이탈 원인은 미확정**이다. Pi 되읽기만으로는 실제 파형 변동과 에지 기록 지연이 각각 얼마나 기여했는지 구분할 수 없다. `oe_low_requested`는 소프트웨어 요청값이며 실제 OE 단자 전압과 물리 스위치 조작부터의 차단 시간은 미측정이다.

### 2026-11-25 RC 연동 고정 중립 PWM 최종 보완 시험

**누적 판정은 재무장·차단 기능 4개 항목 PASS, 개별 PWM 펄스 폭 품질 FAIL이다.** 이번 실행에서는 RF 단절 전에 최초 무장이 이루어지지 않았다. 단절 당시 이미 출력이 차단돼 있었으므로, 발생 중인 PWM의 RF 단절 차단은 앞선 재시험의 실측 결과로 판정했다.

| 실행 | ROS bag | GPIO 에지 | 기록기 시각 |
|---|---|---|---|
| 최종 보완 시험 | [rc_rearm_final_20261125_200422_0.db3](/home/ubuntu/rc_rearm_final_20261125_200422/rc_rearm_final_20261125_200422_0.db3) | [rc_rearm_final_edges_20261125_200331.log](/home/ubuntu/rc_rearm_final_edges_20261125_200331.log) | [같은 실행의 .log.times](/home/ubuntu/rc_rearm_final_edges_20261125_200331.log.times) |
| RF 단절 차단을 확인한 앞선 재시험 | [rc_rf_loss_bench_retry_20261125_194125_0.db3](/home/ubuntu/rc_rf_loss_bench_retry_20261125_194125/rc_rf_loss_bench_retry_20261125_194125_0.db3) | [rc_rf_loss_edges_20261125_194414.log](/home/ubuntu/rc_rf_loss_edges_20261125_194414.log) | [같은 실행의 .log.times](/home/ubuntu/rc_rf_loss_edges_20261125_194414.log.times) |

아래 시각은 KST이며 ROS 사건은 bag 수신 시각을 사용했다. 최종 시험의 GPIO 기록 구간(20:03:31.570~20:12:58.806)은 bag 기록 구간(20:04:25.537~20:12:48.409)을 포함한다. GPIO 에지의 단조시각은 `.log.times` 시작 시계쌍으로 환산했다. 종료 시계쌍에서 구한 오프셋과의 차이는 0.574 µs였고 기록기 종료코드는 0이었다.

#### 재무장·차단 기능

| 항목 | 누적 판정 | 실측 근거 |
|---|---|---|
| 1. RF 단절 시 양쪽 PWM 차단 | PASS (앞선 재시험) | 앞선 재시험에서 무장 후 CH0·CH1에 완전한 펄스가 각각 537개 발생했다. 마지막 RC는 19:47:08.235, 양쪽 마지막 에지는 19:47:08.538이었다. 이어 제어기가 RUNNING→KILLED(`rc_timeout`)로 전환했고 19:47:08.586에 미무장 상태가 수신됐다. 최종 시험에서는 최초 CH5 허용(20:10:48.192)이 모터 기동보다 빨랐고 단절 전부터 미무장·무펄스였다. 이 실행의 `rc_timeout`(20:11:03.225)은 KILLED→KILLED였으므로 이 항목은 이번 실행 단독으로는 판정 보류다. |
| 2. RF 복구 후 자동 재무장 방지 | PASS | RC가 20:11:21.367, 유효 LQ가 20:11:21.447에 복귀했다. CH5 허용 상태에서 다음 SA 조작까지 12.674초 동안 양쪽 추가 에지는 0개였다. 이 구간에서 미무장 상태 24건이 발행됐고 첫·마지막 표본 사이는 12.102초였다. |
| 3. 새 SA KILL→허용 후 양쪽 PWM 재개 | PASS | CH5가 20:11:34.121에 KILL(191), 20:11:57.331에 허용(1792)으로 전환됐다. 첫 상승 에지는 CH0 20:11:58.391957, CH1 20:11:58.391982였다. 20:11:58.402에 `armed=true`, `oe_low_requested=true`를 수신했고 이 펄스열에서 양쪽 각각 444개의 완전한 펄스를 관측했다. |
| 4. 마지막 SA KILL 후 재차단 및 1초 이상 추가 에지 없음 | PASS | CH5 KILL은 20:12:07.251163, 마지막 하강 에지는 CH0 20:12:07.256307, CH1 20:12:07.256328이었다. 20:12:07.263에 RUNNING→KILLED(`radio_kill`), 20:12:07.313에 `armed=false`, `oe_low_requested=false`를 수신했다. 양쪽 모두 마지막 에지부터 모터의 `finished` 상태까지 29.853초 동안 추가 에지 0개였다. GPIO 기록 종료까지의 전체 관찰 시간은 51.550초였다. |

모터 기동 로그는 20:10:50.023이었다. 기동 기준 180초 예상 만료는 20:13:50.023이고, 마지막 SA KILL은 기동 후 77.228초에 발생했다. 1초 차단 조건은 모터가 미무장 상태를 계속 발행하던 동안 충족됐다. `finished`는 기동 후 107.086초인 20:12:37.109에 기록됐으며 **종료 원인은 미확정**이다.

#### 개별 PWM 펄스 폭 품질과 후속 개발

**개별 폭 1475~1525 µs 기준은 FAIL이다.** 각 채널에서 상승·하강 쌍으로 완전한 펄스 444개를 측정했다. 아래 폭 통계는 전체 펄스 기준이고, 이탈 수는 첫·끝 펄스를 제외한 내부 442개 기준이다.

| 채널 | 주기 중앙값 (ms) | 폭 최소 / 중앙 / 최대 (µs) | 내부 펄스 범위 이탈 |
|---|---:|---|---:|
| CH0 (BCM24) | 20.008 | 1436.698 / 1497.078 / 1525.475 | 6 / 442 |
| CH1 (BCM25) | 20.008 | 1440.271 / 1494.938 / 1523.827 | 10 / 442 |

폭 이탈 원인은 미확정이다. 다음 개발 단계는 PCA의 CH0·CH1 출력을 외부 로직 분석기 또는 오실로스코프로 Pi 에지 기록과 동시에 측정하고, 이탈이 발생한 같은 주기의 상승·하강 시각과 폭을 대조하는 것이다. 외부 계측에서도 이탈이 재현되는지 확인한 뒤 출력 파형과 Pi 기록 경로 중 조사할 대상을 정한다. `oe_low_requested`는 소프트웨어 요청값이다. 실제 OE 단자 전압과 물리적인 송신기 OFF 또는 SA 조작부터 양쪽 출력 차단까지의 시간은 미측정이다.

### 2026-11-25 고정 중립 PWM 품질과 CRSF 수신·bag 부하 비교

추진 배터리와 양쪽 ESC 신호선을 분리한 조건에서 같은 Pi와 설치된 `bench_pca`를 사용했다. 매회 PRESCALE 0x8A와 CH0·CH1 각 307카운트를 되읽고 BCM24·25의 양쪽 에지를 같은 `gpiomon` 형식으로 기록했다. 중앙 30초에 상승한 완전한 펄스의 폭을 집계했다. A는 고정 중립 출력과 계측, C는 A에 `boat_rc drive`만 추가, B는 C에 두 RC 토픽의 ROS bag 기록을 추가한 조건이다. [실행기](../../pwm_ab/run.py) · [분석기](../../pwm_ab/analyze.py) · [회차별 원본](../../pwm_ab/results/)

| 조건 (각 4회) | CH0 범위 밖 / 전체 | CH1 범위 밖 / 전체 | 평균 CPU | 평균 시스템 IRQ/s | 양쪽 폭 상관계수 범위 |
|---|---:|---:|---:|---:|---:|
| A · 계측 | 14/5,996 (0.23%) | 42/5,996 (0.70%) | 18.9% | 3,568 | 0.810~0.881 |
| C · drive | 31/5,995 (0.52%) | 106/5,995 (1.77%) | 약 32.3% | 4,629 | 0.796~0.879 |
| B · drive와 bag | 84/5,993 (1.40%) | 132/5,993 (2.20%) | 53.1% | 5,723 | 0.756~0.907 |

C1~C4에서 CH0/CH1 이탈 수는 각각 6/25, 13/33, 5/25, 7/23개였다. 4회 모두 짝·주기 간격 이상 0건, 차단 요청 후 1초 관찰에서 양쪽 추가 에지 0개, 기록기 정상 종료였다. 측정 창 양 끝에서 drive의 UART 점유를 확인했고 회차별 프로세스 읽기 카운터 증가량은 195,128~195,310바이트였다. 이 값은 프로세스 전체 읽기 활동을 나타낸다. C의 개별 RC 프레임 및 LQ 유효성은 별도 구독이나 bag으로 측정하지 않았다. 기존 A/B 원본은 시험 전후 파일 내용 해시가 일치했다.

**원인별 판단**

- **수신 부하.** C−A의 이탈률 차이는 CH0 +0.28%p, CH1 +1.07%p였다. UART 공유 IRQ 행(`uart-pl011`)의 회차 평균은 A 0회/s, C와 B 모두 약 500회/s였다. 수신 경로 활성화와 폭 이탈 증가가 함께 나타났고, CH1에서 증가폭이 컸다. UART, 파싱, ROS 발행 각각의 기여는 분리되지 않았다.
- **bag 추가 부하.** B−C의 차이는 CH0 +0.88%p, CH1 +0.44%p였다. CPU 사용률과 전체 IRQ도 증가했다. MMC 공유 IRQ 행의 평균은 A/C/B에서 각각 555/541/539회/s였다. 저장장치 IRQ 횟수 증가로 이 차이를 설명할 근거는 부족하다. DDS 전달·직렬화·기록기 처리·저장 작업의 개별 기여는 미확정이다.
- **측정 경로와 출력.** 동일한 고정 레지스터 설정에서 부하가 큰 조건일수록 기록 폭의 이탈률이 높았다. GPIO 에지 시각 기록 지연 가설과 일치하지만 세 조건 모두 같은 Pi 입력 경로를 썼고, 상관계수는 부하 순서대로 증가하지 않았다. 실제 PCA 파형과 기록 지연의 분담은 미확정이다. 개별 폭 품질은 FAIL이며 기존 재무장·차단 기능 4개 PASS를 유지한다.
- **비교 한계와 최소 대조.** 모든 회차의 부팅 시각이 같고 마지막 B 측정 창 종료에서 첫 C 시작까지 약 3분 42초였다. 다만 C 네 회차를 A/B 뒤에 연속 실행해 시간대·온도·백그라운드 부하의 영향이 섞일 수 있다. 조건 평균은 기술적으로 비교 가능하며 차이를 인과 효과로 확정하기에는 제한이 있다. 순서 효과를 확인할 최소 후속 시험은 새 결과 디렉터리에서 같은 세션에 A→C→B, B→C→A 두 블록(조건별 2회)을 실행하는 것이다.

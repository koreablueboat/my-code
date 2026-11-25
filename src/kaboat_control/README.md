# v3.4 현장 PREVIEW 제어

`ros2 run kaboat_control control`은 /rc_channels, /rc_uplink_lq, /boat/motor_status와
AUTO 후보 /boat/auto/cmd_vel (Twist)을 구독합니다. /boat/cmd_vel은 이 노드만 발행합니다.
기본 `ros2 run kaboat_motor boat_motor`는 양방향 PREVIEW입니다.

워크스페이스 설치본을 로드한 뒤 제어기가 기대할 백엔드를 시작 시 선택합니다.

```bash
source ~/kaboat_ws/install/setup.bash
ros2 run kaboat_control control --ros-args -p expected_backend:=pca9685
```

기본값은 `preview`이며 명시하려면 `-p expected_backend:=preview`를 사용합니다.
이 파라미터는 읽기 전용입니다. 모터는 별도로 자신의 백엔드를 보고하며,
상태의 `backend`가 기대값과 다르면 제어기는 KILL 상태를 유지합니다.

CH1 증가 방향은 우회전, CH2는 중앙 중립입니다. 후진·전진 한계는 −0.009와 +0.03,
회전 한계는 ±0.06입니다. 보정과 채널 매핑은 scripts/control_config.json에 있습니다.
CH6 SB 191은 STANDBY, 997은 MANUAL, 1792는 AUTO입니다. 중간의 미정 구간에서는 잠깁니다.

1. MANUAL 또는 AUTO를 선택하고 SA 위(191, KILL)에서 두 스틱을 중립으로 최소 1초 유지합니다.
   신선한 RC, LQ, 선택한 백엔드와 일치하는 준비된 양방향 모터 상태가 필요합니다. AUTO에서는 단일 후보 발행자가 중립을 계속 보내야 합니다.
2. SA를 아래(1792)로 전환하고 중립을 유지합니다. 제어기가 정지 입력 해제, 모터 reset,
   새 상태 확인, 0 명령 0.60초 유지, 모터 arm, 새 상태 확인을 순차 진행합니다.
3. KILL·모드 변경·입력 만료 뒤에는 처음부터 다시 진행합니다.
/boat/control/rearm (Trigger)는 단계와 준비 상태를 조회하며 RUNNING에서만 success입니다.
에이전트는 `/boat/auto/cmd_vel`에만 후보 명령을 발행합니다. AUTO 후보의 0.30초 유효시간은 수신 시각 기준입니다. 제어 노드 로그에는 상태 전이 사유와 마지막 AUTO 수신 시각(단조 시계 기준 초)이 기록됩니다.

ROS 서비스 호출 없이 백엔드 선택과 재무장을 검증하려면 설치본을 로드한 뒤
`colcon test --packages-select kaboat_control --ctest-args -R '^backend_selection$'`를 실행합니다.

PREVIEW 모터의 ROS arm 서비스까지 호출하는 기존 합성 입력 통합 시험은 워크스페이스 루트에서 install/setup.bash를 로드한 뒤
`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=117 python3 src/kaboat_control/test/preview_integration.py`로 실행합니다.

# RC 보트 개발 워크스페이스

ROS 2 Humble에서 `boat_rc`가 수신기의 채널과 링크 품질을 발행하고, `kaboat_control`이 RC 모드와 재무장 상태를 확인해 좌우 추진기를 제어한다. 통합 실행은 양방향 PREVIEW를 사용한다.

## 자율주행 에이전트 연결

에이전트 → `/boat/auto/cmd_vel` → `kaboat_control` → `/boat/cmd_vel` → `kaboat_motor`

에이전트 하나가 AUTO 후보 명령을 `geometry_msgs/msg/Twist`로 발행한다. 전진은 `linear.x` 0~0.03, 회전은 `angular.z` ±0.06이며 나머지 성분은 0이다. AUTO 재무장 동안 중립 명령을 유지하고 운행 중에는 수신 간격을 0.30초 이내로 유지한다. RC와 링크 품질도 계속 유효해야 한다. 최종 명령 토픽은 제어기가 단독 발행한다. [제어기 운용 방법](src/kaboat_control/README.md)

## 빌드와 검증

워크스페이스 루트에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

등록된 시험은 합성 UART와 모의 PCA로 실행한다. 현장 측정에는 [벤치 검증표](docs/records/bench-stop.md)를 사용하고, 작업 이력은 [개발 기록](docs/records/README.md)에 남긴다.

# boat_rc

`ros2 run boat_rc drive`는 RP3 CRSF UART(기본 /dev/ttyAMA1, 420000 baud)를 읽습니다.
유효 프레임마다 /rc_channels (std_msgs/msg/Int32MultiArray)에 CH1부터 CH16까지
원시값을 발행합니다. port, baud는 ROS 파라미터입니다. 새 프레임이 없으면 발행을 멈춥니다.
송신기 단절 시 수신기가 이전 값을 계속 전송하는 경우에는 프레임 시간만으로 단절을 식별할 수 없습니다.

CRC가 유효한 CRSF 0x14 링크 통계에서 uplink LQ를 /rc_uplink_lq (std_msgs/msg/UInt8)로 발행합니다.

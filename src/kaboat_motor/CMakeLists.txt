cmake_minimum_required(VERSION 3.8)
project(kaboat_motor)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    boat_motor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME boat_motor
)

install(
  FILES
    reverse_motor_core.py
    bench_pca.py
    t200_once.py
    motor_core.py
    pca_backend.py
    esc_config.py
    preview_forward_only.json
    preview_bidirectional.json
    manual_dual.py
    manual_dual_candidate.json
    left_water_candidate.json
    replacement_board_candidate.json
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS neutral_probe.py DESTINATION lib/${PROJECT_NAME} RENAME neutral_probe)
install(PROGRAMS left_t200_once.py DESTINATION lib/${PROJECT_NAME} RENAME left_t200_once)

install(PROGRAMS right_t200_once.py DESTINATION lib/${PROJECT_NAME} RENAME right_t200_once)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(motor_core test_motor_core.py)
  ament_add_pytest_test(pca_config test_pca_config.py)
  ament_add_pytest_test(neutral_probe_test test_neutral_probe.py)
  ament_add_pytest_test(left_t200_once_test test_left_t200_once.py)
  ament_add_pytest_test(manual_dual test_manual_dual.py)
  ament_add_pytest_test(left_water test_left_water.py)
  ament_add_pytest_test(rc_fixed_neutral test_rc_fixed_neutral.py)
endif()

ament_package()

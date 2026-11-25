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
    motor_core.py
    pca_backend.py
    esc_config.py
    preview_forward_only.json
    preview_bidirectional.json
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(motor_core test_motor_core.py)
  ament_add_pytest_test(pca_config test_pca_config.py)
endif()

ament_package()

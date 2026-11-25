cmake_minimum_required(VERSION 3.8)
project(kaboat_control)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/control.py DESTINATION lib/${PROJECT_NAME} RENAME control)
install(FILES scripts/supervisor.py scripts/control_config.json DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(backend_selection test/backend_selection.py
    ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=117 "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(preview_integration test/preview_integration.py
    TIMEOUT 180
    ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=117 "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
endif()

ament_package()

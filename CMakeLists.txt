cmake_minimum_required(VERSION 3.8)
project(rsf_simulator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(clock_throttle src/clock_throttle.cpp)
target_link_libraries(clock_throttle rclcpp::rclcpp ${rosgraph_msgs_TARGETS})

install(TARGETS clock_throttle DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch worlds models config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/rsf_device_emulator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rsf_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(clock_throttle src/clock_throttle.cpp)
target_link_libraries(clock_throttle rclcpp::rclcpp ${rosgraph_msgs_TARGETS})

add_executable(rsf_device_emulator src/rsf_device_emulator.cpp)
target_link_libraries(rsf_device_emulator
  rclcpp::rclcpp ${nav_msgs_TARGETS} ${nmea_msgs_TARGETS} ${sensor_msgs_TARGETS})

install(TARGETS clock_throttle rsf_device_emulator DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch worlds models config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gnss_geo test/test_gnss_geo.cpp)
  target_include_directories(test_gnss_geo PRIVATE src)
endif()

ament_package()

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

set(MESH_RELEASE_URL https://github.com/kyo0221/rsf_simulator/releases/download/tc-2026)
set(MESH_SHA256_tsukuba 65643659c174120fe98666881f849e66501f4f271f4c7ea38973c2409cd8e5df)
set(MESH_SHA256_tsudanuma 3e34ed74d2060387f21e6e944c8c6e98e32a174ddc376971545ad129ed3fe087)
foreach(world tsukuba tsudanuma)
  set(archive ${CMAKE_CURRENT_BINARY_DIR}/${world}_meshes.tar.gz)
  set(extract_dir ${CMAKE_CURRENT_BINARY_DIR}/meshes/${world})
  file(DOWNLOAD ${MESH_RELEASE_URL}/${world}_meshes.tar.gz ${archive}
    EXPECTED_HASH SHA256=${MESH_SHA256_${world}} SHOW_PROGRESS)
  if(NOT EXISTS ${extract_dir}/meshes)
    file(ARCHIVE_EXTRACT INPUT ${archive} DESTINATION ${extract_dir})
  endif()
  install(DIRECTORY ${extract_dir}/meshes DESTINATION share/${PROJECT_NAME}/models/${world})
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gnss_geo test/test_gnss_geo.cpp)
  target_include_directories(test_gnss_geo PRIVATE src)
endif()

ament_package()

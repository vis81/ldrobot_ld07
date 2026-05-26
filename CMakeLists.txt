cmake_minimum_required(VERSION 3.8)
project(ldrobot_ld07)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ldrobot_ld07_node
  src/serial_port.cpp
  src/ld07_driver.cpp
  src/ld07_node.cpp
  src/main.cpp
)

target_include_directories(ldrobot_ld07_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ldrobot_ld07_node rclcpp sensor_msgs)

install(TARGETS ldrobot_ld07_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

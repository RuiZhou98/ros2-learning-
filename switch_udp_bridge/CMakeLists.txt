cmake_minimum_required(VERSION 3.8)
project(switch_udp_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(switch_interfaces REQUIRED)

add_library(${PROJECT_NAME}_core
  src/crc16.cpp
  src/protocol.cpp
  src/udp_socket.cpp
  src/tx_scheduler.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  switch_interfaces
)

add_executable(udp_bridge_node
  src/main.cpp
  src/udp_bridge_node.cpp
)

target_include_directories(udp_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(udp_bridge_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(udp_bridge_node
  rclcpp
  std_msgs
  switch_interfaces
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    udp_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
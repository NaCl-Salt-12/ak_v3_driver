cmake_minimum_required(VERSION 3.8)
project(ak_v3_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- custom message/service interfaces defined by this package ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "srv/SetOrigin.srv"
  "srv/SetFeedbackConfig.srv"
  DEPENDENCIES std_msgs
)
# NOTE: generating and consuming custom interfaces from within the same
# ament_cmake package (rather than a separate *_msgs package) requires the
# rosidl_get_typesupport_target() pattern below, available on Humble and
# later. On Foxy this package would need to be split into
# ak_v3_driver_msgs (interfaces only) + ak_v3_driver (this node), built and
# sourced in that order.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- protocol library: no rclcpp dependency, unit-testable standalone ---
add_library(ak_v3_protocol
  src/protocol.cpp
  src/motor_limits.cpp
)
target_include_directories(ak_v3_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- SocketCAN transport library ---
add_library(ak_v3_can_socket
  src/can_socket.cpp
)
target_link_libraries(ak_v3_can_socket PUBLIC ak_v3_protocol)

# --- the node itself ---
add_library(ak_v3_driver_node_lib
  src/motor_driver_node.cpp
)
target_link_libraries(ak_v3_driver_node_lib PUBLIC
  ak_v3_protocol
  ak_v3_can_socket
  "${cpp_typesupport_target}"
)
ament_target_dependencies(ak_v3_driver_node_lib PUBLIC
  rclcpp
  std_srvs
  std_msgs
  diagnostic_msgs
)

add_executable(motor_driver_node src/motor_driver_node_main.cpp)
target_link_libraries(motor_driver_node PRIVATE ak_v3_driver_node_lib)
ament_target_dependencies(motor_driver_node PUBLIC rclcpp)

install(TARGETS
  ak_v3_protocol
  ak_v3_can_socket
  ak_v3_driver_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright/license linters until a real LICENSE header policy is
  # decided -- flip these back on once that's settled.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ak_v3_protocol)
endif()

ament_export_include_directories(include)
ament_export_libraries(ak_v3_protocol ak_v3_can_socket ak_v3_driver_node_lib)
ament_export_dependencies(rclcpp std_srvs std_msgs diagnostic_msgs rosidl_default_runtime)

ament_package()

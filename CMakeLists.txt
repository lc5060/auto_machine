cmake_minimum_required(VERSION 3.8)
project(auto_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutoMachineDef.msg"
  "msg/DeviceFault.msg"
  "srv/AutoMachine.srv"
  DEPENDENCIES std_msgs
)

# 服务端可执行文件
add_executable(auto_machine_node
  src/auto_machine_node.cpp
  src/auto_machine_modbus.cpp
  src/auto_machine_t1.cpp
  src/auto_machine_v6.cpp
  src/auto_machine_mag04.cpp
  src/auto_machine_mag06.cpp
  src/auto_machine_xyzr.cpp
  src/closed_loop.cpp
  src/fault_reporter.cpp
)
target_include_directories(auto_machine_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(auto_machine_node rclcpp)

# Jazzy/Humble 现代typesupport链接方式
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(auto_machine_node "${cpp_typesupport_target}")

install(TARGETS
  auto_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


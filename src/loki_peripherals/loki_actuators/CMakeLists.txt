cmake_minimum_required(VERSION 3.10)
project(loki_actuators)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(std_msgs    REQUIRED)
find_package(std_srvs    REQUIRED)
find_package(loki_msgs   REQUIRED)

add_executable(hw_bridge src/hw_bridge.cpp)

ament_target_dependencies(hw_bridge
  rclcpp
  std_msgs
  std_srvs
  loki_msgs
)

install(TARGETS hw_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

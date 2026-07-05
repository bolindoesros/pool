cmake_minimum_required(VERSION 3.10)
project(loki_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake  REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(std_srvs     REQUIRED)
find_package(nav_msgs     REQUIRED)
find_package(tf2          REQUIRED)

add_library(pid_lib STATIC src/pid.cpp)
target_include_directories(pid_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(loki_controller
  src/controller.cpp
  src/main.cpp)
target_include_directories(loki_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(loki_controller pid_lib)
ament_target_dependencies(loki_controller
  rclcpp std_msgs std_srvs nav_msgs tf2)

install(TARGETS loki_controller pid_lib
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

ament_package()

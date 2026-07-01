cmake_minimum_required(VERSION 3.8)
project(loki_cerulean)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Parser
add_library(tracker650_parser
  src/tracker650_parser.cpp
)

target_include_directories(tracker650_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Receiver node: talks to Tracker 650 over UDP and publishes raw sentences
add_executable(tracker650_receiver
  src/tracker650_receiver.cpp
)

target_include_directories(tracker650_receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tracker650_receiver
  rclcpp
  std_msgs
)

target_link_libraries(tracker650_receiver
  pthread
)

# Republisher node: parses raw DVL sentences and publishes TwistWithCovarianceStamped
add_executable(tracker650_republisher
  src/tracker650_republisher.cpp
)

target_include_directories(tracker650_republisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tracker650_republisher
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(tracker650_republisher
  tracker650_parser
)
# Install executables
install(TARGETS
  tracker650_parser
  tracker650_receiver
  tracker650_republisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Export include directory
ament_export_include_directories(include)

ament_package()
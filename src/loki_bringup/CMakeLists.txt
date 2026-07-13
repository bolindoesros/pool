cmake_minimum_required(VERSION 3.10)
project(loki_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY urdf/     DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes/   DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY scripts/  DESTINATION share/${PROJECT_NAME}/scripts)

install(PROGRAMS
  scripts/servo_test.py
  scripts/circle_test.py
  scripts/box_test.py
  scripts/teleop.py
  scripts/current_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

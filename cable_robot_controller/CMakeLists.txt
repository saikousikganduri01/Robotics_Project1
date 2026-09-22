cmake_minimum_required(VERSION 3.10)
project(cable_robot_controller)

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(cable_robot_controller SHARED src/CableSearchController.cc)
add_library(cable_robot_controller_twist SHARED src/TwistCmdController.cc)

# Keep output as libcable_robot_controller.so
set_target_properties(cable_robot_controller PROPERTIES
  OUTPUT_NAME cable_robot_controller)
set_target_properties(cable_robot_controller_twist PROPERTIES
  OUTPUT_NAME cable_robot_controller_twist)

target_link_libraries(cable_robot_controller
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)

target_link_libraries(cable_robot_controller_twist
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)

cmake_minimum_required(VERSION 3.5)
project(test_script)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  moveit
  moveit_ros_planning_interface
  control_msgs
)

add_executable(test_script src/test_script.cpp)
ament_target_dependencies(test_script  ${THIS_PACKAGE_DEPENDS})

install(TARGETS
  test_script
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

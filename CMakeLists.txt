cmake_minimum_required(VERSION 3.5)
project(urdf_robodog)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joint_state_node src/joint_state_node.cpp)
ament_target_dependencies(joint_state_node rclcpp geometry_msgs trajectory_msgs sensor_msgs)

install(TARGETS
  joint_state_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
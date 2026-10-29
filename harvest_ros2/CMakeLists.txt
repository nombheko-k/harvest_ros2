cmake_minimum_required(VERSION 3.8)
project(harvest_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Forward Kinematics Node
add_executable(fk_node src/kinematics_nodes/fk_node.cpp)
ament_target_dependencies(fk_node rclcpp sensor_msgs geometry_msgs)

# Inverse Kinematics Node
add_executable(ik_node src/kinematics_nodes/ik_node.cpp)
ament_target_dependencies(ik_node rclcpp geometry_msgs trajectory_msgs sensor_msgs)

# Trajectory Planner Executable
add_executable(trajectory_planner_node src/trajectory_nodes/trajectory_planner_node.cpp)
ament_target_dependencies(trajectory_planner_node rclcpp trajectory_msgs builtin_interfaces sensor_msgs)

# Feedforward gravity compensation
add_executable(gravity_compensation_node src/feedforward/gravity_compensation_node.cpp)
ament_target_dependencies(gravity_compensation_node rclcpp builtin_interfaces trajectory_msgs sensor_msgs std_msgs)

install(TARGETS
  fk_node
  ik_node
  trajectory_planner_node
  gravity_compensation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

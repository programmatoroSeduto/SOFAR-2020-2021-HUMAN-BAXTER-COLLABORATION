cmake_minimum_required(VERSION 3.0.2)
project(controller_baxter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
  moveit_msgs
  sensor_msgs
  controller_manager
  roslaunch
  trajectory_msgs
  tf2_ros
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  human_baxter_collaboration
  )

catkin_package()

include_directories(
${catkin_INCLUDE_DIRS}
)

add_executable(controller_baxter src/controller_baxter.cpp)
target_link_libraries(controller_baxter ${catkin_LIBRARIES})
add_dependencies(controller_baxter ${catkin_EXPORTED_TARGETS})

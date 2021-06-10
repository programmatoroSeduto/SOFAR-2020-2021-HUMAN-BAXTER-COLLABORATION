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
  message_generation
  )

catkin_python_setup()

add_service_files(DIRECTORY srv)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  moveit_msgs
  sensor_msgs
)

catkin_package()

include_directories(
${catkin_INCLUDE_DIRS}
)

add_executable(controller_baxter src/controller_baxter.cpp)
target_link_libraries(controller_baxter ${catkin_LIBRARIES})
add_dependencies(controller_baxter ${catkin_EXPORTED_TARGETS})

add_executable(baxter_at_home src/baxter_at_home.cpp)
target_link_libraries(baxter_at_home ${catkin_LIBRARIES})
add_dependencies(baxter_at_home ${catkin_EXPORTED_TARGETS})

add_executable(go_to_home src/go_to_home.cpp)
target_link_libraries(go_to_home ${catkin_LIBRARIES})
add_dependencies(go_to_home ${catkin_EXPORTED_TARGETS})

catkin_install_python( PROGRAMS scripts/task_manager.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

catkin_install_python( PROGRAMS scripts/debug.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

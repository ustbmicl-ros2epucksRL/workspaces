cmake_minimum_required(VERSION 3.0.2)
project(robosar_emergency_response)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  robosar_messages
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    robosar_messages
)

catkin_install_python(PROGRAMS
  scripts/response_goal_sequence_node.py
  scripts/deform_dispatch_bridge_node.py
  scripts/response_group_coordinator_node.py
  scripts/response_cmd_mux_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_stage0_response_deform_demo.sh
  scripts/start_stage0_response_tasks.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

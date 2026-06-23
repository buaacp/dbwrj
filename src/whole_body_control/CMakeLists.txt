cmake_minimum_required(VERSION 3.0.2)
project(whole_body_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/whole_body_pose_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

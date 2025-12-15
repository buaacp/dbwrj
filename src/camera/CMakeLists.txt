cmake_minimum_required(VERSION 3.0.2)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros
  tf2
  tf2_geometry_msgs
  tf2_ros
  message_generation
)

add_message_files(
  FILES
  neighbor_camera_data.msg
  pixel_msg.msg
  pos_message.msg
  servo_msg.msg
  gimbal_msg.msg
  detect_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  #CATKIN_DEPENDS include
  CATKIN_DEPENDS  geometry_msgs mavros roscpp rospy std_msgs tf2 tf2_geometry_msgs tf2_ros message_runtime
  INCLUDE_DIRS include/camera
)

include_directories(
  include/camera
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/pos_analyze.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/position_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS
  scripts/gimbal_target.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/cube_position_D435i.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/position_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
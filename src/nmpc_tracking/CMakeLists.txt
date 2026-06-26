cmake_minimum_required(VERSION 3.0.2)
project(nmpc_tracking)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  mavros_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/run_dual_mpc_pregrasp.py
  scripts/test_px4_rate_thrust_interface.py
  scripts/test_arm_velocity_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

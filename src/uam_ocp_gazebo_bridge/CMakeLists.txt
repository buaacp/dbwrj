cmake_minimum_required(VERSION 3.0.2)
project(uam_ocp_gazebo_bridge)
find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()
catkin_install_python(PROGRAMS scripts/g1_static_hold_runner.py scripts/analyze_g1_results.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/run_g1_static_hold.sh scripts/analyze_g1_results.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

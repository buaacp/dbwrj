cmake_minimum_required(VERSION 3.0.2)
project(uav_bulb_camera_pose)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  diagnostic_msgs
  cv_bridge
  image_transport
  gazebo_msgs
)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/bundle_pose_node.py
  scripts/capture_pnp_debug_frame.py
  scripts/check_bundle_geometry_consistency.py
  scripts/image_impairment_node.py
  scripts/pose_evaluator.py
  scripts/run_offline_regression.py
  sim/scripts/generate_tag_textures.py
  sim/scripts/generate_scene_from_yaml.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch sim test results
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

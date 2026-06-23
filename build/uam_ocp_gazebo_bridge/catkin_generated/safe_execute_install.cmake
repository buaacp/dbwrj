execute_process(COMMAND "/home/zlhq/px4_fly_ws/build/uam_ocp_gazebo_bridge/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/zlhq/px4_fly_ws/build/uam_ocp_gazebo_bridge/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

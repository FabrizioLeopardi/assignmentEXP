execute_process(COMMAND "/root/Desktop/ros_ws/build/rosbot_description/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/root/Desktop/ros_ws/build/rosbot_description/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

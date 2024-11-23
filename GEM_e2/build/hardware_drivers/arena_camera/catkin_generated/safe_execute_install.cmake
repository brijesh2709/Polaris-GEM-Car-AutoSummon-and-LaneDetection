execute_process(COMMAND "/home/gem/cuberts_484/build/hardware_drivers/arena_camera/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/gem/cuberts_484/build/hardware_drivers/arena_camera/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

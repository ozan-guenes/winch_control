cmake_minimum_required(VERSION 3.0.2)
project(winch_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  rospy
  std_msgs
  # message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES 
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES std_msgs
# )

catkin_package(
  CATKIN_DEPENDS dynamixel_sdk rospy std_msgs #message_runtime
)

## Tell CMake about your Python “include” dir so it’s on PYTHONPATH
include_directories(
  ${catkin_INCLUDE_DIRS}
  # ${CMAKE_CURRENT_SOURCE_DIR}/src 
)

# ## Alternative to setuptools (setup.py)
# ## Install the Python module under src/winch_control
# install(DIRECTORY src/winch_control
#   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )

catkin_install_python(PROGRAMS
  scripts/winch_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dcsl_messages)
# Load catkin and all dependencies required for this package
set(CATKIN_DEPS geometry_msgs std_msgs genmsg message_runtime message_generation)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


add_message_files(
  DIRECTORY msg
  FILES belugaInput.msg BelugaArray.msg TwistArray.msg State.msg StateArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)


catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPS}
)

include_directories(${catkin_INCLUDE_DIRS})


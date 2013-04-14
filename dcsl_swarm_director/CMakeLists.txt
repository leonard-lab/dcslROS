# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dcsl_swarm_director)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS rospy sensor_msgs geometry_msgs dcsl_messages std_msgs)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# Install python scripts using distutils
catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS ${CATKIN_DEPS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/dcsl_swarm_director_node.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
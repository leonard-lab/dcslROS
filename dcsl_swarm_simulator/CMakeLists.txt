# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dcsl_swarm_simulator)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS geometry_msgs std_msgs dcsl_messages rospy dcsl_beluga_main)
set(SYS_DEPS python-numpy python-scipy)
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

## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS  ${SYS_DEPS}
    CATKIN_DEPENDS ${CATKIN_DEPS}
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

# include_directories(include ${catkin_INCLUDE_DIRS})

install(PROGRAMS src/dcsl_swarm_simulator/beluga_sim.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
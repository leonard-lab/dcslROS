# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dcsl_beluga_main)

# Load catkin and all dependencies required for this package
set(SYS_DEPS numpy)
set(CATKIN_DEPS rospy)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

catkin_python_setup()

## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS ${SYS_DEPS}
    CATKIN_DEPENDS ${CATKIN_DEPS}
    INCLUDE_DIRS 
    LIBRARIES
)



install(PROGRAMS src/dcsl_beluga_main/beluga.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

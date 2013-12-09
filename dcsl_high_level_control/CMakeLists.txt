# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dcsl_high_level_control)

# Load catkin and all dependencies required for this package
#set(CATKIN_DEPS roscpp geometry_msgs dcsl_messages)
set(CATKIN_DEPS dcsl_messages geometry_msgs dynamic_reconfigure rospy)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/dcsl_miabot_high_level_control_config.cfg
    )

## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
#catkin_package(
#    DEPENDS roscpp geometry_msgs dcsl_messages
#    CATKIN_DEPENDS # TODO
#    INCLUDE_DIRS # TODO include
#    LIBRARIES # TODO
#)
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS ${CATKIN_DEPS}
    INCLUDE_DIRS 
    LIBRARIES
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_library(dcsl_miabot_high_level_math src/dcsl_miabot_high_level_math.cc)
#add_executable(dcsl_miabot_follower_control src/dcsl_miabot_follower_control.cc)
#target_link_libraries(dcsl_miabot_follower_control dcsl_miabot_high_level_math ${catkin_LIBRARIES})

#install(TARGETS dcsl_miabot_follower_control
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS src/dcsl_high_level_control/dcsl_miabot_high_level_control.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

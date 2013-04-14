# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dcsl_vision_tracker)

# Load catkin and all dependencies required for this package
set(SYS_DEPS opencv2 python-munkres)
set(CATKIN_DEPS sensor_msgs cv_bridge rospy std_msgs rosunit dynamic_reconfigure roscpp)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# catkin_add_nosetests(test/test_dcsl_vision_tracker.py)

catkin_python_setup()

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/dcsl_beluga_tracker_config.cfg cfg/dcsl_miabot_tracker_config.cfg)

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

install(PROGRAMS scripts/background_generator.py scripts/dcsl_beluga_tracker.py scripts/dcsl_miabot_tracker.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
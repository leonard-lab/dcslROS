# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dcsl_vision_tracker)

# Load catkin and all dependencies required for this package
set(SYS_DEPS opencv2 python-munkres)
set(CATKIN_DEPS sensor_msgs cv_bridge rospy std_msgs rosunit dynamic_reconfigure roscpp genmsg actionlib actionlib_msgs image_transport cv_bridge)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# Add action files
add_action_files(
  DIRECTORY action
  FILES ToggleTracking.action ToggleWindow.action GenerateBackground.action
)

# catkin_add_nosetests(test/test_dcsl_vision_tracker.py)

catkin_python_setup()

generate_messages(
  DEPENDENCIES actionlib_msgs
)

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

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(dcsl_tracking_overlay src/dcsl_tracking_overlay.cpp)
set_target_properties(dcsl_tracking_overlay PROPERTIES COMPILE_FLAGS -std=c++0x)
target_link_libraries(dcsl_tracking_overlay ${catkin_LIBRARIES})

install(TARGETS dcsl_tracking_overlay
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS src/dcsl_vision_tracker/background_generator.py src/dcsl_vision_tracker/dcsl_beluga_tracker.py src/dcsl_vision_tracker/dcsl_miabot_tracker.py src/dcsl_vision_tracker/dcsl_vision_tracker_API.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

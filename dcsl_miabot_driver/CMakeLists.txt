# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dcsl_miabot_driver)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS message_generation message_runtime std_msgs geometry_msgs rospy roscpp genmsg actionlib actionlib_msgs)
set(SYS_DEPS python-bluez python-sys)
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
  FILES ConnectMiabot.action
)

# Install python scripts using distutils
catkin_python_setup()


# Generate action messages
generate_messages(
  DEPENDENCIES actionlib_msgs
)

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

add_executable(dcsl_miabot_joy src/dcsl_miabot_joy.cpp)
target_link_libraries(dcsl_miabot_joy ${catkin_LIBRARIES})

install(TARGETS dcsl_miabot_joy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS src/dcsl_miabot_driver/dcsl_miabot_driver.py src/dcsl_miabot_driver/dcsl_miabot_API.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

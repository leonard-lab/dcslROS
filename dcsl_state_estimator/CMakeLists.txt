# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dcsl_state_estimator)

# Load catkin and all dependencies required for this package
set(SYS_DEPS eigen)
set(CATKIN_DEPS dcsl_messages roscpp geometry_msgs std_msgs rospy)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

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

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(dcsl_miabot_estimator_math src/dcsl_miabot_estimator_math.cc)
add_executable(dcsl_miabot_state_estimator src/dcsl_miabot_state_estimator.cc)
target_link_libraries(dcsl_miabot_state_estimator dcsl_miabot_estimator_math ${catkin_LIBRARIES})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gtest(miabot_estimator_math_test test/miabot_estimator_math_test.cc)
target_link_libraries(miabot_estimator_math_test dcsl_miabot_estimator_math ${catkin_LIBRARIES})
target_link_libraries(miabot_estimator_math_test gtest)

install(TARGETS dcsl_miabot_state_estimator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS src/dcsl_state_estimator/estimate_by_planar_passthrough.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

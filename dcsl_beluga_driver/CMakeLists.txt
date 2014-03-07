# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dcsl_beluga_driver)

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS roscpp std_msgs sensor_msgs dcsl_messages)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(dcsl_beluga_joy src/dcsl_beluga_joy.cpp)
target_link_libraries(dcsl_beluga_joy ${catkin_LIBRARIES})

add_executable(dcsl_beluga_joy_sysID src/dcsl_beluga_joy_sysID.cpp)
target_link_libraries(dcsl_beluga_joy_sysID ${catkin_LIBRARIES})

install(TARGETS dcsl_beluga_joy dcsl_beluga_joy_sysID
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

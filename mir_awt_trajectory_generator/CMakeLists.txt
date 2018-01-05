cmake_minimum_required(VERSION 2.8.3)
project(mir_awt_trajectory_generator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  #kdl_parser
  roscpp
  rospy
  interactive_markers
  std_msgs
)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

#if(NOT Eigen_FOUND)
 # include(${PROJ_SOURCE_DIR}/cfg/FindEigen3.cmake)
  #set(Eigen_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
#endif()

pkg_search_module(Eigen3 REQUIRED eigen3)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs kdl_parser roscpp rospy std_msgs
 #DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(awt_trajectory_generator
    src/awt_trajectory_generator.cpp
    scripts/awt_trajectory_generator_node.cpp
)

target_link_libraries(awt_trajectory_generator
  ${catkin_LIBRARIES}
)

add_dependencies(awt_trajectory_generator
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS awt_trajectory_generator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)


## Add gtest based cpp test target and link libraries
#rosbuild_add_gtest(test/trajectory_generator_unittest test/trajectory_generator_unittest.cpp)
#catkin_add_gtest(trajectory_generator_unittest test/trajectory_generator_unittest.cpp)
#if(TARGET ${PROJECT_NAME}-test)
  #target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
#endif()

add_executable(trajectory_generator_unittest src/awt_trajectory_generator.cpp test/trajectory_generator_unittest.cpp)

target_link_libraries(
  trajectory_generator_unittest
  gtest
  gtest_main
)
add_test( trajectory_generator_unittest trajectory_generator_unittest )

## Add folders to be run by python nosetests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/velocity_commands_test.py)
  #target_link_libraries(velocity_commands_test ${catkin_LIBRARIES})
endif()

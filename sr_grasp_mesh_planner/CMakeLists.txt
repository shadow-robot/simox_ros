cmake_minimum_required(VERSION 2.8.3)
project(sr_grasp_mesh_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  actionlib
  pcl_ros
  roscpp
  rospy
  rostest
  sr_robot_msgs
  actionlib_msgs
  moveit_msgs
  sensor_msgs
  std_msgs
)

## Boost
find_package(Boost REQUIRED COMPONENTS system)

## Eigen
find_package(Eigen REQUIRED)


## Qt4
# http://www.briangoldfain.com/2013/02/using-qt-in-a-ros-catkin-package/
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

## Simox
find_package(Simox REQUIRED)
IF(NOT Simox_VISUALIZATION)
  message( FATAL_ERROR "Simox_VISUALIZATION is false, CMake will exit." )
ENDIF()
IF(NOT Simox_USE_COIN_VISUALIZATION)
  message( FATAL_ERROR "Simox_USE_COIN_VISUALIZATION is false, CMake will exit." )
ENDIF()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   actionlib_msgs#   std_msgs
# )

###################################
##      dynamic reconfigure      ##
###################################
generate_dynamic_reconfigure_options(
  cfg/Planner.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
  # LIBRARIES sr_grasp_mesh_planner
  CATKIN_DEPENDS roscpp rospy sr_robot_msgs actionlib_msgs sensor_msgs std_msgs
  DEPENDS eigen
)

###########
## Build ##
###########

include(${QT_USE_FILE})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}) # ui_GraspPlanner.h
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(SYSTEM ${Simox_INCLUDE_DIRS})
include_directories(SYSTEM ${Simox_BASE_DIR}/GraspPlanning)
include_directories(SYSTEM ${Simox_VISUALIZATION_INCLUDE_PATHS})

# Collect all required files for build
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/sr_grasp_mesh_planner/grasp_planner_window.hpp)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/grasp_planner.cpp src/grasp_planner_window.cpp src/grasp_action_server.cpp src/sr_approach_movement_bounding_box.cpp src/sr_approach_movement_surface_normal.cpp src/mesh_obstacle.cpp src/coin_viewer.cpp)

QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

## Declare a cpp executable
add_executable(sr_grasp_mesh_planner_qt
  ${QT_SOURCES}
  ${QT_FORMS_HPP}
  ${QT_MOC_HPP}
)

#add_executable(grasp_action_client_mesh
#  src/grasp_action_client_mesh.cpp
#  src/read_ply.cpp
#)

## Declare a cpp library
# add_library(sr_grasp_mesh_planner
#   src/${PROJECT_NAME}/sr_grasp_mesh_planner.cpp
# )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# http://answers.ros.org/question/62092/catkin_make-started-failing-to-generate-h-from-msg/
# http://answers.ros.org/question/52744/how-to-specify-dependencies-with-foo_msgs-catkin-packages/
add_dependencies(sr_grasp_mesh_planner_qt
  sr_robot_msgs_gencpp
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)
#add_dependencies(grasp_action_client_mesh
#  sr_robot_msgs_gencpp
#  ${catkin_EXPORTED_TARGETS}
#)

## Specify libraries to link a library or executable target against
target_link_libraries(sr_grasp_mesh_planner_qt
  ${QT_LIBRARIES}
  ${Simox_LIBRARIES}
  ${Simox_VISUALIZATION_LIBS}
  ${catkin_LIBRARIES}
)

#target_link_libraries(grasp_action_client_mesh
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS sr_grasp_mesh_planner sr_grasp_mesh_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
add_rostest_gtest(test_grasp_mesh_planner
  test/test_grasp_mesh_planner.test
  test/test_grasp_mesh_planner.cpp
  src/read_ply.cpp
)
target_link_libraries(test_grasp_mesh_planner
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)
add_dependencies(test_grasp_mesh_planner
  sr_robot_msgs_gencpp
  ${catkin_EXPORTED_TARGETS}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


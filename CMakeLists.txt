cmake_minimum_required(VERSION 2.8.3)
project(conformal_lattice_planner)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++14 -Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable)
add_compile_options(-std=c++14 -Wall -fmax-errors=3 -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp

  tf2
  tf2_ros
  tf2_geometry_msgs

  std_msgs
  geometry_msgs
  visualization_msgs
  image_transport

  actionlib
  actionlib_msgs

  message_generation
  message_runtime
)

# System dependencies are found with CMake's conventions
find_package(Boost 1.69 REQUIRED COMPONENTS timer)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Policy.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# Generate actions in the 'action' folder
add_action_files(
  FILES
  AgentPlan.action
  EgoPlan.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carla_ros_bridge
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  /home/ke/RosWorkSpace/self_driving_ws/src/carla/Examples/CppClient/libcarla-install/include
  /home/ke/RosWorkSpace/self_driving_ws/src/carla/Examples/CppClient/libcarla-install/include/system
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  /home/ke/RosWorkSpace/self_driving_ws/src/carla/Examples/CppClient/libcarla-install/lib
)

set(CARLA_LIBS
  carla_client
  rpc
  Recast
  Detour
  DetourCrowd
  DetourTileCache
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/carla_ros_bridge.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(all_planning
  src/conformal_lattice_planner/lane_follower.cpp
  src/conformal_lattice_planner/utils.cpp
  src/conformal_lattice_planner/loop_router.cpp
)
target_link_libraries(all_planning
  ${CARLA_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(all_planning
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_library(all_ros_nodes
  src/ros/simulator_node.cpp
  src/ros/random_traffic_node.cpp
  src/ros/no_traffic_node.cpp

  src/ros/ego_lane_following_node.cpp
  src/ros/agents_lane_following_node.cpp

  src/ros/convert_to_visualization_msgs.cpp
  src/conformal_lattice_planner/utils.cpp
)
target_link_libraries(all_ros_nodes
  all_planning
  ${CARLA_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(all_ros_nodes
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/carla_ros_bridge_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(random_traffic_node
  src/ros/nodes/random_traffic_simulator.cpp
)
target_link_libraries(random_traffic_node
  all_planning
  all_ros_nodes
  ${CARLA_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(random_traffic_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(no_traffic_node
  src/ros/nodes/no_traffic_simulator.cpp
)
target_link_libraries(no_traffic_node
  all_planning
  all_ros_nodes
  ${CARLA_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(no_traffic_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(ego_lane_following_node
  src/ros/nodes/ego_lane_follower.cpp
)
target_link_libraries(ego_lane_following_node
  all_planning
  all_ros_nodes
  ${CARLA_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(ego_lane_following_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(agents_lane_following_node
  src/ros/nodes/agents_lane_follower.cpp
)
target_link_libraries(agents_lane_following_node
  all_planning
  all_ros_nodes
  ${CARLA_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(agents_lane_following_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#add_executable(agents_lane_following_node
#  src/ros/nodes/agents_lane_following_node.cpp
#  src/ros/convert_to_visualization_msgs.cpp
#  src/conformal_lattice_planner/utils.cpp
#)
#target_link_libraries(agents_lane_following_node
#  all_planning
#  ${CARLA_LIBS}
#  ${catkin_LIBRARIES}
#)
#add_dependencies(agents_lane_following_node
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_carla_ros_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
catkin_add_gtest(test_idm
  src/conformal_lattice_planner/tests/test_intelligent_driver_model.cpp
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

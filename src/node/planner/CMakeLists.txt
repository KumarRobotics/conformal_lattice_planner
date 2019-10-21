# Agents lane following node.
add_executable(agents_lane_following_node
  agents_lane_following_node.cpp
  ../common/convert_to_visualization_msgs.cpp
)
target_link_libraries(agents_lane_following_node
  routing_algos
  planning_algos
  ${Carla_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(agents_lane_following_node
  routing_algos
  planning_algos
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Ego lane following node.
add_executable(ego_lane_following_node
  ego_lane_following_node.cpp
  ../common/convert_to_visualization_msgs.cpp
)
target_link_libraries(ego_lane_following_node
  routing_algos
  planning_algos
  ${Carla_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(ego_lane_following_node
  routing_algos
  planning_algos
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Ego conformal lattice planning node
add_executable(ego_idm_lattice_planning_node
  ego_idm_lattice_planning_node.cpp
  ../common/convert_to_visualization_msgs.cpp
)
target_link_libraries(ego_idm_lattice_planning_node
  routing_algos
  planning_algos
  ${Carla_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(ego_idm_lattice_planning_node
  routing_algos
  planning_algos
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


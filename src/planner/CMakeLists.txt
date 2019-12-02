set(planner_srcs
  common/traffic_lattice.cpp
  common/traffic_manager.cpp
  common/snapshot.cpp
  common/utils.cpp
  common/vehicle_path.cpp
  common/traffic_simulator.cpp
  idm_lattice_planner/idm_lattice_planner.cpp
  spatiotemporal_lattice_planner/spatiotemporal_lattice_planner.cpp
  slc_lattice_planner/slc_lattice_planner.cpp
)

add_library(planning_algos
  ${planner_srcs}
)
target_link_libraries(planning_algos
  routing_algos
  ${Carla_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(planning_algos
  routing_algos
)

add_subdirectory(tests)

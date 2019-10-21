#file(GLOB_RECURSE router_srcs *.cpp)
set(router_srcs
  loop_router/loop_router.cpp
)

add_library(routing_algos
  ${router_srcs}
)
target_link_libraries(routing_algos
  ${Carla_LIBRARIES}
  ${Boost_LIBRARIES}
)

/*
 * Copyright 2020 Ke Sun
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <array>
#include <limits>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>

#include <planner/common/waypoint_lattice.h>
#include <node/simulator/fixed_scenario_node.h>

using namespace router;
using namespace planner;

namespace node {

void FixedScenarioNode::spawnVehicles() {

  // The start position.
  // TODO: This may be load from the ROS parameter server.
  std::array<double, 3> start_pt{0, 0, 0};

  // Find the available spawn point cloest to the start point.
  std::vector<CarlaTransform> spawn_points =
    map_->GetRecommendedSpawnPoints();
  CarlaTransform start_transform;
  double min_distance_sq = std::numeric_limits<double>::max();

  for (const auto& pt : spawn_points) {
    const double x_diff = pt.location.x - start_pt[0];
    const double y_diff = pt.location.y - start_pt[1];
    const double z_diff = pt.location.z - start_pt[2];
    const double distance_sq = x_diff*x_diff + y_diff*y_diff + z_diff*z_diff;
    if (distance_sq < min_distance_sq) {
      start_transform = pt;
      min_distance_sq = distance_sq;
    }
  }
  ROS_INFO_NAMED("carla_simulator", "Start waypoint transform\nx:%f y:%f z:%f",
      start_transform.location.x, start_transform.location.y, start_transform.location.z);

  // Start waypoint of the lattice.
  boost::shared_ptr<CarlaWaypoint> start_waypoint =
    fast_map_->waypoint(start_transform.location);

  boost::shared_ptr<WaypointLattice> waypoint_lattice=
    boost::make_shared<WaypointLattice>(start_waypoint, 150, 1.0, loop_router_);

  // Spawn the ego vehicle.
  // The ego vehicle is at 50m on the lattice, and there is an 100m buffer
  // in the front of the ego vehicle.
  boost::shared_ptr<const CarlaWaypoint> ego_waypoint =
    waypoint_lattice->front(start_waypoint, 50.0)->waypoint();
  if (!ego_waypoint) {
    throw std::runtime_error("Cannot find the ego waypoint on the traffic lattice.");
  }
  ROS_INFO_NAMED("carla_simulator", "Ego vehicle initial transform\nx:%f y:%f z:%f",
      ego_waypoint->GetTransform().location.x,
      ego_waypoint->GetTransform().location.y,
      ego_waypoint->GetTransform().location.z);

  if (!spawnEgoVehicle(ego_waypoint, 20, false)) {
    throw std::runtime_error("Cannot spawn the ego vehicle.");
  }
  ego_.speed() = 15.0;

  // Spawn agent vehicles.
  // Braking scenario.
  //{
  //  boost::shared_ptr<const CarlaWaypoint> agent_waypoint =
  //    waypoint_lattice->front(ego_waypoint, 30.0)->waypoint();
  //  if (!spawnAgentVehicle(agent_waypoint, 5.0, false)) {
  //    throw std::runtime_error("Cannot spawn an agent vehicle.");
  //  }
  //}

  // Lane merging scenario.
  {
    boost::shared_ptr<const CarlaWaypoint> agent_waypoint =
      waypoint_lattice->front(ego_waypoint, 20.0)->waypoint();
    if (!spawnAgentVehicle(agent_waypoint, 15.0, false)) {
      throw std::runtime_error("Cannot spawn an agent vehicle.");
    }
  }

  {
    boost::shared_ptr<const CarlaWaypoint> agent_waypoint =
      waypoint_lattice->rightBack(ego_waypoint, 20.0)->waypoint();
    if (!spawnAgentVehicle(agent_waypoint, 20.0, false)) {
      throw std::runtime_error("Cannot spawn an agent vehicle.");
    }
  }

  {
    boost::shared_ptr<const CarlaWaypoint> agent_waypoint =
      waypoint_lattice->rightFront(ego_waypoint, 20.0)->waypoint();
    if (!spawnAgentVehicle(agent_waypoint, 20.0, false)) {
      throw std::runtime_error("Cannot spawn an agent vehicle.");
    }
  }

  {
    boost::shared_ptr<const CarlaWaypoint> intermediate_waypoint =
      waypoint_lattice->rightFront(ego_waypoint, 25.0)->waypoint();
    boost::shared_ptr<const CarlaWaypoint> agent_waypoint =
      intermediate_waypoint->GetRight();
    if (!spawnAgentVehicle(agent_waypoint, 15.0, false)) {
      throw std::runtime_error("Cannot spawn an agent vehicle.");
    }
  }

  // Spawn the following camera of the ego vehicle.
  spawnCamera();
  return;
}

} // End namespace node.

int main(int argc, char** argv) {

  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  if(ros::console::set_logger_level(
        ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  node::FixedScenarioNodePtr sim =
    boost::make_shared<node::FixedScenarioNode>(nh);
  if (!sim->initialize()) {
    ROS_ERROR("Cannot initialize the CARLA simulator.");
  }

  ros::spin();
  return 0;
}

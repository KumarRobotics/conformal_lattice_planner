/*
 * Copyright [2019] [Ke Sun]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
    boost::make_shared<WaypointLattice>(start_waypoint, 100, 1.0, loop_router_);

  // Spawn the ego vehicle.
  // The ego vehicle is at 50m on the lattice, and there is an 100m buffer
  // in the front of the ego vehicle.
  boost::shared_ptr<const CarlaWaypoint> ego_waypoint =
    waypoint_lattice->rightFront(start_waypoint, 50.0)->waypoint();
  if (!ego_waypoint) {
    throw std::runtime_error("Cannot find the ego waypoint on the traffic lattice.");
  }
  ROS_INFO_NAMED("carla_simulator", "Ego vehicle initial transform\nx:%f y:%f z:%f",
      ego_waypoint->GetTransform().location.x,
      ego_waypoint->GetTransform().location.y,
      ego_waypoint->GetTransform().location.z);

  if (!spawnEgoVehicle(ego_waypoint, 25, false)) {
    throw std::runtime_error("Cannot spawn the ego vehicle.");
  }

  // Spawn agent vehicles.
  {
    boost::shared_ptr<const CarlaWaypoint> agent_waypoint =
      waypoint_lattice->front(ego_waypoint, 30.0)->waypoint();
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

  //{
  //  boost::shared_ptr<const CarlaWaypoint> agent_waypoint =
  //    waypoint_lattice->leftBack(ego_waypoint, 15.0)->waypoint();
  //  if (!spawnAgentVehicle(agent_waypoint, 25.0, false)) {
  //    throw std::runtime_error("Cannot spawn an agent vehicle.");
  //  }
  //}

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

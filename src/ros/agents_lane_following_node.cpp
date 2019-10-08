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

#include <string>
#include <vector>
#include <chrono>
#include <unordered_set>

#include <conformal_lattice_planner/vehicle_path.h>
#include <ros/agents_lane_following_node.h>
#include <conformal_lattice_planner/vehicle_speed_planner.h>
#include <conformal_lattice_planner/lane_follower.h>

using namespace router;
using namespace planner;

namespace carla {

bool AgentsLaneFollowingNode::initialize() {

  bool all_param_exist = true;

  std::string host = "localhost";
  int port = 2000;
  all_param_exist &= nh_.param<std::string>("host", host, "localhost");
  all_param_exist &= nh_.param<int>("port", port, 2000);

  // Get the world.
  ROS_INFO_NAMED("agents_lane_following_planner", "connect to the server.");
  client_ = boost::make_shared<CarlaClient>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  client_->GetWorld();

  // Start the action server.
  ROS_INFO_NAMED("agents_lane_following_planner", "start action server.");
  server_.start();

  ROS_INFO_NAMED("agents_lane_following_planner", "initialization finishes.");
  return all_param_exist;
}

void AgentsLaneFollowingNode::executeCallback(
    const conformal_lattice_planner::AgentPlanGoalConstPtr& goal) {

  ROS_INFO_NAMED("agents_lane_following_planner", "executeCallback()");

  // Update the carla world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  map_ = world_->GetMap();

  // Get the ego and agent policies.
  const std::pair<size_t, double> ego_policy = egoPolicy(goal);
  std::unordered_map<size_t, double> agent_policies = agentPolicies(goal);

  // Create the current snapshot.
  //std::printf("Create snapshot.\n");
  boost::shared_ptr<Snapshot> snapshot = createSnapshot(ego_policy, agent_policies);

  // Create Path planner.
  //std::printf("Find start waypoint for lattice.\n");
  std::vector<boost::shared_ptr<const WaypointNodeWithVehicle>>
    traffic_lattice_entries = snapshot->trafficLattice()->latticeEntries();

  const double range = snapshot->trafficLattice()->range() + 55.0;
  boost::shared_ptr<const CarlaWaypoint> waypoint = nullptr;
  for (const auto& node : traffic_lattice_entries) {
    if (node->distance() != 0.0) continue;
    waypoint = node->waypoint();
    break;
  }

  if (!waypoint) throw std::runtime_error("No node with distance 0.");

  //std::printf("Create lane follower path planner.\n");
  boost::shared_ptr<LaneFollower> path_planner =
    boost::make_shared<LaneFollower>(map_, waypoint, range, router_);

  // Create speed planner.
  //std::printf("Create speed planner.\n");
  boost::shared_ptr<VehicleSpeedPlanner> speed_planner =
    boost::make_shared<VehicleSpeedPlanner>();

  double dt = 0.05;
  nh_.param<double>("fixed_delta_seconds", dt, 0.05);

  for (const auto& item : snapshot->agents()) {
    const Vehicle& agent = item.second;
    //std::printf("Plan for agent %lu\n", agent.id());

    const DiscretePath path = path_planner->plan(agent.id(), *snapshot);
    const double accel = speed_planner->plan(agent.id(), *snapshot);

    const double movement = agent.speed()*dt + 0.5*accel*dt*dt;
    const CarlaTransform updated_transform = path.transformAt(movement).first;
    const double updated_speed = agent.speed() + accel*dt;
    //std::printf("movement: %f\n", movement);
    //std::printf("updated speed: %f\n", updated_speed);
    //std::printf("updated transform: x:%f y:%f z:%f r:%f p:%f y:%f\n",
    //    updated_transform.location.x,
    //    updated_transform.location.y,
    //    updated_transform.location.z,
    //    updated_transform.rotation.roll,
    //    updated_transform.rotation.pitch,
    //    updated_transform.rotation.yaw);

    boost::shared_ptr<CarlaVehicle> vehicle = carlaVehicle(agent.id());
    vehicle->SetTransform(updated_transform);
    vehicle->SetVelocity(updated_transform.GetForwardVector()*updated_speed);
  }

  // Inform the client the result of plan.
  conformal_lattice_planner::AgentPlanResult result;
  result.success = true;
  server_.setSucceeded(result);

  return;
}
} // End namespace carla.

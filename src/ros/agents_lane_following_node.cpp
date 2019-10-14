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
  ROS_INFO_NAMED("agents_planner", "connect to the server.");
  client_ = boost::make_shared<CarlaClient>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  client_->GetWorld();

  // Create world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  map_ = world_->GetMap();
  fast_map_ = boost::make_shared<utils::FastWaypointMap>(map_);

  // Start the action server.
  ROS_INFO_NAMED("agents_planner", "start action server.");
  server_.start();

  ROS_INFO_NAMED("agents_planner", "initialization finishes.");
  return all_param_exist;
}

void AgentsLaneFollowingNode::executeCallback(
    const conformal_lattice_planner::AgentPlanGoalConstPtr& goal) {

  ROS_INFO_NAMED("agents_planner", "executeCallback()");

  // Update the carla world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  //map_ = world_->GetMap();

  // Get the ego and agent policies.
  const std::pair<size_t, double> ego_policy = egoPolicy(goal);
  const std::pair<size_t, double> ego_speed  = egoSpeed(goal);
  const std::unordered_map<size_t, double> agent_policies = agentPolicies(goal);
  const std::unordered_map<size_t, double> agent_speed    = agentSpeed(goal);

  // Create the current snapshot.
  boost::shared_ptr<Snapshot> snapshot =
    createSnapshot(ego_policy, ego_speed, agent_policies, agent_speed);

  // Create Path planner.
  std::vector<boost::shared_ptr<const WaypointNodeWithVehicle>>
    traffic_lattice_entries = snapshot->trafficLattice()->latticeEntries();

  const double range = snapshot->trafficLattice()->range() + 55.0;
  boost::shared_ptr<const CarlaWaypoint> waypoint = nullptr;
  for (const auto& node : traffic_lattice_entries) {
    if (node->distance() != 0.0) continue;
    waypoint = node->waypoint();
    break;
  }

  //if (!waypoint)
  //  throw std::runtime_error("No node with distance 0.");

  boost::shared_ptr<LaneFollower> path_planner =
    boost::make_shared<LaneFollower>(map_, fast_map_, waypoint, range, router_);

  // Create speed planner.
  boost::shared_ptr<VehicleSpeedPlanner> speed_planner =
    boost::make_shared<VehicleSpeedPlanner>();

  double dt = 0.05;
  nh_.param<double>("fixed_delta_seconds", dt, 0.05);

  // Compute the target speed and transform of all agents.
  conformal_lattice_planner::AgentPlanResult result;

  for (const auto& item : snapshot->agents()) {
    const Vehicle& agent = item.second;

    double accel = 0.0;
    double movement = 0.0;
    CarlaTransform updated_transform;
    double updated_speed = agent.speed();

    try {
      const DiscretePath path = path_planner->plan(agent.id(), *snapshot);
      accel = speed_planner->plan(agent.id(), *snapshot);

      movement = agent.speed()*dt + 0.5*accel*dt*dt;
      updated_transform = path.transformAt(movement).first;
      updated_speed = agent.speed() + accel*dt;

    } catch(...) {
      movement = agent.speed() * dt;
      // If we fail to plan a path for an agent vehicle,
      // we assume it moves with constant speed and get to the next accessible waypoint.
      boost::shared_ptr<CarlaWaypoint> agent_waypoint =
        fast_map_->waypoint(agent.transform());
      std::vector<boost::shared_ptr<CarlaWaypoint>> front_waypoints =
        agent_waypoint->GetNext(movement);
      updated_transform = front_waypoints.front()->GetTransform();
    }

    ROS_INFO_NAMED("agents_planner", "agent %lu", agent.id());
    ROS_INFO_NAMED("agents_planner", "movement:%f", movement);
    ROS_INFO_NAMED("agents_planner", "acceleration:%f", accel);
    ROS_INFO_NAMED("agents_planner", "updated speed:%f", updated_speed);
    ROS_INFO_NAMED("agents_planner", "updated transform: x:%f y:%f z:%f r:%f p:%f y:%f",
        updated_transform.location.x,
        updated_transform.location.y,
        updated_transform.location.z,
        updated_transform.rotation.roll,
        updated_transform.rotation.pitch,
        updated_transform.rotation.yaw);

    //if (updated_transform.location.x==0.0 &&
    //    updated_transform.location.y==0.0 &&
    //    updated_transform.location.z==0.0) {
    //  std::string error_msg(
    //      "AgentsLaneFollowingNode::executeCallback(): "
    //      "The updated transform of an agent vehicle is at origin.\n");
    //  std::string agent_msg = (boost::format("Agent ID: %lu\n") % agent.id()).str();
    //  throw std::runtime_error(error_msg + agent_msg);
    //}

    // Update the agent transform in the simulator.
    boost::shared_ptr<CarlaVehicle> vehicle = carlaVehicle(agent.id());
    vehicle->SetTransform(updated_transform);
    //vehicle->SetVelocity(updated_transform.GetForwardVector()*updated_speed);

    result.agent_target_speed.push_back(conformal_lattice_planner::VehicleSpeed());
    result.agent_target_speed.back().id = agent.id();
    result.agent_target_speed.back().speed = updated_speed;
  }

  // Inform the client the result of plan.
  result.success = true;
  server_.setSucceeded(result);

  return;
}
} // End namespace carla.

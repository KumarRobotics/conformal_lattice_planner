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
#include <chrono>
#include <unordered_set>

#include <conformal_lattice_planner/vehicle_path.h>
#include <conformal_lattice_planner/vehicle_speed_planner.h>
#include <conformal_lattice_planner/lane_follower.h>
#include <ros/ego_lane_following_node.h>
#include <ros/convert_to_visualization_msgs.h>

using namespace planner;
using namespace router;

namespace carla {

bool EgoLaneFollowingNode::initialize() {

  // Create publishers and subscribers.
  path_pub_ = nh_.advertise<visualization_msgs::Marker>("ego_path", 1, true);

  // Load parameters.
  bool all_param_exist = true;

  std::string host = "localhost";
  int port = 2000;
  all_param_exist &= nh_.param<std::string>("host", host, "localhost");
  all_param_exist &= nh_.param<int>("port", port, 2000);

  // Get the world.
  ROS_INFO_NAMED("ego_lane_following_planner", "connect to the server.");
  client_ = boost::make_shared<CarlaClient>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  client_->GetWorld();

  // Start the action server.
  ROS_INFO_NAMED("ego_lane_following_planner", "start action server.");
  server_.start();

  ROS_INFO_NAMED("ego_lane_following_planner", "initialization finishes.");
  return all_param_exist;
}

void EgoLaneFollowingNode::executeCallback(
    const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) {

  ROS_INFO_NAMED("ego_lane_following_planner", "executeCallback()");

  // Update the carla world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  map_ = world_->GetMap();

  // Get the ego and agent policies.
  const std::pair<size_t, double> ego_policy = egoPolicy(goal);
  std::unordered_map<size_t, double> agent_policies = agentPolicies(goal);

  // Create the current snapshot.
  boost::shared_ptr<Snapshot> snapshot = createSnapshot(ego_policy, agent_policies);

  // Plan path.
  // The range of the lattice is just enough for the ego vehicle.
  boost::shared_ptr<LaneFollower> path_planner =
    boost::make_shared<LaneFollower>(
      map_, carlaVehicleWaypoint(ego_policy.first), 55.0, router_);
  const ContinuousPath ego_path =
    path_planner->plan<ContinuousPath>(ego_policy.first, *snapshot);

  // Plan speed.
  boost::shared_ptr<VehicleSpeedPlanner> speed_planner =
    boost::make_shared<VehicleSpeedPlanner>();
  const double ego_accel = speed_planner->plan(ego_policy.first, *snapshot);

  // Update the ego vehicle in the simulator.
  double dt = 0.05;
  nh_.param<double>("fixed_delta_seconds", dt, 0.05);

  const double movement = snapshot->ego().speed()*dt + 0.5*ego_accel*dt*dt;
  const CarlaTransform updated_transform = ego_path.transformAt(movement);
  const double updated_speed = snapshot->ego().speed() + ego_accel*dt;

  boost::shared_ptr<CarlaVehicle> ego_vehicle = carlaVehicle(ego_policy.first);
  ego_vehicle->SetTransform(updated_transform);
  ego_vehicle->SetVelocity(updated_transform.GetForwardVector()*updated_speed);

  // Publish the path planned for the ego.
  path_pub_.publish(createEgoPathMsg(ego_path));

  // Inform the client the result of plan.
  conformal_lattice_planner::EgoPlanResult result;
  result.success = true;
  server_.setSucceeded(result);

  return;
}

} // End namespace carla.

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
#include <boost/timer/timer.hpp>
#include <ros/ego_conformal_lattice_planning_node.h>

using namespace planner;
using namespace router;

namespace carla {

bool EgoConformalLatticePlanningNode::initialize() {

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

  double fixed_delta_seconds = 0.05;
  all_param_exist &= nh_.param<double>("fixed_delta_seconds", fixed_delta_seconds, 0.05);

  // Start the action server.
  ROS_INFO_NAMED("ego_lane_following_planner", "start action server.");
  server_.start();

  ROS_INFO_NAMED("ego_lane_following_planner", "initialization finishes.");
  return all_param_exist;
}

void EgoConformalLatticePlanningNode::executeCallback(
    const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) {

  ROS_INFO_NAMED("ego_lane_following_planner", "executeCallback()");

  // Get the ego policy.
  const std::pair<size_t, double> ego_policy = egoPolicy(goal);

  // Get the agents IDs.
  // Do not need the desired speed for them.
  std::unordered_map<size_t, double> agent_policies = agentPolicies(goal);

  // TODO: construct the planner.
  double fixed_delta_seconds = 0.05;
  nh_.param<double>("fixed_delta_seconds", fixed_delta_seconds, 0.05);
  boost::shared_ptr<LoopRouter> loop_router = boost::make_shared<LoopRouter>();

  planner_ = boost::make_shared<ConformalLatticePlanner>(
      fixed_delta_seconds, ego_policy.first, 155.0, loop_router);

  // Update the world for the planner.
  boost::shared_ptr<CarlaWorld> world =
    boost::make_shared<CarlaWorld>(client_->GetWorld());
  planner_->updateWorld(world);

  // Plan for the ego vehicle.
  planner_->plan(ego_policy, agent_policies);

  // Inform the client the result of plan.
  conformal_lattice_planner::EgoPlanResult result;
  result.success = true;
  server_.setSucceeded(result);

  return;
}

} // End namespace carla.

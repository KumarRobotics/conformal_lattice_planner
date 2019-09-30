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
#include <ros/convert_to_visualization_msgs.h>

using namespace planner;
using namespace router;

namespace carla {

bool EgoConformalLatticePlanningNode::initialize() {

  bool all_param_exist = true;

  // Create the publishers.
  conformal_lattice_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "conformal_lattice", 1, true);

  std::string host = "localhost";
  int port = 2000;
  all_param_exist &= nh_.param<std::string>("host", host, "localhost");
  all_param_exist &= nh_.param<int>("port", port, 2000);

  // Get the world.
  ROS_INFO_NAMED("ego_conformal_lattice_planner", "connect to the server.");
  client_ = boost::make_shared<CarlaClient>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  client_->GetWorld();

  double fixed_delta_seconds = 0.05;
  all_param_exist &= nh_.param<double>("fixed_delta_seconds", fixed_delta_seconds, 0.05);

  // Start the action server.
  ROS_INFO_NAMED("ego_conformal_lattice_planner", "start action server.");
  server_.start();

  ROS_INFO_NAMED("ego_conformal_lattice_planner", "initialization finishes.");
  return all_param_exist;
}

void EgoConformalLatticePlanningNode::executeCallback(
    const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) {

  ROS_INFO_NAMED("ego_conformal_lattice_planner", "executeCallback()");

  // Get the ego policy.
  const std::pair<size_t, double> ego_policy = egoPolicy(goal);
  std::printf("ego:%lu policy:%f\n", ego_policy.first, ego_policy.second);

  // Get the agents IDs.
  // Do not need the desired speed for them.
  std::unordered_map<size_t, double> agent_policies = agentPolicies(goal);
  for (const auto& agent : agent_policies)
    std::printf("agent:%lu policy:%f\n", agent.first, agent.second);

  // TODO: construct the planner.
  //double fixed_delta_seconds = 0.05;
  //nh_.param<double>("fixed_delta_seconds", fixed_delta_seconds, 0.05);
  boost::shared_ptr<LoopRouter> loop_router = boost::make_shared<LoopRouter>();
  boost::shared_ptr<CarlaWorld> world = boost::make_shared<CarlaWorld>(client_->GetWorld());

  std::printf("Construct conformal lattice planner.\n");
  boost::shared_ptr<ConformalLatticePlanner> planner_ =
    boost::make_shared<ConformalLatticePlanner>(
        0.1, ego_policy.first, 205.0, loop_router, world);

  // Update the world for the planner.
  //planner_->updateWorld(world);

  // Plan for the ego vehicle.
  std::printf("Calling conformal lattice planner.\n");
  planner_->plan(ego_policy, agent_policies);

  // Publish the station graph.
  std::printf("Publish conformal lattice message.\n");
  conformal_lattice_pub_.publish(createConformalLatticeMsg(planner_));

  // Inform the client the result of plan.
  std::printf("Wrap up the callback function.\n");
  conformal_lattice_planner::EgoPlanResult result;
  result.success = true;
  server_.setSucceeded(result);

  // FIXME: For debugging only, just response to the client one time.
  //server_.shutdown();
  std::cin.get();

  return;
}

} // End namespace carla.

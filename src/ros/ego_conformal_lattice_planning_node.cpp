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

#include <conformal_lattice_planner/conformal_lattice_planner.h>
#include <ros/ego_conformal_lattice_planning_node.h>
#include <ros/convert_to_visualization_msgs.h>

using namespace planner;
using namespace router;

namespace carla {

bool EgoConformalLatticePlanningNode::initialize() {

  // Create the publishers.
  conformal_lattice_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "conformal_lattice", 1, true);

  bool all_param_exist = true;

  std::string host = "localhost";
  int port = 2000;
  all_param_exist &= nh_.param<std::string>("host", host, "localhost");
  all_param_exist &= nh_.param<int>("port", port, 2000);

  // Get the world.
  ROS_INFO_NAMED("ego_conformal_lattice_planner", "connect to the server.");
  client_ = boost::make_shared<CarlaClient>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  client_->GetWorld();

  //double fixed_delta_seconds = 0.05;
  //all_param_exist &= nh_.param<double>("fixed_delta_seconds", fixed_delta_seconds, 0.05);

  // Start the action server.
  ROS_INFO_NAMED("ego_conformal_lattice_planner", "start action server.");
  server_.start();

  ROS_INFO_NAMED("ego_conformal_lattice_planner", "initialization finishes.");
  return all_param_exist;
}

void EgoConformalLatticePlanningNode::executeCallback(
    const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) {

  ROS_INFO_NAMED("ego_conformal_lattice_planner", "executeCallback()");

  // Update the carla world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  map_ = world_->GetMap();

  // Get the ego and agent policies.
  const std::pair<size_t, double> ego_policy = egoPolicy(goal);
  std::unordered_map<size_t, double> agent_policies = agentPolicies(goal);

  // Create the current snapshot.
  boost::shared_ptr<Snapshot> snapshot = createSnapshot(ego_policy, agent_policies);


  //double fixed_delta_seconds = 0.05;
  //nh_.param<double>("fixed_delta_seconds", fixed_delta_seconds, 0.05);
  // Create the loop router.
  boost::shared_ptr<LoopRouter> loop_router = boost::make_shared<LoopRouter>();

  // Construct the planner.
  std::printf("Construct conformal lattice planner.\n");
  boost::shared_ptr<ConformalLatticePlanner> path_planner_ =
    boost::make_shared<ConformalLatticePlanner>(
        0.1, 105.0, loop_router, map_);

  // Plan path.
  std::printf("Calling conformal lattice planner.\n");
  const ContinuousPath path = path_planner_->plan<ContinuousPath>(ego_policy.first, *snapshot);

  std::printf("nodes #: %lu\n", path_planner_->nodes().size());
  std::printf("edges #: %lu\n", path_planner_->edges().size());

  // Publish the station graph.
  std::printf("Publish conformal lattice message.\n");
  conformal_lattice_pub_.publish(createConformalLatticeMsg(path_planner_));

  //// Plan speed.
  //boost::shared_ptr<VehicleSpeedPlanner> speed_planner =
  //  boost::make_shared<VehicleSpeedPlanner>();
  //const double ego_accel = speed_planner->plan(ego_policy.first, *snapshot);

  //// Update the ego vehicle in the simulator.
  //double dt = 0.05;
  //nh_.param<double>("fixed_delta_seconds", dt, 0.05);

  //const double movement = snapshot->ego().speed()*dt + 0.5*ego_accel*dt*dt;
  //const CarlaTransform updated_transform = ego_path.transformAt(movement);
  //const double updated_speed = snapshot->ego().speed() + ego_accel*dt;

  //boost::shared_ptr<CarlaVehicle> ego_vehicle = carlaVehicle(ego_policy.first);
  //ego_vehicle->SetTransform(updated_transform);
  //ego_vehicle->SetVelocity(updated_transform.GetForwardVector()*updated_speed);

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

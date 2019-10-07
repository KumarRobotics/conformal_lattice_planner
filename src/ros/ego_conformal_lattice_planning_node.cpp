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

  // Create the publishers.
  path_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "ego_path", 1, true);
  conformal_lattice_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "conformal_lattice", 1, true);
  waypoint_lattice_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "waypoint_lattice", 1, true);

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

  // Get the world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  map_ = world_->GetMap();

  // Initialize the path and speed planner.
  boost::shared_ptr<router::LoopRouter> router = boost::make_shared<router::LoopRouter>();
  path_planner_ = boost::make_shared<planner::ConformalLatticePlanner>(0.1, 105.0, router, map_);
  speed_planner_ = boost::make_shared<planner::VehicleSpeedPlanner>();

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

  // Plan path.
  //std::printf("Calling conformal lattice planner.\n");
  const DiscretePath ego_path = path_planner_->plan(ego_policy.first, *snapshot);

  std::printf("station lattice nodes #: %lu\n", path_planner_->nodes().size());
  std::printf("station lattice edges #: %lu\n", path_planner_->edges().size());

  // Publish the station graph.
  //std::printf("Publish conformal lattice message.\n");
  conformal_lattice_pub_.publish(createConformalLatticeMsg(path_planner_));
  path_pub_.publish(createEgoPathMsg(ego_path));
  waypoint_lattice_pub_.publish(createWaypointLatticeMsg(path_planner_->waypointLattice()));

  // Plan speed.
  const double ego_accel = speed_planner_->plan(ego_policy.first, *snapshot);
  std::printf("ego accel: %f\n", ego_accel);

  // Update the ego vehicle in the simulator.
  double dt = 0.05;
  nh_.param<double>("fixed_delta_seconds", dt, 0.05);

  const double movement = snapshot->ego().speed()*dt + 0.5*ego_accel*dt*dt;
  const CarlaTransform updated_transform = ego_path.transformAt(movement);
  const double updated_speed = snapshot->ego().speed() + ego_accel*dt;
  std::printf("movement: %f\n", movement);
  std::printf("current transform: x:%f y:%f z:%f r:%f p:%f y:%f\n",
      snapshot->ego().transform().location.x,
      snapshot->ego().transform().location.y,
      snapshot->ego().transform().location.z,
      snapshot->ego().transform().rotation.roll,
      snapshot->ego().transform().rotation.pitch,
      snapshot->ego().transform().rotation.yaw);
  std::printf("updated transform: x:%f y:%f z:%f r:%f p:%f y:%f\n",
      updated_transform.location.x,
      updated_transform.location.y,
      updated_transform.location.z,
      updated_transform.rotation.roll,
      updated_transform.rotation.pitch,
      updated_transform.rotation.yaw);
  std::printf("updated speed: %f\n", updated_speed);

  boost::shared_ptr<CarlaVehicle> ego_vehicle = carlaVehicle(ego_policy.first);
  ego_vehicle->SetTransform(updated_transform);
  ego_vehicle->SetVelocity(updated_transform.GetForwardVector()*updated_speed);

  //std::cin.get();

  // Inform the client the result of plan.
  conformal_lattice_planner::EgoPlanResult result;
  result.success = true;
  server_.setSucceeded(result);

  return;
}

} // End namespace carla.

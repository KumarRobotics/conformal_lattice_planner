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
  ROS_INFO_NAMED("ego_planner", "connect to the server.");
  client_ = boost::make_shared<CarlaClient>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  ros::Duration(1.0).sleep();

  // Get the world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  map_ = world_->GetMap();
  fast_map_ = boost::make_shared<utils::FastWaypointMap>(map_);

  // Initialize the path and speed planner.
  boost::shared_ptr<router::LoopRouter> router = boost::make_shared<router::LoopRouter>();
  path_planner_ = boost::make_shared<planner::ConformalLatticePlanner>(0.1, 105.0, router, map_, fast_map_);
  speed_planner_ = boost::make_shared<planner::VehicleSpeedPlanner>();

  // Start the action server.
  ROS_INFO_NAMED("ego_planner", "start action server.");
  server_.start();

  ROS_INFO_NAMED("ego_planner", "initialization finishes.");
  return all_param_exist;
}

boost::shared_ptr<planner::Snapshot> EgoConformalLatticePlanningNode::createSnapshot(
    const std::pair<size_t, double>& ego_policy,
    const std::pair<size_t, double>& ego_speed,
    const std::unordered_map<size_t, double>& agent_policies,
    const std::unordered_map<size_t, double>& agent_speed) {

  // Create the ego vehicle.
  const boost::shared_ptr<CarlaWaypoint> ego_waypoint =
    carlaVehicleWaypoint(ego_policy.first);
  if (!ego_curvature)
    ego_curvature = utils::curvatureAtWaypoint(ego_waypoint, map_);
  const planner::Vehicle ego_vehicle =
    planner::Vehicle(carlaVehicle(ego_policy.first),
                     ego_speed.second,
                     ego_policy.second,
                     *ego_curvature);

  // Create the agent vehicles.
  std::unordered_map<size_t, planner::Vehicle> agent_vehicles;
  for (const auto& agent : agent_policies) {
    const boost::shared_ptr<CarlaWaypoint> waypoint =
      carlaVehicleWaypoint(agent.first);
    const double curvature =
      utils::curvatureAtWaypoint(waypoint, map_);

    const double policy_speed = agent.second;
    const double current_speed = agent_speed.find(agent.first)->second;

    agent_vehicles.insert(std::make_pair(agent.first, planner::Vehicle(
            carlaVehicle(agent.first),
            current_speed,
            policy_speed,
            curvature)));
  }

  // Create the snapshot.
  return boost::make_shared<planner::Snapshot>(
      ego_vehicle, agent_vehicles, router_, map_, fast_map_);
}

void EgoConformalLatticePlanningNode::executeCallback(
    const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) {

  ROS_INFO_NAMED("ego_planner", "executeCallback()");

  ros::Time start_time = ros::Time::now();

  // Update the carla world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  //map_ = world_->GetMap();

  // Get the ego and agent policies and speed.
  const std::pair<size_t, double> ego_policy = egoPolicy(goal);
  const std::pair<size_t, double> ego_speed  = egoSpeed(goal);
  const std::unordered_map<size_t, double> agent_policies = agentPolicies(goal);
  const std::unordered_map<size_t, double> agent_speed    = agentSpeed(goal);

  // Create the current snapshot.
  boost::shared_ptr<Snapshot> snapshot =
    createSnapshot(ego_policy, ego_speed, agent_policies, agent_speed);

  // Plan path.
  const DiscretePath ego_path = path_planner_->plan(ego_policy.first, *snapshot);

  // Publish the station graph.
  conformal_lattice_pub_.publish(createConformalLatticeMsg(path_planner_));
  path_pub_.publish(createEgoPathMsg(ego_path));
  //waypoint_lattice_pub_.publish(createWaypointLatticeMsg(path_planner_->waypointLattice()));

  // Plan speed.
  const double ego_accel = speed_planner_->plan(ego_policy.first, *snapshot);

  // Update the ego vehicle in the simulator.
  double dt = 0.05;
  nh_.param<double>("fixed_delta_seconds", dt, 0.05);

  const double movement = snapshot->ego().speed()*dt + 0.5*ego_accel*dt*dt;
  const std::pair<CarlaTransform, double> updated_transform_curvature = ego_path.transformAt(movement);
  const CarlaTransform updated_transform = updated_transform_curvature.first;
  ego_curvature = updated_transform_curvature.second;
  const double updated_speed = snapshot->ego().speed() + ego_accel*dt;

  ROS_INFO_NAMED("ego_planner", "ego %lu", snapshot->ego().id());
  ROS_INFO_NAMED("ego_planner", "movement:%f", movement);
  ROS_INFO_NAMED("ego_planner", "acceleration:%f", ego_accel);
  ROS_INFO_NAMED("ego_planner", "speed:%f", updated_speed);
  ROS_INFO_NAMED("ego_planner", "transform: x:%f y:%f z:%f r:%f p:%f y:%f",
      updated_transform.location.x,
      updated_transform.location.y,
      updated_transform.location.z,
      updated_transform.rotation.roll,
      updated_transform.rotation.pitch,
      updated_transform.rotation.yaw);

  boost::shared_ptr<CarlaVehicle> ego_vehicle = carlaVehicle(ego_policy.first);
  ego_vehicle->SetTransform(updated_transform);
  //ego_vehicle->SetVelocity(updated_transform.GetForwardVector()*updated_speed);

  //std::cin.get();

  // Inform the client the result of plan.
  conformal_lattice_planner::EgoPlanResult result;
  result.success = true;
  result.ego_target_speed.id = ego_policy.first;
  result.ego_target_speed.speed = updated_speed;
  server_.setSucceeded(result);

  ros::Time end_time = ros::Time::now();
  ROS_INFO_NAMED("ego_planner", "planning time: %f",
      (end_time-start_time).toSec());
  if ((end_time-start_time).toSec() < 0.2) {
    ros::Duration delay(0.2-(end_time-start_time).toSec());
    delay.sleep();
  }

  return;
}

} // End namespace carla.

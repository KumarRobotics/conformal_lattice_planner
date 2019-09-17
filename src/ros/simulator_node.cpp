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
#include <ros/simulator_node.h>
#include <ros/convert_to_visualization_msgs.h>

namespace carla {

bool SimulatorNode::initialize() {

  bool all_param_exist = true;

  // Create publishers.
  map_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("town_map", 1, true);
  traffic_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("traffic", 1, true);

  // Get the world.
  std::string host = "localhost";
  int port = 2000;
  all_param_exist &= nh_.param<std::string>("host", host, "localhost");
  all_param_exist &= nh_.param<int>("port", port, 2000);

  ROS_INFO_NAMED("carla_simulator", "connect to the server.");
  client_ = boost::make_shared<CarlaClient>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  ros::Duration(1.0).sleep();

  // Applying the world settings.
  double fixed_delta_seconds = 0.05;
  bool no_rendering_mode = true;
  bool synchronous_mode = true;
  all_param_exist &= nh_.param<double>("fixed_delta_seconds", fixed_delta_seconds, 0.05);
  all_param_exist &= nh_.param<bool>("no_rendering_mode", no_rendering_mode, true);
  all_param_exist &= nh_.param<bool>("synchronous_mode", synchronous_mode, true);

  ROS_INFO_NAMED("carla_simulator", "apply world settings.");
  carla::rpc::EpisodeSettings settings = world_->GetSettings();
  if (settings.fixed_delta_seconds) {
    ROS_DEBUG_NAMED("carla_simulator",
        "old settings: fixed_delta_seconds:N/A no_rendering_mode:%d synchronous_mode:%d",
        settings.no_rendering_mode,
        settings.synchronous_mode);
  } else {
    ROS_DEBUG_NAMED("carla_simulator",
        "old settings: fixed_delta_seconds:%f no_rendering_mode:%d synchronous_mode:%d",
        *(settings.fixed_delta_seconds),
        settings.no_rendering_mode,
        settings.synchronous_mode);
  }
  settings.fixed_delta_seconds = fixed_delta_seconds;
  settings.no_rendering_mode = no_rendering_mode;
  settings.synchronous_mode = synchronous_mode;
  world_->ApplySettings(settings);
  ros::Duration(1.0).sleep();

  settings = world_->GetSettings();
  ROS_INFO_NAMED("carla_simulator",
      "new settings: fixed_delta_seconds:%f no_rendering_mode:%d synchronous_mode:%d",
      *(settings.fixed_delta_seconds),
      settings.no_rendering_mode,
      settings.synchronous_mode);

  // Publish the map.
  ROS_INFO_NAMED("carla_simulator", "publish global map.");
  publishMap();

  // Initialize the ego vehicle.
  ROS_INFO_NAMED("carla_simulator", "spawn the vehicles.");
  spawnVehicles();

  // Publish the ego vehicle marker.
  ROS_INFO_NAMED("carla_simulator", "publish ego and agents.");
  publishTraffic();

  // Wait for the planner servers.
  ROS_INFO_NAMED("carla_simulator", "waiting for action servers.");
  ego_client_.waitForServer(ros::Duration(5.0));
  agents_client_.waitForServer(ros::Duration(5.0));

  // Send out the first goal of ego.
  ROS_INFO_NAMED("carla_simulator", "send the first goals to action servers");
  sendEgoGoal();
  sendAgentsGoal();

  ROS_INFO_NAMED("carla_simulator", "initialization finishes.");
  return all_param_exist;
}

boost::shared_ptr<SimulatorNode::CarlaVehicle>
  SimulatorNode::egoVehicle() {
  return boost::static_pointer_cast<CarlaVehicle>(
      world_->GetActor(ego_policy_.first));
}

boost::shared_ptr<const SimulatorNode::CarlaVehicle>
  SimulatorNode::egoVehicle() const {
  return boost::static_pointer_cast<CarlaVehicle>(
      world_->GetActor(ego_policy_.first));
}

boost::shared_ptr<SimulatorNode::CarlaVehicle>
  SimulatorNode::agentVehicle(const size_t agent) {
  if (agent_policies_.count(agent) == 0) return nullptr;
  return boost::static_pointer_cast<CarlaVehicle>(world_->GetActor(agent));
}

boost::shared_ptr<const SimulatorNode::CarlaVehicle>
  SimulatorNode::agentVehicle(const size_t agent) const {
  if (agent_policies_.count(agent) == 0) return nullptr;
  return boost::static_pointer_cast<CarlaVehicle>(world_->GetActor(agent));
}

std::vector<boost::shared_ptr<SimulatorNode::CarlaVehicle>>
  SimulatorNode::agentVehicles() {
  std::vector<boost::shared_ptr<CarlaVehicle>> vehicles;
  for (const auto& agent : agent_policies_)
    vehicles.push_back(agentVehicle(agent.first));
  return vehicles;
}

std::vector<boost::shared_ptr<const SimulatorNode::CarlaVehicle>>
  SimulatorNode::agentVehicles() const {
  std::vector<boost::shared_ptr<const CarlaVehicle>> vehicles;
  for (const auto& agent : agent_policies_)
    vehicles.push_back(agentVehicle(agent.first));
  return vehicles;
}

std::vector<boost::shared_ptr<SimulatorNode::CarlaVehicle>>
  SimulatorNode::vehicles() {
  std::vector<boost::shared_ptr<CarlaVehicle>> vehicles = agentVehicles();
  vehicles.push_back(egoVehicle());
  return vehicles;
}

std::vector<boost::shared_ptr<const SimulatorNode::CarlaVehicle>>
  SimulatorNode::vehicles() const {
  std::vector<boost::shared_ptr<const CarlaVehicle>> vehicles = agentVehicles();
  vehicles.push_back(egoVehicle());
  return vehicles;
}

void SimulatorNode::publishMap() const {

  std::vector<boost::shared_ptr<CarlaWaypoint>> waypoints =
    world_->GetMap()->GenerateWaypoints(5.0);
  std::vector<boost::shared_ptr<const CarlaWaypoint>> const_waypoints;

  for (const auto& waypoint : waypoints)
    const_waypoints.push_back(waypoint);

  visualization_msgs::MarkerPtr waypoints_msg =
    createWaypointMsg(const_waypoints);
  visualization_msgs::MarkerPtr junctions_msg =
    createJunctionMsg(world_->GetMap()->GetTopology());
  visualization_msgs::MarkerArrayPtr road_ids_msg =
    createRoadIdsMsg(world_->GetMap()->GetMap().GetMap().GetRoads());

  visualization_msgs::MarkerArrayPtr map_msg(new visualization_msgs::MarkerArray);
  map_msg->markers.push_back(*waypoints_msg);
  map_msg->markers.push_back(*junctions_msg);
  map_msg->markers.insert(
      map_msg->markers.begin(),
      road_ids_msg->markers.begin(), road_ids_msg->markers.end());

  map_pub_.publish(map_msg);
  return;
}

void SimulatorNode::publishTraffic() const {

  // Ego vehicle transform.
  tf_broadcaster_.sendTransform(*(createVehicleTransformMsg(egoVehicle(), "ego")));

  // Traffic msg.
  visualization_msgs::MarkerArrayPtr vehicles_msg =
    createVehiclesMsg(this->vehicles());
  visualization_msgs::MarkerArrayPtr vehicle_ids_msg =
    createVehicleIdsMsg(this->vehicles());

  visualization_msgs::MarkerArrayPtr traffic_msg(
      new visualization_msgs::MarkerArray);
  traffic_msg->markers.insert(
      traffic_msg->markers.end(),
      vehicles_msg->markers.begin(), vehicles_msg->markers.end());
  traffic_msg->markers.insert(
      traffic_msg->markers.end(),
      vehicle_ids_msg->markers.begin(), vehicle_ids_msg->markers.end());

  traffic_pub_.publish(traffic_msg);
  return;
}

void SimulatorNode::sendEgoGoal() {

  conformal_lattice_planner::EgoPlanGoal goal;
  goal.ego_policy.id = ego_policy_.first;
  goal.ego_policy.desired_speed = ego_policy_.second;

  for (const auto agent_policy : agent_policies_) {
    goal.agent_policies.push_back(conformal_lattice_planner::Policy());
    goal.agent_policies.back().id = agent_policy.first;
    goal.agent_policies.back().desired_speed = agent_policy.second;
  }

  ego_client_.sendGoal(
      goal,
      boost::bind(&SimulatorNode::egoPlanDoneCallback, this, _1, _2),
      boost::bind(&SimulatorNode::egoPlanActiveCallback, this),
      boost::bind(&SimulatorNode::egoPlanFeedbackCallback, this, _1));

  ego_ready_ = false;

  return;
}

void SimulatorNode::egoPlanDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const conformal_lattice_planner::EgoPlanResultConstPtr& result) {

  ROS_INFO_NAMED("carla_simulator", "egoPlanDoneCallback().");
  ego_ready_ = true;

  if (ego_ready_ && agents_ready_) {
    ROS_INFO_NAMED("carla_simulator", "tick world by ego client.");
    tickWorld();
  }

  return;
}

void SimulatorNode::sendAgentsGoal() {

  conformal_lattice_planner::AgentPlanGoal goal;
  goal.ego_policy.id = ego_policy_.first;
  goal.ego_policy.desired_speed = ego_policy_.second;

  for (const auto agent_policy : agent_policies_) {
    goal.agent_policies.push_back(conformal_lattice_planner::Policy());
    goal.agent_policies.back().id = agent_policy.first;
    goal.agent_policies.back().desired_speed = agent_policy.second;
  }

  agents_client_.sendGoal(
      goal,
      boost::bind(&SimulatorNode::agentsPlanDoneCallback, this, _1, _2),
      boost::bind(&SimulatorNode::agentsPlanActiveCallback, this),
      boost::bind(&SimulatorNode::agentsPlanFeedbackCallback, this, _1));

  agents_ready_ = false;
  return;
}

void SimulatorNode::agentsPlanDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const conformal_lattice_planner::AgentPlanResultConstPtr& result) {

  ROS_INFO_NAMED("carla_simulator", "agentsPlanDoneCallback().");
  agents_ready_ = true;

  if (ego_ready_ && agents_ready_) {
    ROS_INFO_NAMED("carla_simulator", "tick world by agents client.");
    tickWorld();
  }

  return;
}

} // End namespace carla.
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
#include <random>

#include <ros/ros.h>
#include <ros/console.h>

#include <planner/common/vehicle_path.h>
#include <planner/common/vehicle_speed_planner.h>
#include <planner/lane_follower/lane_follower.h>
#include <planner/common/utils.h>
#include <node/planner/agents_lane_following_node.h>

using namespace router;
using namespace planner;
using namespace planner::lane_follower;

namespace node {

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

void AgentsLaneFollowingNode::perturbAgentPolicies(
    const boost::shared_ptr<planner::Snapshot>& snapshot) {

  // Collect all current agent IDs.
  std::unordered_set<size_t> current_agents;
  for (const auto& agent : snapshot->agents())
    current_agents.insert(agent.first);

  // Update the policy speed of all agents.
  const size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine rand_gen(seed);

  for (const size_t agent : current_agents) {

    double noise = 0.0;
    if (agent_policy_.count(agent) > 0) {
      const double sigma = 2.0;
      const double sigma_xy = 1.95;
      std::normal_distribution<double> normal_dist(
          sigma_xy/sigma*agent_policy_[agent].second,
          std::sqrt(sigma-sigma_xy*sigma_xy/sigma));
      noise = normal_dist(rand_gen);
    }

    agent_policy_[agent] =
      std::make_pair(snapshot->agent(agent).policySpeed(), noise);
    snapshot->agent(agent).policySpeed() =
      agent_policy_[agent].first + agent_policy_[agent].second;
  }

  // Remove the agents that are no longer in the snapshot.
  std::unordered_set<size_t> agents_to_erase;
  for (const auto agent : agent_policy_) {
    if (current_agents.count(agent.first) > 0) continue;
    agents_to_erase.insert(agent.first);
  }

  for (const size_t agent : agents_to_erase)
    agent_policy_.erase(agent);

  return;
}

void AgentsLaneFollowingNode::manageAgentIdms(
    const boost::shared_ptr<planner::Snapshot>& snapshot) {

  // Collect all current agent IDs.
  std::unordered_set<size_t> current_agents;
  for (const auto& agent : snapshot->agents())
    current_agents.insert(agent.first);

  // Generate IDMs for new agents if necessary.
  const size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine rand_gen(seed);
  std::uniform_real_distribution<double> headway_noise_dist(-0.2, 0.2);
  std::uniform_real_distribution<double> distance_noise_dist(-1.0, 1.0);

  for (const size_t agent : current_agents) {
    if (agent_idm_.count(agent) > 0) continue;
    agent_idm_[agent] = boost::make_shared<IntelligentDriverModel>(
        1.0 + headway_noise_dist(rand_gen),
        6.0 + distance_noise_dist(rand_gen));
  }

  // Remove agents that are no longer in the snapshot.
  std::unordered_set<size_t> agents_to_erase;
  for (const auto agent : agent_idm_) {
    if (current_agents.count(agent.first) > 0) continue;
    agents_to_erase.insert(agent.first);
  }

  for (const size_t agent : agents_to_erase)
    agent_idm_.erase(agent);

  return;
}

void AgentsLaneFollowingNode::executeCallback(
    const conformal_lattice_planner::AgentPlanGoalConstPtr& goal) {

  ROS_INFO_NAMED("agents_planner", "executeCallback()");

  // Update the carla world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  //map_ = world_->GetMap();

  // Create the current snapshot.
  boost::shared_ptr<Snapshot> snapshot = createSnapshot(goal->snapshot);
  perturbAgentPolicies(snapshot);
  manageAgentIdms(snapshot);

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

  boost::shared_ptr<LaneFollower> path_planner =
    boost::make_shared<LaneFollower>(map_, fast_map_, waypoint, range, router_);

  //// Create speed planner.
  //boost::shared_ptr<VehicleSpeedPlanner> speed_planner =
  //  boost::make_shared<VehicleSpeedPlanner>();

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

      boost::shared_ptr<VehicleSpeedPlanner> speed_planner =
        boost::make_shared<VehicleSpeedPlanner>(agent_idm_[agent.id()]);

      const DiscretePath path = path_planner->planPath(agent.id(), *snapshot);
      accel = speed_planner->planSpeed(agent.id(), *snapshot);

      movement = agent.speed()*dt + 0.5*accel*dt*dt;
      updated_transform = path.transformAt(movement).first;
      updated_speed = agent.speed() + accel*dt;

    } catch(...) {
      //movement = agent.speed() * dt;
      // FIXME: It seems sometimes the speed is set back to 0.
      //        Not sure what causes this.
      movement = 1.0;
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

    // Update the agent transform in the simulator.
    boost::shared_ptr<CarlaVehicle> vehicle = carlaVehicle(agent.id());
    vehicle->SetTransform(updated_transform);
    //vehicle->SetVelocity(updated_transform.GetForwardVector()*updated_speed);

    planner::Vehicle updated_agent(
        agent.id(),
        agent.boundingBox(),
        updated_transform,
        updated_speed,
        agent_policy_[agent.id()].first,
        accel,
        utils::curvatureAtWaypoint(map_->GetWaypoint(updated_transform.location), map_));

    result.agents.push_back(conformal_lattice_planner::Vehicle());
    populateVehicleMsg(updated_agent, result.agents.back());
  }

  // Inform the client the result of plan.
  result.header.stamp = ros::Time::now();
  result.success = true;
  server_.setSucceeded(result);

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

  node::AgentsLaneFollowingNodePtr planner =
    boost::make_shared<node::AgentsLaneFollowingNode>(nh);
  if (!planner->initialize()) {
    ROS_ERROR("Cannot initialize the agents lane following planner.");
  }

  ros::spin();
  return 0;
}

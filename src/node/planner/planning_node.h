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

#pragma once

#include <utility>
#include <unordered_map>

#include <boost/core/noncopyable.hpp>
#include <boost/smart_ptr.hpp>

#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Map.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Waypoint.h>
#include <carla/geom/Transform.h>
#include <carla/road/Road.h>
#include <carla/road/element/RoadInfoGeometry.h>

#include <ros/ros.h>
#include <router/loop_router/loop_router.h>
#include <planner/common/snapshot.h>
#include <planner/common/utils.h>
#include <planner/common/fast_waypoint_map.h>

namespace carla {

class PlanningNode : private boost::noncopyable {

protected:

  using CarlaClient    = carla::client::Client;
  using CarlaWorld     = carla::client::World;
  using CarlaMap       = carla::client::Map;
  using CarlaWaypoint  = carla::client::Waypoint;
  using CarlaVehicle   = carla::client::Vehicle;
  using CarlaTransform = carla::geom::Transform;

protected:

  boost::shared_ptr<router::LoopRouter> router_       = nullptr;
  boost::shared_ptr<utils::FastWaypointMap> fast_map_ = nullptr;

  boost::shared_ptr<CarlaClient> client_ = nullptr;
  boost::shared_ptr<CarlaWorld> world_   = nullptr;
  boost::shared_ptr<CarlaMap> map_       = nullptr;

  mutable ros::NodeHandle nh_;

public:

  PlanningNode(ros::NodeHandle& nh) :
    router_(boost::make_shared<router::LoopRouter>()), nh_(nh) {}

  virtual ~PlanningNode() {}

  virtual bool initialize() = 0;

protected:

  virtual boost::shared_ptr<planner::Snapshot> createSnapshot(
      const std::pair<size_t, double>& ego_policy,
      const std::pair<size_t, double>& ego_speed,
      const std::unordered_map<size_t, double>& agent_policies,
      const std::unordered_map<size_t, double>& agent_speed) {

    // FIXME: In this function, we assume all vehicles are lane followers.
    //        Therefore, the curvature of the vehicle path is the curvature
    //        of the road. In case this is not true, one should override
    //        this function.

    // Create the ego vehicle.
    const boost::shared_ptr<CarlaWaypoint> ego_waypoint =
      carlaVehicleWaypoint(ego_policy.first);
    const double ego_curvature =
      utils::curvatureAtWaypoint(ego_waypoint, map_);
    const planner::Vehicle ego_vehicle =
      planner::Vehicle(carlaVehicle(ego_policy.first),
                       ego_speed.second,
                       ego_policy.second,
                       ego_curvature);

    if (ego_vehicle.transform().location.x==0.0 &&
        ego_vehicle.transform().location.y==0.0 &&
        ego_vehicle.transform().location.z==0.0) {
      std::string error_msg(
          "PlanningNode::createSnapshot(): "
          "ego vehicle is set back to origin.\n");
      std::string ego_msg = (
          boost::format("Ego ID: %lu\n") % ego_vehicle.id()).str();
      throw std::runtime_error(error_msg + ego_msg);
    }

    // Create the agent vehicles.
    std::unordered_map<size_t, planner::Vehicle> agent_vehicles;
    for (const auto& agent : agent_policies) {
      const boost::shared_ptr<CarlaWaypoint> waypoint =
        carlaVehicleWaypoint(agent.first);
      const double curvature =
        utils::curvatureAtWaypoint(waypoint, map_);

      const double policy_speed = agent.second;
      const double current_speed = agent_speed.find(agent.first)->second;

      const planner::Vehicle vehicle(
          carlaVehicle(agent.first), current_speed, policy_speed, curvature);

      if (vehicle.transform().location.x==0.0 &&
          vehicle.transform().location.y==0.0 &&
          vehicle.transform().location.z==0.0) {
        std::string error_msg(
            "PlanningNode::createSnapshot(): "
            "an agent vehicle is set back to origin.\n");
        std::string agent_msg = (
            boost::format("Agent ID: %lu\n") % vehicle.id()).str();
        throw std::runtime_error(error_msg + agent_msg);
      }

      agent_vehicles.insert(std::make_pair(agent.first, vehicle));
    }

    // Create the snapshot.
    return boost::make_shared<planner::Snapshot>(
        ego_vehicle, agent_vehicles, router_, map_, fast_map_);
  }

  /// Get the carla vehicle by ID.
  boost::shared_ptr<CarlaVehicle> carlaVehicle(const size_t id) const {
    boost::shared_ptr<CarlaVehicle> vehicle =
      boost::dynamic_pointer_cast<CarlaVehicle>(world_->GetActor(id));
    if (!vehicle) {
      throw std::runtime_error(
          "PlanningNode::carlaVehicle(): "
          "Cannot get the required vehicle in the carla server.");
    }
    return vehicle;
  }

  /// Get the carla vehicle transform by the vehicle ID.
  CarlaTransform carlaVehicleTransform(const size_t id) const {
    return carlaVehicle(id)->GetTransform();
  }

  /// Get the carla waypoint a vehicle is at by its ID.
  boost::shared_ptr<CarlaWaypoint> carlaVehicleWaypoint(const size_t id) const {
    return fast_map_->waypoint(carlaVehicleTransform(id).location);
  }

  /// Get the policy for the ego vehicle.
  template<typename GoalConstPtr>
  std::pair<size_t, double> egoPolicy(const GoalConstPtr& goal) const {
    return std::make_pair(goal->ego_policy.id, goal->ego_policy.speed);
  }

  /// Get the speed for the ego vehicle.
  template<typename GoalConstPtr>
  std::pair<size_t, double> egoSpeed(const GoalConstPtr& goal) const {
    return std::make_pair(goal->ego_speed.id, goal->ego_speed.speed);
  }

  /// Get the policy for all agent vehicles.
  template<typename GoalConstPtr>
  std::unordered_map<size_t, double> agentPolicies(const GoalConstPtr& goal) const {
    std::unordered_map<size_t, double> agents;
    for (const auto& agent_policy : goal->agent_policies)
      agents[agent_policy.id] = agent_policy.speed;
    return agents;
  }

  /// Get the speed for all agent vehicles.
  template<typename GoalConstPtr>
  std::unordered_map<size_t, double> agentSpeed(const GoalConstPtr& goal) const {
    std::unordered_map<size_t, double> agents;
    for (const auto& agent_speed : goal->agent_speed)
      agents[agent_speed.id] = agent_speed.speed;
    return agents;
  }
}; // End class PlanningNode.

} // End namespace carla.


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
#include <boost/core/noncopyable.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/bind.hpp>

#include <carla/client/Client.h>
#include <carla/client/World.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <conformal_lattice_planner/EgoPlanAction.h>

namespace carla {

class EgoPlanningNode : private boost::noncopyable {

protected:

  using CarlaClient = carla::client::Client;
  using CarlaWorld = carla::client::World;

protected:

  boost::shared_ptr<CarlaClient> client_ = nullptr;

  mutable ros::NodeHandle nh_;
  mutable actionlib::SimpleActionServer<
    conformal_lattice_planner::EgoPlanAction> server_;

public:

  EgoPlanningNode(ros::NodeHandle& nh) :
    nh_(nh),
    server_(nh, "ego_plan", boost::bind(&EgoPlanningNode::executeCallback, this, _1), false) {}

  virtual ~EgoPlanningNode() {}

  virtual bool initialize() = 0;

protected:

  virtual void executeCallback(const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) = 0;

  std::pair<size_t, double> egoPolicy(
      const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) const {
    return std::make_pair(goal->ego_policy.id, goal->ego_policy.desired_speed);
  }

  std::unordered_map<size_t, double> agentPolicies(
      const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) const {
    std::unordered_map<size_t, double> agents;
    for (const auto& agent_policy : goal->agent_policies)
      agents[agent_policy.id] = agent_policy.desired_speed;
    return agents;
  }

}; // End class EgoPlanningNode.

} // End namespace carla.

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

#include <conformal_lattice_planner/loop_router.h>
#include <ros/simulator_node.h>

namespace carla {

class NoTrafficNode : public SimulatorNode {

private:

  using Base = SimulatorNode;
  using This = NoTrafficNode;

public:

  using Ptr = boost::shared_ptr<This>;
  using ConstPtr = boost::shared_ptr<const This>;

protected:

  /// Loop router, the router is predefined on Town04.
  boost::shared_ptr<router::LoopRouter> loop_router_;

public:

  NoTrafficNode(ros::NodeHandle nh) :
    Base(nh), loop_router_(new router::LoopRouter) {}

  virtual bool initialize() override;

protected:

  virtual void spawnVehicles() override;

  boost::optional<size_t> spawnEgoVehicle(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double policy_speed,
      const bool noisy_policy_speed = true,
      const bool noisy_start_speed = true);

  boost::optional<size_t> spawnAgentVehicle(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double policy_speed,
      const bool noisy_policy_speed = true,
      const bool noisy_start_speed = true);

  /// Since there is no agent vehicles in this object, we don't have
  /// to send the goals for the agent vehicles.
  virtual void tickWorld() override {
    world_->Tick();
    publishTraffic();
    sendEgoGoal();
  }

}; // End class NoTrafficNode.

using NoTrafficNodePtr = NoTrafficNode::Ptr;
using NoTrafficNodeConstPtr = NoTrafficNode::ConstPtr;

} // End namespace carla.


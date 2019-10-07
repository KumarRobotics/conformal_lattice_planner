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

class FixedScenarioNode : public SimulatorNode {

private:

  using Base = SimulatorNode;
  using This = FixedScenarioNode;

public:

  using Ptr = boost::shared_ptr<This>;
  using ConstPtr = boost::shared_ptr<const This>;

protected:

  /// Loop router, the router is predefined on Town04.
  boost::shared_ptr<router::LoopRouter> loop_router_;

public:

  FixedScenarioNode(ros::NodeHandle nh) :
    Base(nh), loop_router_(new router::LoopRouter) {}

protected:

  virtual void spawnVehicles() override;

}; // End class FixedScenarioNode.

using FixedScenarioNodePtr = FixedScenarioNode::Ptr;
using FixedScenarioNodeConstPtr = FixedScenarioNode::ConstPtr;

} // End namespace carla.


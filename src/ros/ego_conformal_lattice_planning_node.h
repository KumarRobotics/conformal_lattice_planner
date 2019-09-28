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

#include <ros/ego_planning_node.h>
#include <conformal_lattice_planner/loop_router.h>
#include <conformal_lattice_planner/conformal_lattice_planner.h>

namespace carla {
class EgoConformalLatticePlanningNode : public EgoPlanningNode {

private:

  using Base = EgoPlanningNode;
  using This = EgoConformalLatticePlanningNode;

public:

  using Ptr = boost::shared_ptr<This>;
  using ConstPtr = boost::shared_ptr<const This>;

protected:

  ros::Publisher conformal_lattice_pub_;

public:

  EgoConformalLatticePlanningNode(ros::NodeHandle& nh) : Base(nh) {}

  virtual ~EgoConformalLatticePlanningNode() {}

  virtual bool initialize() override;

protected:

  virtual void executeCallback(
      const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) override;

}; // End class EgoConformalLatticePlanningNode.

using EgoConformalLatticePlanningNodePtr = EgoConformalLatticePlanningNode::Ptr;
using EgoConformalLatticePlanningNodeConstPtr = EgoConformalLatticePlanningNode::ConstPtr;

} // End namespace carla.

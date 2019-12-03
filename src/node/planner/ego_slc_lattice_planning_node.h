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

#include <boost/smart_ptr.hpp>
#include <boost/optional.hpp>
#include <actionlib/server/simple_action_server.h>

#include <conformal_lattice_planner/EgoPlanAction.h>
#include <planner/common/vehicle_speed_planner.h>
#include <planner/slc_lattice_planner/slc_lattice_planner.h>
#include <node/planner/planning_node.h>

namespace node {

class EgoSLCLatticePlanningNode : public PlanningNode {

private:

  using Base = PlanningNode;
  using This = EgoSLCLatticePlanningNode;

public:

  using Ptr = boost::shared_ptr<This>;
  using ConstPtr = boost::shared_ptr<const This>;

protected:

  /// Stores the curvature of the ego vehicle at end of last step.
  /// If it is not set, it is initialized with the curvature of the road.
  // FIXME: The variable feels sketchy.
  boost::optional<double> ego_curvature = boost::none;

  boost::shared_ptr<planner::SLCLatticePlanner> path_planner_ = nullptr;
  boost::shared_ptr<planner::VehicleSpeedPlanner> speed_planner_ = nullptr;

  mutable ros::Publisher path_pub_;
  mutable ros::Publisher conformal_lattice_pub_;
  mutable ros::Publisher waypoint_lattice_pub_;

  mutable actionlib::SimpleActionServer<
    conformal_lattice_planner::EgoPlanAction> server_;

public:

  EgoSLCLatticePlanningNode(ros::NodeHandle& nh) :
    Base(nh),
    server_(
        nh,
        "ego_plan",
        boost::bind(&EgoSLCLatticePlanningNode::executeCallback, this, _1),
        false) {}

  virtual ~EgoSLCLatticePlanningNode() {}

  virtual bool initialize() override;

protected:

  boost::shared_ptr<planner::Snapshot> createSnapshot(
      const std::pair<size_t, double>& ego_policy,
      const std::pair<size_t, double>& ego_speed,
      const std::unordered_map<size_t, double>& agent_policies,
      const std::unordered_map<size_t, double>& agent_speed) override;

  virtual void executeCallback(
      const conformal_lattice_planner::EgoPlanGoalConstPtr& goal);

}; // End class EgoSLCLatticePlanningNode.

using EgoSLCLatticePlanningNodePtr = EgoSLCLatticePlanningNode::Ptr;
using EgoSLCLatticePlanningNodeConstPtr = EgoSLCLatticePlanningNode::ConstPtr;

} // End namespace node.


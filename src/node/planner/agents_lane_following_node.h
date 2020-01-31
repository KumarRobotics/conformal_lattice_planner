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

#include <unordered_map>
#include <actionlib/server/simple_action_server.h>
#include <conformal_lattice_planner/AgentPlanAction.h>
#include <node/planner/planning_node.h>

namespace node {

class AgentsLaneFollowingNode : public PlanningNode {

private:

  using Base = PlanningNode;
  using This = AgentsLaneFollowingNode;

public:

  using Ptr = boost::shared_ptr<This>;
  using ConstPtr = boost::shared_ptr<const This>;

protected:

  /// Stores the base policy and noise pair.
  std::unordered_map<size_t, std::pair<double, double>> agent_policy_;

  /// Stores the IDMs for different agents.
  std::unordered_map<size_t, boost::shared_ptr<planner::IntelligentDriverModel>> agent_idm_;

  mutable actionlib::SimpleActionServer<
    conformal_lattice_planner::AgentPlanAction> server_;

public:

  AgentsLaneFollowingNode(ros::NodeHandle& nh) :
    Base(nh),
    server_(nh, "agents_plan", boost::bind(&AgentsLaneFollowingNode::executeCallback, this, _1), false) {}

  virtual ~AgentsLaneFollowingNode() {}

  virtual bool initialize() override;

protected:

  void perturbAgentPolicies(
      const boost::shared_ptr<planner::Snapshot>& snapshot);

  void manageAgentIdms(
      const boost::shared_ptr<planner::Snapshot>& snapshot);

  virtual void executeCallback(
      const conformal_lattice_planner::AgentPlanGoalConstPtr& goal);

}; // End class AgentsLaneFollowingNode.

using AgentsLaneFollowingNodePtr = AgentsLaneFollowingNode::Ptr;
using AgentsLaneFollowingNodeConstPtr = AgentsLaneFollowingNode::ConstPtr;

} // End namespace node.


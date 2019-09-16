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

#include <conformal_lattice_planner/lane_follower.h>
#include <ros/agents_planning_node.h>

namespace carla {

class AgentsLaneFollowingNode : public AgentsPlanningNode {

public:

  using Ptr = boost::shared_ptr<AgentsLaneFollowingNode>;
  using ConstPtr = boost::shared_ptr<AgentsLaneFollowingNode>;

private:

  using Base = AgentsPlanningNode;
  using This = AgentsLaneFollowingNode;

protected:

  boost::shared_ptr<planner::LaneFollower> planner_ = nullptr;

public:

  AgentsLaneFollowingNode(ros::NodeHandle& nh) : Base(nh) {}

  virtual bool initialize() override;

protected:

  virtual void executeCallback(
      const conformal_lattice_planner::AgentPlanGoalConstPtr& goal) override;

}; // End class AgentsLaneFollowingNode.

using AgentsLaneFollowingNodePtr = AgentsLaneFollowingNode::Ptr;
using AgentsLaneFollowingNodeConstPtr = AgentsLaneFollowingNode::ConstPtr;

} // End namespace carla.


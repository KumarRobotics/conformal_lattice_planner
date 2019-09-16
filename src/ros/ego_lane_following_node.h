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
#include <conformal_lattice_planner/lane_follower.h>

namespace carla {

class EgoLaneFollowingNode : public EgoPlanningNode {

private:

  using Base = EgoPlanningNode;
  using This = EgoLaneFollowingNode;

public:

  using Ptr = boost::shared_ptr<EgoLaneFollowingNode>;
  using ConstPtr = boost::shared_ptr<const EgoLaneFollowingNode>;

protected:

  boost::shared_ptr<planner::LaneFollower> planner_ = nullptr;

public:

  EgoLaneFollowingNode(ros::NodeHandle& nh) : Base(nh) {}

  virtual ~EgoLaneFollowingNode() {}

  virtual bool initialize() override;

protected:

  virtual void executeCallback(
      const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) override;

}; // End class EgoLaneFollowingNode.

using EgoLaneFollowingNodePtr = EgoLaneFollowingNode::Ptr;
using EgoLaneFollowingNodeConstPtr = EgoLaneFollowingNode::ConstPtr;

} // End namespace carla.

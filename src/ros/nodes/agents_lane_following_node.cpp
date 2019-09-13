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

#include <vector>
#include <boost/core/noncopyable.hpp>
#include <boost/timer/timer.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <conformal_lattice_planner/lane_follower.h>
#include <conformal_lattice_planner/Policy.h>
#include <conformal_lattice_planner/AgentPlanAction.h>

using namespace std;
namespace bst = boost;
namespace cc = carla::client;
namespace cg = carla::geom;
//namespace crpc = carla::rpc;
//namespace csd = carla::sensor::data;
namespace clp = conformal_lattice_planner;

using namespace planner;
using namespace router;

namespace carla {

class AgentsLaneFollowingNode : private bst::noncopyable {

public:

  using Ptr = bst::shared_ptr<AgentsLaneFollowingNode>;
  using ConstPtr = bst::shared_ptr<const AgentsLaneFollowingNode>;

private:

  SharedPtr<cc::Client> client_ = nullptr;
  SharedPtr<LaneFollower> planner_ = nullptr;

  mutable ros::NodeHandle nh_;
  mutable actionlib::SimpleActionServer<clp::AgentPlanAction> server_;

public:

  AgentsLaneFollowingNode(ros::NodeHandle& nh) :
    nh_(nh),
    server_(nh, "agents_plan", bst::bind(&AgentsLaneFollowingNode::executeCallback, this, _1), false) {}

  bool initialize();

private:

  void executeCallback(const clp::AgentPlanGoalConstPtr& goal);

}; // End namespace AgentsLaneFollowingNode class.

bool AgentsLaneFollowingNode::initialize() {

  bool all_param_exist = true;

  string host = "localhost";
  int port = 2000;
  all_param_exist &= nh_.param<std::string>("host", host, "localhost");
  all_param_exist &= nh_.param<int>("port", port, 2000);

  // Get the world.
  ROS_INFO_NAMED("agents_lane_following_planner", "connect to the server.");
  client_ = bst::make_shared<cc::Client>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  client_->GetWorld();

  // Initialize the planner.
  ROS_INFO_NAMED("agents_lane_following_planner", "initialize lane following planner.");
  double fixed_delta_seconds = 0.05;
  all_param_exist &= nh_.param<double>("fixed_delta_seconds", fixed_delta_seconds, 0.05);
  planner_ = bst::make_shared<LaneFollower>(fixed_delta_seconds);

  // Start the action server.
  ROS_INFO_NAMED("agents_lane_following_planner", "start action server.");
  server_.start();

  ROS_INFO_NAMED("agents_lane_following_planner", "initialization finishes.");
  return all_param_exist;

  return all_param_exist;
}

void AgentsLaneFollowingNode::executeCallback(
    const clp::AgentPlanGoalConstPtr& goal) {
  ROS_INFO_NAMED("agents_lane_following_planner", "executeCallback()");

  // Get the ego ID.
  const size_t ego_id = goal->ego_policy.id;

  // Get the agents IDs.
  std::unordered_set<size_t> agent_ids;
  for (const auto& policy : goal->agent_policies)
    agent_ids.insert(policy.id);

  // Update the world for the planner.
  SharedPtr<cc::World> world = bst::make_shared<cc::World>(client_->GetWorld());
  planner_->updateWorld(world);

  // Update the router for the planner.
  // FIXME: Not really need to do this now.
  planner_->updateRouter(LoopRouter());

  // Update the traffic lattice for the planner.
  std::unordered_set<size_t> all_ids = agent_ids;
  all_ids.insert(ego_id);
  planner_->updateTrafficLattice(all_ids);

  if (planner_->trafficLattice()->vehicles().size() < all_ids.size())
    ROS_WARN_NAMED("ego_lane_following_planner", "missing vehicle.");

  // Plan for every agent vehicle.
  for (const auto& policy : goal->agent_policies)
    planner_->plan(policy.id, policy.desired_speed);

  // Inform the client the result of plan.
  clp::AgentPlanResult result;
  result.success = true;
  server_.setSucceeded(result);

  return;
}

using AgentsLaneFollowingNodePtr = AgentsLaneFollowingNode::Ptr;
using AgentsLaneFollowingNodeConstPtr = AgentsLaneFollowingNode::ConstPtr;

} // End namespace carla.

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  if(ros::console::set_logger_level(
        ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  carla::AgentsLaneFollowingNodePtr planner =
    bst::make_shared<carla::AgentsLaneFollowingNode>(nh);
  if (!planner->initialize()) {
    ROS_ERROR("Cannot initialize the agents lane following planner.");
  }

  ros::spin();
  return 0;
}

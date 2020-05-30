/*
 * Copyright 2020 Ke Sun
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string>
#include <chrono>
#include <unordered_set>

#include <ros/ros.h>
#include <ros/console.h>

#include <planner/common/vehicle_path.h>
#include <planner/common/vehicle_speed_planner.h>
#include <planner/common/utils.h>
#include <planner/lane_follower/lane_follower.h>
#include <node/common/convert_to_visualization_msgs.h>
#include <node/planner/ego_lane_following_node.h>

using namespace router;
using namespace planner;
using namespace planner::lane_follower;

namespace node {

bool EgoLaneFollowingNode::initialize() {

  // Create publishers and subscribers.
  path_pub_ = nh_.advertise<visualization_msgs::Marker>("ego_path", 1, true);

  // Load parameters.
  bool all_param_exist = true;

  std::string host = "localhost";
  int port = 2000;
  all_param_exist &= nh_.param<std::string>("host", host, "localhost");
  all_param_exist &= nh_.param<int>("port", port, 2000);

  // Get the world.
  ROS_INFO_NAMED("ego_planner", "connect to the server.");
  client_ = boost::make_shared<CarlaClient>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  client_->GetWorld();

  // Create world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  map_ = world_->GetMap();
  fast_map_ = boost::make_shared<utils::FastWaypointMap>(map_);

  // Start the action server.
  ROS_INFO_NAMED("ego_planner", "start action server.");
  server_.start();

  ROS_INFO_NAMED("ego_planner", "initialization finishes.");
  return all_param_exist;
}

void EgoLaneFollowingNode::executeCallback(
    const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) {

  ROS_INFO_NAMED("ego_planner", "executeCallback()");

  // Update the carla world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  //map_ = world_->GetMap();

  // Create the current snapshot.
  boost::shared_ptr<Snapshot> snapshot = createSnapshot(goal->snapshot);

  boost::shared_ptr<CarlaWaypoint> ego_waypoint =
    carlaVehicleWaypoint(snapshot->ego().id());

  // Plan path.
  // The range of the lattice is just enough for the ego vehicle.
  boost::shared_ptr<LaneFollower> path_planner =
    boost::make_shared<LaneFollower>(
      map_, fast_map_, carlaVehicleWaypoint(snapshot->ego().id()), 55.0, router_);

  const DiscretePath ego_path =
    path_planner->planPath(snapshot->ego().id(), *snapshot);

  // Plan speed.
  boost::shared_ptr<VehicleSpeedPlanner> speed_planner =
    boost::make_shared<VehicleSpeedPlanner>();
  const double ego_accel = speed_planner->planSpeed(snapshot->ego().id(), *snapshot);

  // Compute the target speed and transform of the ego.
  double dt = 0.05;
  nh_.param<double>("fixed_delta_seconds", dt, 0.05);

  const double movement = snapshot->ego().speed()*dt + 0.5*ego_accel*dt*dt;
  const CarlaTransform updated_transform = ego_path.transformAt(movement).first;
  const double updated_speed = snapshot->ego().speed() + ego_accel*dt;

  ROS_INFO_NAMED("ego_planner", "ego %lu", snapshot->ego().id());
  ROS_INFO_NAMED("ego_planner", "movement:%f", movement);
  ROS_INFO_NAMED("ego_planner", "acceleration:%f", ego_accel);
  ROS_INFO_NAMED("ego_planner", "speed:%f", updated_speed);
  ROS_INFO_NAMED("ego_planner", "transform: x:%f y:%f z:%f r:%f p:%f y:%f",
      updated_transform.location.x,
      updated_transform.location.y,
      updated_transform.location.z,
      updated_transform.rotation.roll,
      updated_transform.rotation.pitch,
      updated_transform.rotation.yaw);

  // Update the transform of the ego in the simulator.
  boost::shared_ptr<CarlaVehicle> ego_vehicle = carlaVehicle(snapshot->ego().id());
  ego_vehicle->SetTransform(updated_transform);
  //ego_vehicle->SetVelocity(updated_transform.GetForwardVector()*updated_speed);

  // Publish the path planned for the ego.
  path_pub_.publish(createEgoPathMsg(ego_path));

  // Inform the client the result of plan.
  planner::Vehicle updated_ego(
      snapshot->ego().id(),
      snapshot->ego().boundingBox(),
      updated_transform,
      updated_speed,
      snapshot->ego().policySpeed(),
      ego_accel,
      utils::curvatureAtWaypoint(map_->GetWaypoint(updated_transform.location), map_));

  conformal_lattice_planner::EgoPlanResult result;
  result.header.stamp = ros::Time::now();
  result.success = true;
  result.path_type = conformal_lattice_planner::EgoPlanResult::LANE_KEEP;
  populateVehicleMsg(updated_ego, result.ego);
  server_.setSucceeded(result);

  return;
}

} // End namespace node.

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  if(ros::console::set_logger_level(
        ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  node::EgoLaneFollowingNodePtr planner =
    boost::make_shared<node::EgoLaneFollowingNode>(nh);
  if (!planner->initialize()) {
    ROS_ERROR("Cannot initialize the ego lane following planner.");
  }

  ros::spin();
  return 0;
}

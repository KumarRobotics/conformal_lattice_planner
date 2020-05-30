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
#include <boost/timer/timer.hpp>
#include <gperftools/profiler.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <node/common/convert_to_visualization_msgs.h>
#include <node/planner/ego_idm_lattice_planning_node.h>

using namespace router;
using namespace planner;
using namespace planner::idm_lattice_planner;

namespace node {

bool EgoIDMLatticePlanningNode::initialize() {

  // Create the publishers.
  path_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "ego_path", 1, true);
  conformal_lattice_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "conformal_lattice", 1, true);
  waypoint_lattice_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "waypoint_lattice", 1, true);

  bool all_param_exist = true;

  std::string host = "localhost";
  int port = 2000;
  all_param_exist &= nh_.param<std::string>("host", host, "localhost");
  all_param_exist &= nh_.param<int>("port", port, 2000);

  // Get the world.
  ROS_INFO_NAMED("ego_planner", "connect to the server.");
  client_ = boost::make_shared<CarlaClient>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  ros::Duration(1.0).sleep();

  // Get the world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  map_ = world_->GetMap();
  fast_map_ = boost::make_shared<utils::FastWaypointMap>(map_);

  // Initialize the path and speed planner.
  boost::shared_ptr<router::LoopRouter> router = boost::make_shared<router::LoopRouter>();
  path_planner_ = boost::make_shared<planner::IDMLatticePlanner>(0.1, 150.0, router, map_, fast_map_);
  speed_planner_ = boost::make_shared<planner::VehicleSpeedPlanner>();

  // Start the action server.
  ROS_INFO_NAMED("ego_planner", "start action server.");
  server_.start();

  ROS_INFO_NAMED("ego_planner", "initialization finishes.");
  return all_param_exist;
}

void EgoIDMLatticePlanningNode::executeCallback(
    const conformal_lattice_planner::EgoPlanGoalConstPtr& goal) {

  ROS_INFO_NAMED("ego_planner", "executeCallback()");

  // Update the carla world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  //map_ = world_->GetMap();

  // Create the current snapshot.
  boost::shared_ptr<Snapshot> snapshot = createSnapshot(goal->snapshot);

  // Plan path.
  ros::Time start_time = ros::Time::now();
  const DiscretePath ego_path = path_planner_->planPath(snapshot->ego().id(), *snapshot);
  ros::Duration path_planning_time = ros::Time::now() - start_time;

  // Publish the station graph.
  //conformal_lattice_pub_.publish(createConformalLatticeMsg(
  //      path_planner_->nodes(), path_planner_->edges()));
  path_pub_.publish(createEgoPathMsg(ego_path));
  //waypoint_lattice_pub_.publish(createWaypointLatticeMsg(path_planner_->waypointLattice()));

  // Plan speed.
  const double ego_accel = speed_planner_->planSpeed(snapshot->ego().id(), *snapshot);

  // Update the ego vehicle in the simulator.
  double dt = 0.05;
  nh_.param<double>("fixed_delta_seconds", dt, 0.05);

  const double movement = snapshot->ego().speed()*dt + 0.5*ego_accel*dt*dt;
  const std::pair<CarlaTransform, double> updated_transform_curvature = ego_path.transformAt(movement);
  const CarlaTransform updated_transform = updated_transform_curvature.first;
  const double updated_curvature = updated_transform_curvature.second;
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

  boost::shared_ptr<CarlaVehicle> ego_vehicle = carlaVehicle(snapshot->ego().id());
  ego_vehicle->SetTransform(updated_transform);
  //ego_vehicle->SetVelocity(updated_transform.GetForwardVector()*updated_speed);

  // Inform the client the result of plan.
  planner::Vehicle updated_ego(
      snapshot->ego().id(),
      snapshot->ego().boundingBox(),
      updated_transform,
      updated_speed,
      snapshot->ego().policySpeed(),
      ego_accel,
      updated_curvature);

  conformal_lattice_planner::EgoPlanResult result;
  result.header.stamp = ros::Time::now();
  result.success = true;
  result.path_type = ego_path.laneChangeType();
  result.planning_time = path_planning_time.toSec();
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

  node::EgoIDMLatticePlanningNodePtr planner =
    boost::make_shared<node::EgoIDMLatticePlanningNode>(nh);
  if (!planner->initialize()) {
    ROS_ERROR("Cannot initialize the ego IDM lattice planner.");
  }

  //ProfilerStart("conformal_lattice_planner.stat");
  ros::spin();
  //ProfilerStop();
  return 0;
}

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

  ros::Time start_time = ros::Time::now();

  // Update the carla world and map.
  world_ = boost::make_shared<CarlaWorld>(client_->GetWorld());
  //map_ = world_->GetMap();

  // Create the current snapshot.
  boost::shared_ptr<Snapshot> snapshot = createSnapshot(goal->snapshot);

  // Plan path.
  const DiscretePath ego_path = path_planner_->planPath(snapshot->ego().id(), *snapshot);

  // Publish the station graph.
  conformal_lattice_pub_.publish(createConformalLatticeMsg(
        path_planner_->nodes(), path_planner_->edges()));
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
  // FIXME: Use the correct action type.
  result.path_type = conformal_lattice_planner::EgoPlanResult::LANE_KEEP;
  populateVehicleMsg(updated_ego, result.ego);
  server_.setSucceeded(result);

  ros::Time end_time = ros::Time::now();
  ROS_INFO_NAMED("ego_planner", "planning time: %f",
      (end_time-start_time).toSec());
  //if ((end_time-start_time).toSec() < 0.25) {
  //  ros::Duration delay(0.25-(end_time-start_time).toSec());
  //  delay.sleep();
  //}

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

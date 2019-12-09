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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <node/planner/planning_node.h>

namespace node {

boost::shared_ptr<planner::Snapshot> PlanningNode::createSnapshot(
    const conformal_lattice_planner::TrafficSnapshot& snapshot_msg) {

  // Create the ego vehicle.
  planner::Vehicle ego_vehicle;
  populateVehicleObj(snapshot_msg.ego, ego_vehicle);

  if (ego_vehicle.transform().location.x==0.0 &&
      ego_vehicle.transform().location.y==0.0 &&
      ego_vehicle.transform().location.z==0.0) {
    std::string error_msg(
        "PlanningNode::createSnapshot(): "
        "ego vehicle is set back to origin.\n");
    std::string ego_msg = (
        boost::format("Ego ID: %lu\n") % ego_vehicle.id()).str();
    throw std::runtime_error(error_msg + ego_msg);
  }

  // Create the agent vehicles.
  std::unordered_map<size_t, planner::Vehicle> agent_vehicles;
  for (const auto& agent : snapshot_msg.agents) {
    planner::Vehicle agent_vehicle;
    populateVehicleObj(agent, agent_vehicle);

    if (agent_vehicle.transform().location.x==0.0 &&
        agent_vehicle.transform().location.y==0.0 &&
        agent_vehicle.transform().location.z==0.0) {
      std::string error_msg(
          "PlanningNode::createSnapshot(): "
          "an agent vehicle is set back to origin.\n");
      std::string agent_msg = (
          boost::format("Agent ID: %lu\n") % agent_vehicle.id()).str();
      throw std::runtime_error(error_msg + agent_msg);
    }

    agent_vehicles[agent_vehicle.id()] = agent_vehicle;
  }

  // Create the snapshot.
  return boost::make_shared<planner::Snapshot>(
      ego_vehicle, agent_vehicles, router_, map_, fast_map_);
}

void PlanningNode::populateVehicleMsg(
    const planner::Vehicle& vehicle_obj,
    conformal_lattice_planner::Vehicle& vehicle_msg) {
  // ID.
  vehicle_msg.id = vehicle_obj.id();
  // Bounding box.
  vehicle_msg.bounding_box.extent.x = vehicle_obj.boundingBox().extent.x;
  vehicle_msg.bounding_box.extent.y = vehicle_obj.boundingBox().extent.y;
  vehicle_msg.bounding_box.extent.z = vehicle_obj.boundingBox().extent.z;
  vehicle_msg.bounding_box.location.x = vehicle_obj.boundingBox().location.x;
  vehicle_msg.bounding_box.location.y = vehicle_obj.boundingBox().location.y;
  vehicle_msg.bounding_box.location.z = vehicle_obj.boundingBox().location.z;
  // Transform.
  vehicle_msg.transform.position.x = vehicle_obj.transform().location.x;
  vehicle_msg.transform.position.y = vehicle_obj.transform().location.y;
  vehicle_msg.transform.position.z = vehicle_obj.transform().location.z;
  tf2::Matrix3x3 tf_mat;
  tf_mat.setRPY(vehicle_obj.transform().rotation.roll /180.0*M_PI,
                vehicle_obj.transform().rotation.pitch/180.0*M_PI,
                vehicle_obj.transform().rotation.yaw  /180.0*M_PI);
  tf2::Quaternion tf_quat;
  tf_mat.getRotation(tf_quat);
  vehicle_msg.transform.orientation = tf2::toMsg(tf_quat);
  // Speed.
  vehicle_msg.speed = vehicle_obj.speed();
  // Acceleration.
  vehicle_msg.acceleration = vehicle_obj.acceleration();
  // Curvature.
  vehicle_msg.curvature = vehicle_obj.curvature();
  // policy speed.
  vehicle_msg.policy_speed = vehicle_obj.policySpeed();
  return;
}

void PlanningNode::populateVehicleObj(
    const conformal_lattice_planner::Vehicle& vehicle_msg,
    planner::Vehicle& vehicle_obj) {
  // ID.
  vehicle_obj.id() = vehicle_msg.id;
  // Bounding box.
  vehicle_obj.boundingBox().extent.x = vehicle_msg.bounding_box.extent.x;
  vehicle_obj.boundingBox().extent.y = vehicle_msg.bounding_box.extent.y;
  vehicle_obj.boundingBox().extent.z = vehicle_msg.bounding_box.extent.z;
  vehicle_obj.boundingBox().location.x = vehicle_msg.bounding_box.location.x;
  vehicle_obj.boundingBox().location.y = vehicle_msg.bounding_box.location.y;
  vehicle_obj.boundingBox().location.z = vehicle_msg.bounding_box.location.z;
  // Transform.
  vehicle_obj.transform().location.x = vehicle_msg.transform.position.x;
  vehicle_obj.transform().location.y = vehicle_msg.transform.position.y;
  vehicle_obj.transform().location.z = vehicle_msg.transform.position.z;

  tf2::Quaternion tf_quat;
  tf2::fromMsg(vehicle_msg.transform.orientation, tf_quat);
  tf2::Matrix3x3 tf_mat(tf_quat);
  double yaw, pitch, roll;
  tf_mat.getRPY(roll, pitch, yaw);
  vehicle_obj.transform().rotation.yaw   = yaw  /M_PI*180.0;
  vehicle_obj.transform().rotation.pitch = pitch/M_PI*180.0;
  vehicle_obj.transform().rotation.roll  = roll /M_PI*180.0;
  // Speed.
  vehicle_obj.speed() = vehicle_msg.speed;
  // Acceleration.
  vehicle_obj.acceleration() = vehicle_msg.acceleration;
  // Curvature.
  vehicle_obj.curvature() = vehicle_msg.curvature;
  // Policy speed.
  vehicle_obj.policySpeed() = vehicle_msg.policy_speed;
  return;
}

} // End namespace node.


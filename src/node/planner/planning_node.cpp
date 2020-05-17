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


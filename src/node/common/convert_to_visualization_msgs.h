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

#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <unordered_map>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>

#include <carla/client/Vehicle.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/geom/Transform.h>
#include <carla/sensor/data/Image.h>

#include <router/loop_router/loop_router.h>
#include <planner/common/waypoint_lattice.h>
#include <planner/common/traffic_lattice.h>
#include <planner/common/traffic_manager.h>
#include <planner/common/utils.h>
#include <planner/common/vehicle_path.h>

namespace node {

sensor_msgs::ImagePtr createImageMsg(
    const boost::shared_ptr<const carla::sensor::data::Image>&);

visualization_msgs::MarkerPtr createWaypointMsg(
    const std::vector<boost::shared_ptr<const carla::client::Waypoint>>&);

visualization_msgs::MarkerPtr createJunctionMsg(
    const carla::client::Map::TopologyList&);

visualization_msgs::MarkerArrayPtr createRoadIdsMsg(
    const std::unordered_map<uint32_t, carla::road::Road>&);

visualization_msgs::MarkerArrayPtr createVehiclesMsg(
    const std::vector<boost::shared_ptr<const carla::client::Vehicle>>&);

visualization_msgs::MarkerArrayPtr createVehicleIdsMsg(
    const std::vector<boost::shared_ptr<const carla::client::Vehicle>>&);

geometry_msgs::TransformStampedPtr createVehicleTransformMsg(
    const boost::shared_ptr<const carla::client::Vehicle>&, const std::string&);

visualization_msgs::MarkerArrayPtr createWaypointLatticeMsg(
    const boost::shared_ptr<const planner::WaypointLattice>&);

visualization_msgs::MarkerArrayPtr createTrafficLatticeMsg(
    const boost::shared_ptr<const planner::TrafficLattice>&);

visualization_msgs::MarkerArrayPtr createTrafficManagerMsg(
    const boost::shared_ptr<const planner::TrafficManager>&);

template<typename Path>
void populatePathMsg(const Path& path, const visualization_msgs::MarkerPtr& path_msg) {

  std::vector<std::pair<carla::geom::Transform, double>>
    transforms = path.samples();

  for (auto& transform : transforms) {
    utils::convertTransformInPlace(transform.first);
    geometry_msgs::Point pt;
    pt.x = transform.first.location.x;
    pt.y = transform.first.location.y;
    pt.z = transform.first.location.z;

    path_msg->points.push_back(pt);
    path_msg->colors.push_back(path_msg->color);
  }

  return;
}

template<typename Path>
visualization_msgs::MarkerPtr createEgoPathMsg(const Path& path) {

  visualization_msgs::MarkerPtr path_msg(new visualization_msgs::Marker);

  std_msgs::ColorRGBA path_color;
  path_color.r = 0.4;
  path_color.g = 1.0;
  path_color.b = 0.4;
  path_color.a = 1.0;

  path_msg->header.stamp = ros::Time::now();
  path_msg->header.frame_id = "map";
  path_msg->ns = "ego_path";
  path_msg->id = 0;
  path_msg->type = visualization_msgs::Marker::LINE_STRIP;
  path_msg->action = visualization_msgs::Marker::ADD;
  path_msg->lifetime = ros::Duration(0.0);
  path_msg->frame_locked = false;
  path_msg->pose.orientation.w = 1.0;
  path_msg->scale.x = 1.8;
  path_msg->scale.y = 1.8;
  path_msg->scale.z = 1.8;
  path_msg->color = path_color;

  populatePathMsg(path, path_msg);
  return path_msg;
}

visualization_msgs::MarkerArrayPtr createConformalLatticeMsg(
    const std::vector<boost::shared_ptr<const planner::WaypointNode>>& nodes,
    const std::vector<planner::ContinuousPath>& edges);

} // End namespace node.

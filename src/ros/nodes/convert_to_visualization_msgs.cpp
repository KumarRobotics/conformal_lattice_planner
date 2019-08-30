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
#include <set>
#include <chrono>
#include <random>
#include <cmath>
#include <unordered_map>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <carla/client/Vehicle.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/geom/Transform.h>
#include <carla/sensor/data/Image.h>

using namespace std;
namespace bst = boost;
namespace cc = carla::client;
namespace cg = carla::geom;
namespace crpc = carla::rpc;
namespace csd = carla::sensor::data;

namespace carla {

visualization_msgs::MarkerPtr createWaypointMsg(
    const vector<carla::SharedPtr<cc::Waypoint>>& waypoints) {

  std_msgs::ColorRGBA color;
  color.r = 0.5;
  color.g = 0.5;
  color.b = 1.0;
  color.a = 1.0;

  visualization_msgs::MarkerPtr waypoints_msg(new visualization_msgs::Marker);
  waypoints_msg->header.stamp = ros::Time::now();
  waypoints_msg->header.frame_id = "map";
  waypoints_msg->ns = "waypoints";
  waypoints_msg->id = 0;
  waypoints_msg->type = visualization_msgs::Marker::SPHERE_LIST;
  waypoints_msg->action = visualization_msgs::Marker::ADD;
  waypoints_msg->lifetime = ros::Duration(0.0);
  waypoints_msg->frame_locked = false;
  waypoints_msg->pose.orientation.w = 1.0;
  waypoints_msg->scale.x = 1.0;
  waypoints_msg->scale.y = 1.0;
  waypoints_msg->scale.z = 1.0;
  waypoints_msg->color = color;

  for (const auto& wp : waypoints) {
    geometry_msgs::Point pt;
    pt.x = wp->GetTransform().location.x;
    pt.y = wp->GetTransform().location.y;
    pt.z = wp->GetTransform().location.z;

    waypoints_msg->points.push_back(pt);
    waypoints_msg->colors.push_back(color);
  }

  return waypoints_msg;
}

visualization_msgs::MarkerPtr createJunctionMsg(
    const cc::Map::TopologyList& waypoint_pairs) {

  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 0.3;
  color.a = 1.0;

  // Collect the unique waypoints in the topology list.
  set<size_t> waypoint_ids;
  std::vector<carla::SharedPtr<cc::Waypoint>> waypoints;

  for (const auto& wp_pair : waypoint_pairs) {
    if (waypoint_ids.find(wp_pair.first->GetId()) == waypoint_ids.end())
      waypoints.push_back(wp_pair.first);
    if (waypoint_ids.find(wp_pair.second->GetId()) == waypoint_ids.end())
      waypoints.push_back(wp_pair.second);
  }

  visualization_msgs::MarkerPtr junctions_msg(new visualization_msgs::Marker);
  junctions_msg->header.stamp = ros::Time::now();
  junctions_msg->header.frame_id = "map";
  junctions_msg->ns = "junctions";
  junctions_msg->id = 0;
  junctions_msg->type = visualization_msgs::Marker::SPHERE_LIST;
  junctions_msg->action = visualization_msgs::Marker::ADD;
  junctions_msg->lifetime = ros::Duration(0.0);
  junctions_msg->frame_locked = false;
  junctions_msg->pose.orientation.w = 1.0;
  junctions_msg->scale.x = 2.0;
  junctions_msg->scale.y = 2.0;
  junctions_msg->scale.z = 2.0;
  junctions_msg->color = color;

  for (const auto& wp : waypoints) {
    geometry_msgs::Point pt;
    pt.x = wp->GetTransform().location.x;
    pt.y = wp->GetTransform().location.y;
    pt.z = wp->GetTransform().location.z;

    junctions_msg->points.push_back(pt);
    junctions_msg->colors.push_back(color);
  }

  return junctions_msg;
}

visualization_msgs::MarkerPtr createVehicleMarkerMsg(
    const carla::SharedPtr<cc::Actor>& vehicle) {

  static unordered_map<size_t, std_msgs::ColorRGBA> vehicle_colors;
  std_msgs::ColorRGBA fixed_color;

  if (vehicle_colors.find(vehicle->GetId()) != vehicle_colors.end()) {
    fixed_color = vehicle_colors[vehicle->GetId()];
  } else {
    const size_t seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine rand_gen(seed);
    uniform_real_distribution<double> uni_dist(0.0, 1.0);
    fixed_color.r = uni_dist(rand_gen);
    fixed_color.g = uni_dist(rand_gen);
    fixed_color.b = uni_dist(rand_gen);
    fixed_color.a = 1.0;
    vehicle_colors[vehicle->GetId()] = fixed_color;
  }

  visualization_msgs::MarkerPtr vehicle_msg(new visualization_msgs::Marker);
  vehicle_msg->header.stamp = ros::Time::now();
  vehicle_msg->header.frame_id = "map";
  vehicle_msg->ns = "Vehicles";
  vehicle_msg->id = vehicle->GetId();
  vehicle_msg->type = visualization_msgs::Marker::CUBE;
  vehicle_msg->action = visualization_msgs::Marker::ADD;
  vehicle_msg->lifetime = ros::Duration(0.0);
  vehicle_msg->frame_locked = false;
  vehicle_msg->scale.x = 4.0;
  vehicle_msg->scale.y = 2.0;
  vehicle_msg->scale.z = 1.5;
  vehicle_msg->color = fixed_color;

  cg::Transform transform = vehicle->GetTransform();
  vehicle_msg->pose.position.x = transform.location.x;
  vehicle_msg->pose.position.y = transform.location.y;
  vehicle_msg->pose.position.z = transform.location.z;
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(
      transform.rotation.roll/180.0*M_PI,
      -transform.rotation.pitch/180.0*M_PI,
      transform.rotation.yaw/180.0*M_PI);
  tf2::convert(tf2_quat, vehicle_msg->pose.orientation);

  return vehicle_msg;
}

geometry_msgs::TransformStampedPtr createVehicleTransformMsg(
    const carla::SharedPtr<cc::Actor>& vehicle,
    const std::string& vehicle_frame_id) {

  geometry_msgs::TransformStampedPtr transform_msg(
      new geometry_msgs::TransformStamped);
  cg::Transform transform = vehicle->GetTransform();

  transform_msg->header.stamp = ros::Time::now();
  transform_msg->header.frame_id = "map";
  transform_msg->child_frame_id = vehicle_frame_id;

  transform_msg->transform.translation.x = transform.location.x;
  transform_msg->transform.translation.y = transform.location.y;
  transform_msg->transform.translation.z = transform.location.z;

  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(
      transform.rotation.roll/180.0*M_PI,
      -transform.rotation.pitch/180.0*M_PI,
      transform.rotation.yaw/180.0*M_PI);
  transform_msg->transform.rotation.x = tf2_quat.x();
  transform_msg->transform.rotation.y = tf2_quat.y();
  transform_msg->transform.rotation.z = tf2_quat.z();
  transform_msg->transform.rotation.w = tf2_quat.w();

  return transform_msg;
}

sensor_msgs::ImagePtr createImageMsg(const carla::SharedPtr<csd::Image>& img) {

  sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);

  image_msg->header.stamp = ros::Time::now();
  image_msg->header.frame_id = "following_camera";
  image_msg->height = img->GetHeight();
  image_msg->width = img->GetWidth();
  image_msg->encoding = "rgba8";
  // FIXME: Is this bigendian or not?
  //        Since the data is really uint8, it probably does not matter.
  image_msg->is_bigendian = false;
  image_msg->step = 4 * image_msg->width;

  // Fill in the data.
  // FIXME: Is there a more efficient to copy the data cover?
  //        memcpy() won't work since the order of RGBA is different
  //        on both ends.
  image_msg->data.resize(image_msg->height*image_msg->step);
  for (size_t i = 0; i < image_msg->height*image_msg->width; ++i) {
    image_msg->data[4*i+0] = (img->data()+i)->r;
    image_msg->data[4*i+1] = (img->data()+i)->g;
    image_msg->data[4*i+2] = (img->data()+i)->b;
    image_msg->data[4*i+3] = (img->data()+i)->a;
  }

  return image_msg;
}

} // End namespace carla.

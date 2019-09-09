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
#include <unordered_set>

#include <boost/functional/hash.hpp>
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

#include <conformal_lattice_planner/waypoint_lattice.h>
#include <conformal_lattice_planner/traffic_lattice.h>
#include <conformal_lattice_planner/utils.h>

using namespace std;
namespace bst = boost;
namespace cc = carla::client;
namespace cg = carla::geom;
namespace crpc = carla::rpc;
namespace csd = carla::sensor::data;
namespace cr = carla::road;
using namespace planner;

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
    cg::Transform transform = utils::convertTransform(wp->GetTransform());
    geometry_msgs::Point pt;
    pt.x = transform.location.x;
    pt.y = transform.location.y;
    pt.z = transform.location.z;

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
    cg::Transform transform = utils::convertTransform(wp->GetTransform());
    geometry_msgs::Point pt;
    pt.x = transform.location.x;
    pt.y = transform.location.y;
    pt.z = transform.location.z;

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

  cg::Transform transform = utils::convertTransform(vehicle->GetTransform());
  vehicle_msg->pose.position.x = transform.location.x;
  vehicle_msg->pose.position.y = transform.location.y;
  vehicle_msg->pose.position.z = transform.location.z;
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(
      transform.rotation.roll/180.0*M_PI,
      transform.rotation.pitch/180.0*M_PI,
      transform.rotation.yaw/180.0*M_PI);
  tf2::convert(tf2_quat, vehicle_msg->pose.orientation);

  return vehicle_msg;
}

geometry_msgs::TransformStampedPtr createVehicleTransformMsg(
    const carla::SharedPtr<cc::Actor>& vehicle,
    const std::string& vehicle_frame_id) {

  geometry_msgs::TransformStampedPtr transform_msg(
      new geometry_msgs::TransformStamped);
  cg::Transform transform = utils::convertTransform(vehicle->GetTransform());

  transform_msg->header.stamp = ros::Time::now();
  transform_msg->header.frame_id = "map";
  transform_msg->child_frame_id = vehicle_frame_id;

  transform_msg->transform.translation.x = transform.location.x;
  transform_msg->transform.translation.y = transform.location.y;
  transform_msg->transform.translation.z = transform.location.z;

  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(
      transform.rotation.roll/180.0*M_PI,
      transform.rotation.pitch/180.0*M_PI,
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

visualization_msgs::MarkerArrayPtr createWaypointLatticeMsg(
    const bst::shared_ptr<const WaypointNode>& lattice_entry) {

  std_msgs::ColorRGBA color;
  color.r = 0.2;
  color.g = 0.8;
  color.b = 0.2;
  color.a = 1.0;

  //printf("create traffice lattice node and edge msg.\n");
  visualization_msgs::MarkerPtr lattice_node_msg(new visualization_msgs::Marker);
  visualization_msgs::MarkerPtr lattice_edge_msg(new visualization_msgs::Marker);

  lattice_node_msg->header.stamp = ros::Time::now();
  lattice_node_msg->header.frame_id = "map";
  lattice_node_msg->ns = "waypoint_lattice_node";
  lattice_node_msg->id = 0;
  lattice_node_msg->type = visualization_msgs::Marker::SPHERE_LIST;
  lattice_node_msg->action = visualization_msgs::Marker::ADD;
  lattice_node_msg->lifetime = ros::Duration(0.0);
  lattice_node_msg->frame_locked = false;
  lattice_node_msg->pose.orientation.w = 1.0;
  lattice_node_msg->scale.x = 0.5;
  lattice_node_msg->scale.y = 0.5;
  lattice_node_msg->scale.z = 0.5;
  lattice_node_msg->color = color;

  lattice_edge_msg->header.stamp = ros::Time::now();
  lattice_edge_msg->header.frame_id = "map";
  lattice_edge_msg->ns = "waypoint_lattice_edge";
  lattice_edge_msg->id = 0;
  lattice_edge_msg->type = visualization_msgs::Marker::LINE_LIST;
  lattice_edge_msg->action = visualization_msgs::Marker::ADD;
  lattice_edge_msg->lifetime = ros::Duration(0.0);
  lattice_edge_msg->frame_locked = false;
  lattice_edge_msg->pose.orientation.w = 1.0;
  lattice_edge_msg->scale.x = 0.1;
  lattice_edge_msg->scale.y = 0.0;
  lattice_edge_msg->scale.z = 0.0;
  lattice_edge_msg->color = color;

  std::unordered_set<size_t> processed_waypoints;
  std::unordered_set<size_t> drawed_waypoints;
  std::unordered_set<size_t> drawed_waypoint_pairs;
  std::queue<bst::shared_ptr<const WaypointNode>> unprocessed_nodes;
  unprocessed_nodes.push(lattice_entry);

  //printf("Fill in the nodes and edges.\n");
  while (!unprocessed_nodes.empty()) {

    //printf("processed/unprocessed waypoints #: %lu/%lu\n", processed_waypoints.size(), unprocessed_nodes.size());
    // Get the center point to be processed.
    bst::shared_ptr<const WaypointNode> center = unprocessed_nodes.front();
    unprocessed_nodes.pop();
    processed_waypoints.insert(center->waypoint()->GetId());

    //printf("center: %lu\n", center->waypoint()->GetId());
    //for (const auto& pt : unprocessed_nodes) std::printf("%lu\n", pt->waypoint()->GetId());

    cg::Transform center_transform = utils::convertTransform(center->waypoint()->GetTransform());
    geometry_msgs::Point center_pt;
    center_pt.x = center_transform.location.x;
    center_pt.y = center_transform.location.y;
    center_pt.z = center_transform.location.z;

    // Add the front node.
    bst::shared_ptr<const WaypointNode> front = center->front();
    if (front) {
      cg::Transform transform = utils::convertTransform(front->waypoint()->GetTransform());
      geometry_msgs::Point pt;
      pt.x = transform.location.x;
      pt.y = transform.location.y;
      pt.z = transform.location.z;

      if (processed_waypoints.count(front->waypoint()->GetId()) == 0) {
        //printf("Add front: %lu\n", front->waypoint()->GetId());
        unprocessed_nodes.push(front);
        processed_waypoints.insert(front->waypoint()->GetId());
      }

      if (drawed_waypoints.count(front->waypoint()->GetId()) == 0) {
        lattice_node_msg->points.push_back(pt);
        lattice_node_msg->colors.push_back(color);
        drawed_waypoints.insert(front->waypoint()->GetId());
      }

      size_t pair_id0 = 0, pair_id1 = 0;
      utils::hashCombine(pair_id0, center->waypoint()->GetId(), front->waypoint()->GetId());
      utils::hashCombine(pair_id1, front->waypoint()->GetId(), center->waypoint()->GetId());
      if (drawed_waypoint_pairs.count(pair_id0) == 0 && drawed_waypoint_pairs.count(pair_id1) == 0) {
        lattice_edge_msg->points.push_back(center_pt);
        lattice_edge_msg->points.push_back(pt);
        lattice_edge_msg->colors.push_back(color);
        lattice_edge_msg->colors.push_back(color);

        drawed_waypoint_pairs.insert(pair_id0);
        drawed_waypoint_pairs.insert(pair_id1);
      }
    }

    // Add the left node.
    bst::shared_ptr<const WaypointNode> left = center->left();
    if (left) {
      cg::Transform transform = utils::convertTransform(left->waypoint()->GetTransform());
      geometry_msgs::Point pt;
      pt.x = transform.location.x;
      pt.y = transform.location.y;
      pt.z = transform.location.z;

      if (processed_waypoints.count(left->waypoint()->GetId()) == 0) {
        //printf("Add left: %lu\n", left->waypoint()->GetId());
        unprocessed_nodes.push(left);
        processed_waypoints.insert(left->waypoint()->GetId());
      }

      if (drawed_waypoints.count(left->waypoint()->GetId()) == 0) {
        lattice_node_msg->points.push_back(pt);
        lattice_node_msg->colors.push_back(color);
        drawed_waypoints.insert(left->waypoint()->GetId());
      }

      size_t pair_id0 = 0, pair_id1 = 0;
      utils::hashCombine(pair_id0, center->waypoint()->GetId(), left->waypoint()->GetId());
      utils::hashCombine(pair_id1, left->waypoint()->GetId(), center->waypoint()->GetId());
      if (drawed_waypoint_pairs.count(pair_id0) == 0 && drawed_waypoint_pairs.count(pair_id1) == 0) {
        lattice_edge_msg->points.push_back(center_pt);
        lattice_edge_msg->points.push_back(pt);
        lattice_edge_msg->colors.push_back(color);
        lattice_edge_msg->colors.push_back(color);

        drawed_waypoint_pairs.insert(pair_id0);
        drawed_waypoint_pairs.insert(pair_id1);
      }
    }

    // Add the right node.
    bst::shared_ptr<const WaypointNode> right = center->right();
    if (right) {
      cg::Transform transform = utils::convertTransform(right->waypoint()->GetTransform());
      geometry_msgs::Point pt;
      pt.x = transform.location.x;
      pt.y = transform.location.y;
      pt.z = transform.location.z;

      if (processed_waypoints.count(right->waypoint()->GetId()) == 0) {
        //printf("Add right: %lu\n", right->waypoint()->GetId());
        unprocessed_nodes.push(right);
        processed_waypoints.insert(right->waypoint()->GetId());
      }

      if (drawed_waypoints.count(right->waypoint()->GetId()) == 0) {
        lattice_node_msg->points.push_back(pt);
        lattice_node_msg->colors.push_back(color);
        drawed_waypoints.insert(right->waypoint()->GetId());
      }

      size_t pair_id0 = 0, pair_id1 = 0;
      utils::hashCombine(pair_id0, center->waypoint()->GetId(), right->waypoint()->GetId());
      utils::hashCombine(pair_id1, right->waypoint()->GetId(), center->waypoint()->GetId());
      if (drawed_waypoint_pairs.count(pair_id0) == 0 && drawed_waypoint_pairs.count(pair_id1) == 0) {
        lattice_edge_msg->points.push_back(center_pt);
        lattice_edge_msg->points.push_back(pt);
        lattice_edge_msg->colors.push_back(color);
        lattice_edge_msg->colors.push_back(color);

        drawed_waypoint_pairs.insert(pair_id0);
        drawed_waypoint_pairs.insert(pair_id1);
      }
    }
  }

  //std::printf("sphere list size: %lu, %lu\n",
  //    drawed_waypoints.size(),
  //    lattice_node_msg->points.size());
  //std::printf("line list size: %lu, %lu\n",
  //    drawed_waypoint_pairs.size(),
  //    lattice_edge_msg->points.size());

  visualization_msgs::MarkerArrayPtr traffic_lattice_msg(
      new visualization_msgs::MarkerArray);
  traffic_lattice_msg->markers.push_back(*lattice_node_msg);
  traffic_lattice_msg->markers.push_back(*lattice_edge_msg);

  return traffic_lattice_msg;
}

visualization_msgs::MarkerArrayPtr createTrafficLatticeMsg(
    const bst::shared_ptr<const WaypointNodeWithVehicle>& lattice_entry) {

  std_msgs::ColorRGBA color;
  color.r = 0.4;
  color.g = 0.7;
  color.b = 1.0;
  color.a = 1.0;

  std_msgs::ColorRGBA special_color;
  special_color.r = 1.0;
  special_color.g = 0.4;
  special_color.b = 0.4;
  special_color.a = 1.0;

  //printf("create traffice lattice node and edge msg.\n");
  visualization_msgs::MarkerPtr lattice_node_msg(new visualization_msgs::Marker);
  visualization_msgs::MarkerPtr lattice_edge_msg(new visualization_msgs::Marker);

  lattice_node_msg->header.stamp = ros::Time::now();
  lattice_node_msg->header.frame_id = "map";
  lattice_node_msg->ns = "traffic_lattice_node";
  lattice_node_msg->id = 0;
  lattice_node_msg->type = visualization_msgs::Marker::SPHERE_LIST;
  lattice_node_msg->action = visualization_msgs::Marker::ADD;
  lattice_node_msg->lifetime = ros::Duration(0.0);
  lattice_node_msg->frame_locked = false;
  lattice_node_msg->pose.orientation.w = 1.0;
  lattice_node_msg->scale.x = 1.0;
  lattice_node_msg->scale.y = 1.0;
  lattice_node_msg->scale.z = 1.0;
  lattice_node_msg->color = color;

  lattice_edge_msg->header.stamp = ros::Time::now();
  lattice_edge_msg->header.frame_id = "map";
  lattice_edge_msg->ns = "traffic_lattice_edge";
  lattice_edge_msg->id = 0;
  lattice_edge_msg->type = visualization_msgs::Marker::LINE_LIST;
  lattice_edge_msg->action = visualization_msgs::Marker::ADD;
  lattice_edge_msg->lifetime = ros::Duration(0.0);
  lattice_edge_msg->frame_locked = false;
  lattice_edge_msg->pose.orientation.w = 1.0;
  lattice_edge_msg->scale.x = 0.1;
  lattice_edge_msg->scale.y = 0.0;
  lattice_edge_msg->scale.z = 0.0;
  lattice_edge_msg->color = color;

  std::unordered_set<size_t> processed_waypoints;
  std::unordered_set<size_t> drawed_waypoints;
  std::unordered_set<size_t> drawed_waypoint_pairs;
  std::queue<bst::shared_ptr<const WaypointNodeWithVehicle>> unprocessed_nodes;
  unprocessed_nodes.push(lattice_entry);

  //printf("Fill in the nodes and edges.\n");
  while (!unprocessed_nodes.empty()) {

    //printf("processed/unprocessed waypoints #: %lu/%lu\n", processed_waypoints.size(), unprocessed_nodes.size());
    // Get the center point to be processed.
    bst::shared_ptr<const WaypointNodeWithVehicle> center = unprocessed_nodes.front();
    unprocessed_nodes.pop();
    processed_waypoints.insert(center->waypoint()->GetId());

    //printf("center: %lu\n", center->waypoint()->GetId());
    //for (const auto& pt : unprocessed_nodes) std::printf("%lu\n", pt->waypoint()->GetId());

    cg::Transform center_transform = utils::convertTransform(center->waypoint()->GetTransform());
    geometry_msgs::Point center_pt;
    center_pt.x = center_transform.location.x;
    center_pt.y = center_transform.location.y;
    center_pt.z = center_transform.location.z;

    // Add the front node.
    bst::shared_ptr<const WaypointNodeWithVehicle> front = center->front();
    if (front) {
      cg::Transform transform = utils::convertTransform(front->waypoint()->GetTransform());
      geometry_msgs::Point pt;
      pt.x = transform.location.x;
      pt.y = transform.location.y;
      pt.z = transform.location.z;

      if (processed_waypoints.count(front->waypoint()->GetId()) == 0) {
        //printf("Add front: %lu\n", front->waypoint()->GetId());
        unprocessed_nodes.push(front);
        processed_waypoints.insert(front->waypoint()->GetId());
      }

      if (drawed_waypoints.count(front->waypoint()->GetId()) == 0) {
        lattice_node_msg->points.push_back(pt);
        drawed_waypoints.insert(front->waypoint()->GetId());
        if (front->vehicle()) lattice_node_msg->colors.push_back(special_color);
        else lattice_node_msg->colors.push_back(color);
      }

      size_t pair_id0 = 0, pair_id1 = 0;
      utils::hashCombine(pair_id0, center->waypoint()->GetId(), front->waypoint()->GetId());
      utils::hashCombine(pair_id1, front->waypoint()->GetId(), center->waypoint()->GetId());
      if (drawed_waypoint_pairs.count(pair_id0) == 0 && drawed_waypoint_pairs.count(pair_id1) == 0) {
        lattice_edge_msg->points.push_back(center_pt);
        lattice_edge_msg->points.push_back(pt);
        lattice_edge_msg->colors.push_back(color);
        lattice_edge_msg->colors.push_back(color);

        drawed_waypoint_pairs.insert(pair_id0);
        drawed_waypoint_pairs.insert(pair_id1);
      }
    }

    // Add the left node.
    bst::shared_ptr<const WaypointNodeWithVehicle> left = center->left();
    if (left) {
      cg::Transform transform = utils::convertTransform(left->waypoint()->GetTransform());
      geometry_msgs::Point pt;
      pt.x = transform.location.x;
      pt.y = transform.location.y;
      pt.z = transform.location.z;

      if (processed_waypoints.count(left->waypoint()->GetId()) == 0) {
        //printf("Add left: %lu\n", left->waypoint()->GetId());
        unprocessed_nodes.push(left);
        processed_waypoints.insert(left->waypoint()->GetId());
      }

      if (drawed_waypoints.count(left->waypoint()->GetId()) == 0) {
        lattice_node_msg->points.push_back(pt);
        drawed_waypoints.insert(left->waypoint()->GetId());
        if (left->vehicle()) lattice_node_msg->colors.push_back(special_color);
        else lattice_node_msg->colors.push_back(color);
      }

      size_t pair_id0 = 0, pair_id1 = 0;
      utils::hashCombine(pair_id0, center->waypoint()->GetId(), left->waypoint()->GetId());
      utils::hashCombine(pair_id1, left->waypoint()->GetId(), center->waypoint()->GetId());
      if (drawed_waypoint_pairs.count(pair_id0) == 0 && drawed_waypoint_pairs.count(pair_id1) == 0) {
        lattice_edge_msg->points.push_back(center_pt);
        lattice_edge_msg->points.push_back(pt);
        lattice_edge_msg->colors.push_back(color);
        lattice_edge_msg->colors.push_back(color);

        drawed_waypoint_pairs.insert(pair_id0);
        drawed_waypoint_pairs.insert(pair_id1);
      }
    }

    // Add the right node.
    bst::shared_ptr<const WaypointNodeWithVehicle> right = center->right();
    if (right) {
      cg::Transform transform = utils::convertTransform(right->waypoint()->GetTransform());
      geometry_msgs::Point pt;
      pt.x = transform.location.x;
      pt.y = transform.location.y;
      pt.z = transform.location.z;

      if (processed_waypoints.count(right->waypoint()->GetId()) == 0) {
        //printf("Add right: %lu\n", right->waypoint()->GetId());
        unprocessed_nodes.push(right);
        processed_waypoints.insert(right->waypoint()->GetId());
      }

      if (drawed_waypoints.count(right->waypoint()->GetId()) == 0) {
        lattice_node_msg->points.push_back(pt);
        drawed_waypoints.insert(right->waypoint()->GetId());
        if (right->vehicle()) lattice_node_msg->colors.push_back(special_color);
        else lattice_node_msg->colors.push_back(color);
      }

      size_t pair_id0 = 0, pair_id1 = 0;
      utils::hashCombine(pair_id0, center->waypoint()->GetId(), right->waypoint()->GetId());
      utils::hashCombine(pair_id1, right->waypoint()->GetId(), center->waypoint()->GetId());
      if (drawed_waypoint_pairs.count(pair_id0) == 0 && drawed_waypoint_pairs.count(pair_id1) == 0) {
        lattice_edge_msg->points.push_back(center_pt);
        lattice_edge_msg->points.push_back(pt);
        lattice_edge_msg->colors.push_back(color);
        lattice_edge_msg->colors.push_back(color);

        drawed_waypoint_pairs.insert(pair_id0);
        drawed_waypoint_pairs.insert(pair_id1);
      }
    }
  }

  //std::printf("sphere list size: %lu, %lu\n",
  //    drawed_waypoints.size(),
  //    lattice_node_msg->points.size());
  //std::printf("line list size: %lu, %lu\n",
  //    drawed_waypoint_pairs.size(),
  //    lattice_edge_msg->points.size());

  visualization_msgs::MarkerArrayPtr traffic_lattice_msg(
      new visualization_msgs::MarkerArray);
  traffic_lattice_msg->markers.push_back(*lattice_node_msg);
  traffic_lattice_msg->markers.push_back(*lattice_edge_msg);

  return traffic_lattice_msg;
}

visualization_msgs::MarkerArrayPtr createRoadIdsMsg(
    const std::unordered_map<uint32_t, cr::Road>& roads) {

  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 1.0;

  visualization_msgs::MarkerArrayPtr roads_msg(
      new visualization_msgs::MarkerArray);

  for (const auto& item : roads) {

    const uint32_t id = item.first;
    const cr::Road& road = item.second;

    const double length = road.GetLength();
    // Weird enough, the return location is already in right hand coordinate system.
    cg::Location location = road.GetDirectedPointIn(length/2.0).location;
    //std::printf("road:%lu length:%f\n", road.GetId(), length);
    //utils::convertLocationInPlace(location);

    visualization_msgs::MarkerPtr road_msg(new visualization_msgs::Marker);
    road_msg->header.stamp = ros::Time::now();
    road_msg->header.frame_id = "map";
    road_msg->ns = "road";
    road_msg->id = id;
    road_msg->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    road_msg->action = visualization_msgs::Marker::ADD;
    road_msg->lifetime = ros::Duration(0.0);
    road_msg->frame_locked = false;
    road_msg->pose.orientation.w = 1.0;
    road_msg->pose.position.x = location.x;
    road_msg->pose.position.y = location.y;
    road_msg->pose.position.z = location.z;
    road_msg->scale.x = 4.0;
    road_msg->scale.y = 4.0;
    road_msg->scale.z = 4.0;
    road_msg->color = color;
    road_msg->text = std::to_string(id);

    roads_msg->markers.push_back(*road_msg);
  }

  return roads_msg;
}

visualization_msgs::MarkerArrayPtr createVehicleIdsMsg(
    const std::unordered_map<size_t, cg::Transform>& vehicles) {

  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 1.0;

  visualization_msgs::MarkerArrayPtr vehicles_msg(
      new visualization_msgs::MarkerArray);

  for (const auto& vehicle : vehicles) {
    const size_t id = vehicle.first;
    cg::Location location = vehicle.second.location;
    utils::convertLocationInPlace(location);

    visualization_msgs::MarkerPtr vehicle_msg(new visualization_msgs::Marker);
    vehicle_msg->header.stamp = ros::Time::now();
    vehicle_msg->header.frame_id = "map";
    vehicle_msg->ns = "vehicles";
    vehicle_msg->id = id;
    vehicle_msg->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    vehicle_msg->action = visualization_msgs::Marker::ADD;
    vehicle_msg->lifetime = ros::Duration(0.0);
    vehicle_msg->frame_locked = false;
    vehicle_msg->pose.orientation.w = 1.0;
    vehicle_msg->pose.position.x = location.x;
    vehicle_msg->pose.position.y = location.y;
    vehicle_msg->pose.position.z = location.z + 1.5;
    vehicle_msg->scale.x = 2.0;
    vehicle_msg->scale.y = 2.0;
    vehicle_msg->scale.z = 2.0;
    vehicle_msg->color = color;
    vehicle_msg->text = std::to_string(id);

    vehicles_msg->markers.push_back(*vehicle_msg);
  }

  return vehicles_msg;
}

} // End namespace carla.

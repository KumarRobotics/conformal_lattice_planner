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

#include <chrono>
#include <random>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <boost/pointer_cast.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <node/common/convert_to_visualization_msgs.h>

using namespace std;
using namespace boost;
using namespace router;
using namespace planner;
using namespace planner::idm_lattice_planner;
using namespace planner::spatiotemporal_lattice_planner;

using CarlaVehicle         = carla::client::Vehicle;
using CarlaMap             = carla::client::Map;
using CarlaMapTopologyList = carla::client::Map::TopologyList;
using CarlaWaypoint        = carla::client::Waypoint;
using CarlaTransform       = carla::geom::Transform;
using CarlaLocation        = carla::geom::Location;
using CarlaRoad            = carla::road::Road;
using CarlaSensorDataImage = carla::sensor::data::Image;

namespace node {

sensor_msgs::ImagePtr createImageMsg(
    const boost::shared_ptr<const CarlaSensorDataImage>& img) {

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

visualization_msgs::MarkerPtr createWaypointMsg(
    const vector<boost::shared_ptr<const CarlaWaypoint>>& waypoints) {

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
    CarlaTransform transform = utils::convertTransform(wp->GetTransform());
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
    const CarlaMapTopologyList& waypoint_pairs) {

  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 0.3;
  color.a = 1.0;

  // Collect the unique waypoints in the topology list.
  set<size_t> waypoint_ids;
  std::vector<boost::shared_ptr<CarlaWaypoint>> waypoints;

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
    CarlaTransform transform = utils::convertTransform(wp->GetTransform());
    geometry_msgs::Point pt;
    pt.x = transform.location.x;
    pt.y = transform.location.y;
    pt.z = transform.location.z;

    junctions_msg->points.push_back(pt);
    junctions_msg->colors.push_back(color);
  }

  return junctions_msg;
}

visualization_msgs::MarkerArrayPtr createRoadIdsMsg(
    const std::unordered_map<uint32_t, CarlaRoad>& roads) {

  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 1.0;

  visualization_msgs::MarkerArrayPtr roads_msg(
      new visualization_msgs::MarkerArray);

  for (const auto& item : roads) {

    const CarlaRoad& road = item.second;
    const double length = road.GetLength();
    // Weird enough, the return location is already in right hand coordinate system.
    CarlaLocation location = road.GetDirectedPointIn(length/2.0).location;

    visualization_msgs::MarkerPtr road_msg(new visualization_msgs::Marker);
    road_msg->header.stamp = ros::Time::now();
    road_msg->header.frame_id = "map";
    road_msg->ns = "road_ids";
    road_msg->id = road.GetId();
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
    road_msg->text = std::to_string(road.GetId());

    roads_msg->markers.push_back(*road_msg);
  }

  return roads_msg;
}

visualization_msgs::MarkerArrayPtr createVehiclesMsg(
    const vector<boost::shared_ptr<const CarlaVehicle>>& vehicles) {

  static unordered_map<size_t, std_msgs::ColorRGBA> cached_vehicles;
  visualization_msgs::MarkerArrayPtr vehicles_msg(new visualization_msgs::MarkerArray);

  for (const auto& vehicle : vehicles) {

    std_msgs::ColorRGBA color;
    if (cached_vehicles.count(vehicle->GetId()) != 0) {
      color = cached_vehicles[vehicle->GetId()];
    } else {
      const size_t seed = chrono::system_clock::now().time_since_epoch().count();
      default_random_engine rand_gen(seed);
      uniform_real_distribution<double> uni_dist(0.0, 1.0);
      color.r = uni_dist(rand_gen);
      color.g = uni_dist(rand_gen);
      color.b = uni_dist(rand_gen);
      color.a = 1.0;
      cached_vehicles[vehicle->GetId()] = color;
    }

    visualization_msgs::MarkerPtr vehicle_msg(new visualization_msgs::Marker);
    vehicle_msg->header.stamp = ros::Time::now();
    vehicle_msg->header.frame_id = "map";
    vehicle_msg->ns = "vehicles";
    vehicle_msg->id = vehicle->GetId();
    vehicle_msg->type = visualization_msgs::Marker::CUBE;
    vehicle_msg->action = visualization_msgs::Marker::ADD;
    vehicle_msg->lifetime = ros::Duration(0.0);
    vehicle_msg->frame_locked = false;
    vehicle_msg->scale.x = vehicle->GetBoundingBox().extent.x*2.0;
    vehicle_msg->scale.y = vehicle->GetBoundingBox().extent.y*2.0;
    vehicle_msg->scale.z = vehicle->GetBoundingBox().extent.z*2.0;
    vehicle_msg->color = color;

    CarlaTransform transform = utils::convertTransform(vehicle->GetTransform());
    vehicle_msg->pose.position.x = transform.location.x;
    vehicle_msg->pose.position.y = transform.location.y;
    vehicle_msg->pose.position.z = transform.location.z;
    tf2::Quaternion tf2_quat;
    tf2_quat.setEuler(
        transform.rotation.yaw  /180.0*M_PI,
        transform.rotation.pitch/180.0*M_PI,
        transform.rotation.roll /180.0*M_PI);
    tf2::convert(tf2_quat, vehicle_msg->pose.orientation);

    vehicles_msg->markers.push_back(*vehicle_msg);
  }

  // Collect the tracked vehicles.
  std::unordered_set<size_t> tracked_vehicles;
  for (const auto& vehicle : vehicles)
    tracked_vehicles.insert(vehicle->GetId());

  // Collect the disappear vehicles.
  std::unordered_set<size_t> disappear_vehicles;
  for (const auto& item : cached_vehicles) {
    if (tracked_vehicles.count(item.first) == 0)
      disappear_vehicles.insert(item.first);
  }

  // Tell the RViz to remove the markers of disapperred vehicles.
  for (const size_t id : disappear_vehicles) {
    cached_vehicles.erase(id);

    visualization_msgs::MarkerPtr vehicle_msg(new visualization_msgs::Marker);
    vehicle_msg->header.stamp = ros::Time::now();
    vehicle_msg->header.frame_id = "map";
    vehicle_msg->ns = "vehicles";
    vehicle_msg->id = id;
    vehicle_msg->type = visualization_msgs::Marker::CUBE;
    vehicle_msg->action = visualization_msgs::Marker::DELETE;
    vehicle_msg->lifetime = ros::Duration(0.0);
    vehicle_msg->frame_locked = false;

    vehicles_msg->markers.push_back(*vehicle_msg);
  }

  return vehicles_msg;
}

visualization_msgs::MarkerArrayPtr createVehicleIdsMsg(
    const std::vector<boost::shared_ptr<const CarlaVehicle>>& vehicles) {

  static std::unordered_set<size_t> cached_vehicles;

  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 1.0;

  visualization_msgs::MarkerArrayPtr vehicles_msg(
      new visualization_msgs::MarkerArray);

  for (const auto& vehicle : vehicles) {
    const size_t id = vehicle->GetId();
    cached_vehicles.insert(id);
    CarlaLocation location = vehicle->GetTransform().location;
    utils::convertLocationInPlace(location);

    visualization_msgs::MarkerPtr vehicle_msg(new visualization_msgs::Marker);
    vehicle_msg->header.stamp = ros::Time::now();
    vehicle_msg->header.frame_id = "map";
    vehicle_msg->ns = "vehicle_ids";
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

  // Collect the tracked vehicles.
  std::unordered_set<size_t> tracked_vehicles;
  for (const auto& vehicle : vehicles)
    tracked_vehicles.insert(vehicle->GetId());

  // Collect the disappear vehicles.
  std::unordered_set<size_t> disappear_vehicles;
  for (const auto& id : cached_vehicles) {
    if (tracked_vehicles.count(id) == 0)
      disappear_vehicles.insert(id);
  }

  // Tell the RViz to remove the markers of disapperred vehicles.
  for (const size_t id : disappear_vehicles) {
    cached_vehicles.erase(id);

    visualization_msgs::MarkerPtr vehicle_msg(new visualization_msgs::Marker);
    vehicle_msg->header.stamp = ros::Time::now();
    vehicle_msg->header.frame_id = "map";
    vehicle_msg->ns = "vehicle_ids";
    vehicle_msg->id = id;
    vehicle_msg->type = visualization_msgs::Marker::CUBE;
    vehicle_msg->action = visualization_msgs::Marker::DELETE;
    vehicle_msg->lifetime = ros::Duration(0.0);
    vehicle_msg->frame_locked = false;

    vehicles_msg->markers.push_back(*vehicle_msg);
  }

  return vehicles_msg;
}

geometry_msgs::TransformStampedPtr createVehicleTransformMsg(
    const boost::shared_ptr<const CarlaVehicle>& vehicle,
    const std::string& vehicle_frame_id) {

  geometry_msgs::TransformStampedPtr transform_msg(
      new geometry_msgs::TransformStamped);
  CarlaTransform transform = utils::convertTransform(vehicle->GetTransform());

  transform_msg->header.stamp = ros::Time::now();
  transform_msg->header.frame_id = "map";
  transform_msg->child_frame_id = vehicle_frame_id;

  transform_msg->transform.translation.x = transform.location.x;
  transform_msg->transform.translation.y = transform.location.y;
  transform_msg->transform.translation.z = transform.location.z;

  tf2::Quaternion tf2_quat;
  tf2_quat.setEuler(
      transform.rotation.yaw  /180.0*M_PI,
      transform.rotation.pitch/180.0*M_PI,
      transform.rotation.roll /180.0*M_PI);
  transform_msg->transform.rotation.x = tf2_quat.x();
  transform_msg->transform.rotation.y = tf2_quat.y();
  transform_msg->transform.rotation.z = tf2_quat.z();
  transform_msg->transform.rotation.w = tf2_quat.w();

  return transform_msg;
}

visualization_msgs::MarkerArrayPtr createWaypointLatticeMsg(
    const boost::shared_ptr<const WaypointLattice>& waypoint_lattice) {

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

  const std::unordered_map<size_t, boost::shared_ptr<const WaypointNode>> nodes = waypoint_lattice->nodes();
  const std::vector<std::pair<size_t, size_t>> edges = waypoint_lattice->edges();

  size_t nodes_with_back = 0;

  for (const auto& item : nodes) {
    const boost::shared_ptr<const WaypointNode>& node = item.second;
    CarlaTransform transform = utils::convertTransform(node->waypoint()->GetTransform());
    geometry_msgs::Point pt;
    pt.x = transform.location.x;
    pt.y = transform.location.y;
    pt.z = transform.location.z;

    lattice_node_msg->points.push_back(pt);
    lattice_node_msg->colors.push_back(color);

    if (node->back()) ++nodes_with_back;
  }

  for (const auto& item : edges) {
    const boost::shared_ptr<const WaypointNode>& node0 = nodes.find(item.first)->second;
    const boost::shared_ptr<const WaypointNode>& node1 = nodes.find(item.second)->second;

    CarlaTransform transform0 = utils::convertTransform(node0->waypoint()->GetTransform());
    CarlaTransform transform1 = utils::convertTransform(node1->waypoint()->GetTransform());

    geometry_msgs::Point pt0;
    pt0.x = transform0.location.x;
    pt0.y = transform0.location.y;
    pt0.z = transform0.location.z;

    geometry_msgs::Point pt1;
    pt1.x = transform1.location.x;
    pt1.y = transform1.location.y;
    pt1.z = transform1.location.z;

    lattice_edge_msg->points.push_back(pt0);
    lattice_edge_msg->points.push_back(pt1);
    lattice_edge_msg->colors.push_back(color);
    lattice_edge_msg->colors.push_back(color);
  }

  visualization_msgs::MarkerArrayPtr traffic_lattice_msg(
      new visualization_msgs::MarkerArray);
  traffic_lattice_msg->markers.push_back(*lattice_node_msg);
  traffic_lattice_msg->markers.push_back(*lattice_edge_msg);

  return traffic_lattice_msg;
}

visualization_msgs::MarkerArrayPtr createTrafficLatticeMsg(
    const boost::shared_ptr<const TrafficLattice>& traffic_lattice) {

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
  lattice_node_msg->scale.x = 0.5;
  lattice_node_msg->scale.y = 0.5;
  lattice_node_msg->scale.z = 0.5;
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

  const std::unordered_map<size_t, boost::shared_ptr<const WaypointNodeWithVehicle>> nodes = traffic_lattice->nodes();
  const std::vector<std::pair<size_t, size_t>> edges = traffic_lattice->edges();

  for (const auto& item : nodes) {
    const boost::shared_ptr<const WaypointNodeWithVehicle>& node = item.second;
    CarlaTransform transform = utils::convertTransform(node->waypoint()->GetTransform());
    geometry_msgs::Point pt;
    pt.x = transform.location.x;
    pt.y = transform.location.y;
    pt.z = transform.location.z;

    lattice_node_msg->points.push_back(pt);
    if (node->vehicle()) lattice_node_msg->colors.push_back(special_color);
    else lattice_node_msg->colors.push_back(color);
  }

  for (const auto& item : edges) {
    const boost::shared_ptr<const WaypointNodeWithVehicle>& node0 = nodes.find(item.first)->second;
    const boost::shared_ptr<const WaypointNodeWithVehicle>& node1 = nodes.find(item.second)->second;

    CarlaTransform transform0 = utils::convertTransform(node0->waypoint()->GetTransform());
    CarlaTransform transform1 = utils::convertTransform(node1->waypoint()->GetTransform());

    geometry_msgs::Point pt0;
    pt0.x = transform0.location.x;
    pt0.y = transform0.location.y;
    pt0.z = transform0.location.z;

    geometry_msgs::Point pt1;
    pt1.x = transform1.location.x;
    pt1.y = transform1.location.y;
    pt1.z = transform1.location.z;

    lattice_edge_msg->points.push_back(pt0);
    lattice_edge_msg->points.push_back(pt1);
    lattice_edge_msg->colors.push_back(color);
    lattice_edge_msg->colors.push_back(color);
  }

  visualization_msgs::MarkerArrayPtr traffic_lattice_msg(
      new visualization_msgs::MarkerArray);
  traffic_lattice_msg->markers.push_back(*lattice_node_msg);
  traffic_lattice_msg->markers.push_back(*lattice_edge_msg);

  return traffic_lattice_msg;
}

visualization_msgs::MarkerArrayPtr createTrafficManagerMsg(
    const boost::shared_ptr<const TrafficManager>& traffic_manager) {

  const boost::shared_ptr<const TrafficLattice> traffic_lattice =
    boost::static_pointer_cast<const TrafficLattice>(traffic_manager);
  return createTrafficLatticeMsg(traffic_lattice);
}

visualization_msgs::MarkerArrayPtr createConformalLatticeMsg(
    const std::vector<boost::shared_ptr<const planner::WaypointNode>>& nodes,
    const std::vector<planner::ContinuousPath>& edges) {

  static size_t edge_num = 0;

  std_msgs::ColorRGBA node_color;
  node_color.r = 0.8;
  node_color.g = 0.9;
  node_color.b = 1.0;
  node_color.a = 1.0;

  std_msgs::ColorRGBA edge_color;
  edge_color.r = 0.3;
  edge_color.g = 0.6;
  edge_color.b = 1.0;
  edge_color.a = 1.0;

  visualization_msgs::MarkerPtr nodes_msg(new visualization_msgs::Marker);

  nodes_msg->header.stamp = ros::Time::now();
  nodes_msg->header.frame_id = "map";
  nodes_msg->ns = "conformal_lattice_nodes";
  nodes_msg->id = 0;
  nodes_msg->type = visualization_msgs::Marker::SPHERE_LIST;
  nodes_msg->action = visualization_msgs::Marker::ADD;
  nodes_msg->lifetime = ros::Duration(0.0);
  nodes_msg->frame_locked = false;
  nodes_msg->pose.orientation.w = 1.0;
  nodes_msg->scale.x = 2.0;
  nodes_msg->scale.y = 2.0;
  nodes_msg->scale.z = 2.0;
  nodes_msg->color = node_color;

  for (const auto& node : nodes) {
    //const boost::shared_ptr<const WaypointNode> node = station->node();
    CarlaTransform transform = utils::convertTransform(node->waypoint()->GetTransform());
    geometry_msgs::Point pt;
    pt.x = transform.location.x;
    pt.y = transform.location.y;
    pt.z = transform.location.z;

    nodes_msg->points.push_back(pt);
    nodes_msg->colors.push_back(node_color);
  }

  visualization_msgs::MarkerArrayPtr edges_msg(new visualization_msgs::MarkerArray);

  for (size_t i = 0; i < edges.size(); ++i) {

    const ContinuousPath& edge = edges[i];
    visualization_msgs::MarkerPtr edge_msg(new visualization_msgs::Marker);

    edge_msg->header.stamp = ros::Time::now();
    edge_msg->header.frame_id = "map";
    edge_msg->ns = "conformal_lattice_edges";
    edge_msg->id = i;
    edge_msg->type = visualization_msgs::Marker::LINE_STRIP;
    edge_msg->action = visualization_msgs::Marker::ADD;
    edge_msg->lifetime = ros::Duration(0.0);
    edge_msg->frame_locked = false;
    edge_msg->pose.orientation.w = 1.0;
    edge_msg->scale.x = 0.5;
    edge_msg->scale.y = 0.5;
    edge_msg->scale.z = 0.5;
    edge_msg->color = edge_color;

    populatePathMsg(edge, edge_msg);
    edges_msg->markers.push_back(*edge_msg);
  }

  if (edges.size() < edge_num) {
    for (size_t i = edges.size(); i < edge_num; ++i) {
      visualization_msgs::MarkerPtr edge_msg(new visualization_msgs::Marker);
      edge_msg->header.stamp = ros::Time::now();
      edge_msg->header.frame_id = "map";
      edge_msg->ns = "conformal_lattice_edges";
      edge_msg->id = i;
      edge_msg->type = visualization_msgs::Marker::LINE_STRIP;
      edge_msg->action = visualization_msgs::Marker::DELETE;
      edge_msg->lifetime = ros::Duration(0.0);
      edge_msg->frame_locked = false;
      edges_msg->markers.push_back(*edge_msg);
    }
  }

  edge_num = edges.size();

  visualization_msgs::MarkerArrayPtr conformal_lattice_msg(
      new visualization_msgs::MarkerArray);
  conformal_lattice_msg->markers.push_back(*nodes_msg);
  conformal_lattice_msg->markers.insert(
      conformal_lattice_msg->markers.end(),
      edges_msg->markers.begin(),
      edges_msg->markers.end());

  return conformal_lattice_msg;
}

} // End namespace node.

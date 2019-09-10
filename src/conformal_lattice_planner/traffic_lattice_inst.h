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

#pragma once

#include <cmath>
#include <deque>
#include <unordered_map>
#include <stdexcept>
#include <conformal_lattice_planner/traffic_lattice.h>

namespace planner {

template<typename Router>
TrafficLattice<Router>::TrafficLattice(
    const std::vector<VehicleTuple>& vehicles,
    const boost::shared_ptr<CarlaMap>& map,
    const boost::shared_ptr<Router>& router,
    boost::optional<std::unordered_set<size_t>&> disappear_vehicles) : map_(map) {

  this->router_ = router;

  // Find the start waypoint and range of the lattice based
  // on the given vehicles.
  boost::shared_ptr<CarlaWaypoint> start_waypoint = nullptr;
  double range = 0.0;
  latticeStartAndRange(vehicles, start_waypoint, range);

  // Now we can construct the lattice.
  // FIXME: The following is just a copy of the Lattice custom constructor.
  //        Can we avoid this code duplication?
  this->longitudinal_resolution_ = 1.0;
  baseConstructor(start_waypoint, range, 1.0, router);

  // Register the vehicles onto the lattice nodes.
  std::unordered_set<size_t> remove_vehicles;
  if(!registerVehicles(vehicles, remove_vehicles)) {
    throw std::runtime_error("Collisions detected within the input vehicles.");
  }
  if (disappear_vehicles) disappear_vehicles = remove_vehicles;

  return;
}

template<typename Router>
TrafficLattice<Router>::TrafficLattice(
    const std::vector<boost::shared_ptr<const CarlaVehicle>>& vehicles,
    const boost::shared_ptr<CarlaMap>& map,
    const boost::shared_ptr<Router>& router,
    boost::optional<std::unordered_set<size_t>&> disappear_vehicles) : map_(map) {

  this->router_ = router;

  std::vector<VehicleTuple> vehicle_tuples;
  for (const auto& vehicle : vehicles) {
    vehicle_tuples.push_back(std::make_tuple(
          vehicle->GetId(),
          vehicle->GetTransform(),
          vehicle->GetBoundingBox()));
  }

  // Find the start waypoint and range of the lattice based
  // on the given vehicles.
  boost::shared_ptr<CarlaWaypoint> start_waypoint = nullptr;
  double range = 0.0;
  latticeStartAndRange(vehicle_tuples, start_waypoint, range);

  // Now we can construct the lattice.
  // FIXME: The following is just a copy of the Lattice custom constructor.
  //        Can we avoid this code duplication?
  this->longitudinal_resolution_ = 1.0;
  baseConstructor(start_waypoint, range, 1.0, router);

  // Register the vehicles onto the lattice nodes.
  std::unordered_set<size_t> remove_vehicles;
  if(!registerVehicles(vehicle_tuples, remove_vehicles)) {
    throw std::runtime_error("Collisions detected within the input vehicles.");
  }
  if (disappear_vehicles) *disappear_vehicles = remove_vehicles;

  return;
}

template<typename Router>
TrafficLattice<Router>::TrafficLattice(const TrafficLattice<Router>& other) :
  Base(other) {

  // Make sure the weak pointers point to the stuff within this object.
  vehicle_to_nodes_table_ = other.vehicle_to_nodes_table_;

  for (auto& vehicle : vehicle_to_nodes_table_) {
    for (auto& node : vehicle.second) {
      const size_t id = node.lock()->waypoint()->GetId();
      node = this->waypoint_to_node_table_[id];
    }
  }

  // Carla map won't be copied. \c map_ of different objects point to the
  // same piece of memory.
  map_ = other.map_;

  return;
}

template<typename Router>
void TrafficLattice<Router>::swap(TrafficLattice<Router>& other) {

  Base::swap(other);
  std::swap(vehicle_to_nodes_table_, other.vehicle_to_nodes_table_);
  std::swap(map_, other.map_);

  return;
}

template<typename Router>
boost::optional<std::pair<size_t, double>>
  TrafficLattice<Router>::front(const size_t vehicle) const {

  if (vehicle_to_nodes_table_.count(vehicle) == 0)
    throw std::out_of_range("Given vehicle is not on lattice");

  // Find the node in the lattice which corresponds to the
  // head of the vehicle.
  boost::shared_ptr<const Node> start = vehicleHeadNode(vehicle);
  return frontVehicle(start);
}

template<typename Router>
boost::optional<std::pair<size_t, double>>
  TrafficLattice<Router>::back(const size_t vehicle) const {

  if (vehicle_to_nodes_table_.count(vehicle) == 0)
    throw std::out_of_range("Given vehicle is not on lattice");

  // Find the node in the lattice which corresponds to the
  // back of the vehicle.
  boost::shared_ptr<const Node> start = vehicleRearNode(vehicle);
  return backVehicle(start);
}

template<typename Router>
boost::optional<std::pair<size_t, double>>
  TrafficLattice<Router>::leftFront(const size_t vehicle) const {

  if (vehicle_to_nodes_table_.count(vehicle) == 0)
    throw std::out_of_range("Given vehicle is not on lattice");

  // Find the node in the lattice which corresponds to the
  // head of the vehicle.
  boost::shared_ptr<const Node> start = vehicleHeadNode(vehicle);
  if (!start) throw std::runtime_error("Node no longer exists.");

  // Find the left node of the start.
  // If there is no left node, there is no left front vehicle.
  boost::shared_ptr<const Node> left = start->left();
  if (!left) return boost::none;

  if (!left->vehicle()) {
    // If there is no vehicle at the left node, the case is easy.
    // Just search forward from this left node to find the front vehicle.
    return frontVehicle(left);
  } else {
    // If there is a vehicle at the left node, this is the left front vehicle,
    // since the head of this vehicle must be at least the same distance with
    // the head of the query vehicle.
    const size_t left_vehicle = *(left->vehicle());
    const double distance = vehicleRearNode(left_vehicle)->distance() -
                            start->distance();
    return std::make_pair(left_vehicle, distance);
  }
}

template<typename Router>
boost::optional<std::pair<size_t, double>>
  TrafficLattice<Router>::leftBack(const size_t vehicle) const {

  if (vehicle_to_nodes_table_.count(vehicle) == 0)
    throw std::out_of_range("Given vehicle is not on lattice");

  // Find the node in the lattice which corresponds to the
  // rear of the vehicle.
  boost::shared_ptr<const Node> start = vehicleRearNode(vehicle);
  if (!start) throw std::runtime_error("Node no longer exists.");

  // Find the left node of the start.
  // If there is no left node, there is no left back vehicle.
  boost::shared_ptr<const Node> left = start->left();
  if (!left) return boost::none;

  if (!left->vehicle()) {
    // If there is no vehicle at the left node, the case is easy.
    // Just search backward from this left node to find the back vehicle.
    return backVehicle(left);
  } else {
    // If there is a vehicle at the left node, this is the left back vehicle,
    // since the rear of this vehicle must be at least the same distance with
    // the rear of the query vehicle.
    const size_t left_vehicle = *(left->vehicle());
    const double distance = start->distance() -
                            vehicleHeadNode(left_vehicle)->distance();
    return std::make_pair(left_vehicle, distance);
  }
}

template<typename Router>
boost::optional<std::pair<size_t, double>>
  TrafficLattice<Router>::rightFront(const size_t vehicle) const {

  if (vehicle_to_nodes_table_.count(vehicle) == 0)
    throw std::out_of_range("Given vehicle is not on lattice");

  // Find the node in the lattice which corresponds to the
  // head of the vehicle.
  boost::shared_ptr<const Node> start = vehicleHeadNode(vehicle);
  if (!start) throw std::runtime_error("Node no longer exists.");

  // Find the right node of the start.
  // If there is no right node, there is no right front vehicle.
  boost::shared_ptr<const Node> right = start->right();
  if (!right) return boost::none;

  if (!right->vehicle()) {
    // If there is no vehicle at the right node, the case is easy.
    // Just search forward from this right node to find the front vehicle.
    return frontVehicle(right);
  } else {
    // If there is a vehicle at the right node, this is the right front vehicle,
    // since the head of this vehicle must be at least the same distance with
    // the head of the query vehicle.
    const size_t right_vehicle = *(right->vehicle());
    const double distance = vehicleRearNode(right_vehicle)->distance() -
                            start->distance();
    return std::make_pair(right_vehicle, distance);
  }
}

template<typename Router>
boost::optional<std::pair<size_t, double>>
  TrafficLattice<Router>::rightBack(const size_t vehicle) const {

  if (vehicle_to_nodes_table_.count(vehicle) == 0)
    throw std::out_of_range("Given vehicle is not on lattice");

  // Find the node in the lattice which corresponds to the
  // rear of the vehicle.
  boost::shared_ptr<const Node> start = vehicleRearNode(vehicle);
  if (!start) throw std::runtime_error("Node no longer exists.");

  // Find the right node of the start.
  // If there is no right node, there is no right back vehicle.
  boost::shared_ptr<const Node> right = start->right();
  if (!right) return boost::none;

  if (!right->vehicle()) {
    // If there is no vehicle at the right node, the case is easy.
    // Just search backward from this right node to find the back vehicle.
    return backVehicle(right);
  } else {
    // If there is a vehicle at the right node, this is the right back vehicle,
    // since the rear of this vehicle must be at least the same distance with
    // the rear of the query vehicle.
    const size_t right_vehicle = *(right->vehicle());
    const double distance = start->distance() -
                            vehicleHeadNode(right_vehicle)->distance();
    return std::make_pair(right_vehicle, distance);
  }
}

template<typename Router>
std::unordered_set<size_t> TrafficLattice<Router>::vehicles() const {
  std::unordered_set<size_t> vehicles;
  for (const auto& item : vehicle_to_nodes_table_)
    vehicles.insert(item.first);

  return vehicles;
}

template<typename Router>
int32_t TrafficLattice<Router>::deleteVehicle(const size_t vehicle) {
  // If the vehicle is not being tracked, there is nothing to be deleted.
  if (vehicle_to_nodes_table_.count(vehicle) == 0) return 0;

  // Otherwise, we have to first unregister the vehicle at the
  // corresponding nodes. Then remove the vehicle from the table.
  for (auto& node : vehicle_to_nodes_table_[vehicle])
    if (node.lock()) node.lock()->vehicle() = boost::none;

  vehicle_to_nodes_table_.erase(vehicle);
  return 1;
}

template<typename Router>
int32_t TrafficLattice<Router>::addVehicle(const VehicleTuple& vehicle) {

  // Get the ID, transform, and bounding box of the vehicle to be added.
  size_t id; CarlaTransform transform; CarlaBoundingBox bounding_box;
  std::tie(id, transform, bounding_box) = vehicle;

  // If the vehicle is already on the lattice, the vehicle won't be
  // updated with the new position. The function API is provided to
  // add a new vehicle only.
  if (vehicle_to_nodes_table_.count(id) != 0) {
    //std::printf("Already has this vehicle.\n");
    return 0;
  }

  // Find the waypoints (head and rear) of this vehicle.
  boost::shared_ptr<const CarlaWaypoint> head_waypoint =
    vehicleHeadWaypoint(transform, bounding_box);
  boost::shared_ptr<const CarlaWaypoint> rear_waypoint =
    vehicleRearWaypoint(transform, bounding_box);

  // Find the nodes occupied by this vehicle.
  boost::shared_ptr<Node> head_node = this->closestNode(
      head_waypoint, this->longitudinal_resolution_);
  boost::shared_ptr<Node> rear_node = this->closestNode(
      rear_waypoint, this->longitudinal_resolution_);

  // If we can not add the whole vehicle onto the lattice, we won't add it.
  if (!head_node || !rear_node) {
    //if (!head_node) std::printf("Cannot find vehicle head\n");
    //if (!rear_node) std::printf("Cannot find vehicle rear\n");
    return 0;
  }

  std::vector<boost::weak_ptr<Node>> nodes;
  boost::shared_ptr<Node> next_node = rear_node;
  while (next_node->waypoint()->GetId() != head_node->waypoint()->GetId()) {
    nodes.emplace_back(next_node);
    if (next_node->front().lock()) next_node = next_node->front().lock();
    // We won't add this vehicle onto the lattice if the nodes between
    // head and rear are not connected.
    else {
      //std::printf("Vehicle head and rear are not connected.\n");
      return 0;
    }
  }
  nodes.emplace_back(head_node);

  // If there is already a vehicle on any of the found nodes,
  // it indicates there is a collision.
  for (auto& node : nodes) {
    if (node.lock()->vehicle()) return -1;
    else node.lock()->vehicle() = id;
  }

  // Update the \c vehicle_to_nodes_table_.
  vehicle_to_nodes_table_[id] = nodes;
  return 1;
}

template<typename Router>
bool TrafficLattice<Router>::moveTrafficForward(
    const std::vector<VehicleTuple>& vehicles,
    boost::optional<std::unordered_set<size_t>&> disappear_vehicles) {

  // We require there is an update for every vehicle that is
  // currently being tracked, not more or less.
  std::unordered_set<size_t> existing_vehicles;
  for (const auto& item : vehicle_to_nodes_table_)
    existing_vehicles.insert(item.first);

  std::unordered_set<size_t> update_vehicles;
  for (const auto& item : vehicles)
    update_vehicles.insert(std::get<0>(item));

  if (existing_vehicles != update_vehicles) {
    std::printf("existing vehicles: ");
    for (const auto id : existing_vehicles) std::printf("%lu ", id);
    std::printf("\n update vehicles: ");
    for (const auto id : update_vehicles) std::printf("%lu ", id);
    std::printf("\n");
    throw std::runtime_error("The vehicles to update do not match the exisiting vehicles");
  }

  // Clear all vehicles for the moment, will add them back later.
  for (auto& item : vehicle_to_nodes_table_) {
    for (auto& node : item.second) {
      if (node.lock()) node.lock()->vehicle() = boost::none;
    }
  }
  vehicle_to_nodes_table_.clear();

  // Re-search for the start and range of the lattice.
  boost::shared_ptr<CarlaWaypoint> update_start = nullptr;
  double update_range = 0.0;
  latticeStartAndRange(vehicles, update_start, update_range);

  // Modify the lattice to agree with the new start and range.
  boost::shared_ptr<Node> update_start_node = this->closestNode(
      update_start, this->longitudinal_resolution_);
  this->shorten(this->range()-update_start_node->distance());
  this->extend(update_range);

  // Register the vehicles onto the lattice.
  std::unordered_set<size_t> remove_vehicles;
  registerVehicles(vehicles, remove_vehicles);
  if (disappear_vehicles) *disappear_vehicles = remove_vehicles;

  return true;
}

template<typename Router>
bool TrafficLattice<Router>::moveTrafficForward(
    const std::vector<boost::shared_ptr<const CarlaVehicle>>& vehicles,
    boost::optional<std::unordered_set<size_t>&> disappear_vehicles) {

  // Convert vehicle objects into tuples.
  std::vector<VehicleTuple> vehicle_tuples;
  for (const auto& vehicle : vehicles) {
    vehicle_tuples.push_back(std::make_tuple(
          vehicle->GetId(),
          vehicle->GetTransform(),
          vehicle->GetBoundingBox()));
  }

  std::unordered_set<size_t> remove_vehicles;
  const bool valid = moveTrafficForward(vehicle_tuples, remove_vehicles);
  if (disappear_vehicles) *disappear_vehicles = remove_vehicles;

  return valid;
}

template<typename Router>
void TrafficLattice<Router>::baseConstructor(
    const boost::shared_ptr<CarlaWaypoint>& start,
    const double range,
    const double longitudinal_resolution,
    const boost::shared_ptr<Router>& router) {

  this->longitudinal_resolution_ = longitudinal_resolution;
  this->router_ = router;

  if (range <= this->longitudinal_resolution_) {
    throw std::runtime_error(
        (boost::format("The given range [%1%] is too small."
                       "Range should be at least 1xlongitudinal_resolution.") % range).str());
  }

  // Create the start node.
  boost::shared_ptr<Node> start_node = boost::make_shared<Node>(start);
  start_node->distance() = 0.0;
  this->lattice_entry_ = start_node;
  this->lattice_exit_ = start_node;

  this->augmentWaypointToNodeTable(start->GetId(), start_node);
  this->augmentRoadlaneToWaypointsTable(start);

  // Construct the lattice.
  this->extend(range);

  return;
}

template<typename Router>
void TrafficLattice<Router>::latticeStartAndRange(
    const std::vector<VehicleTuple>& vehicles,
    boost::shared_ptr<CarlaWaypoint>& start,
    double& range) const {

  // Arrange the vehicles by id.
  //std::printf("Arrange the vehicles by IDs.\n");
  std::unordered_map<size_t, CarlaTransform> vehicle_transforms;
  std::unordered_map<size_t, CarlaBoundingBox> vehicle_bounding_boxes;
  for (const auto& vehicle : vehicles) {
    size_t id; CarlaTransform transform; CarlaBoundingBox bounding_box;
    std::tie(id, transform, bounding_box) = vehicle;

    vehicle_transforms[id] = transform;
    vehicle_bounding_boxes[id] = bounding_box;
  }

  // Arrange the vehicles by roads.
  //std::printf("Arrange the vehicles by roads.\n");
  std::unordered_map<size_t, std::vector<size_t>> road_to_vehicles_table;
  for (const auto& vehicle : vehicle_transforms) {
    const size_t road = vehicleWaypoint(vehicle.second)->GetRoadId();
    // Initialize the vector if necessary.
    if (road_to_vehicles_table.count(road) == 0)
      road_to_vehicles_table[road] = std::vector<size_t>();
    road_to_vehicles_table[road].push_back(vehicle.first);
  }

  // Sort the vehicles on each road based on its distance.
  // Vehicles with smaller distance are at the beginning of the vector.
  //std::printf("Sort the vehicles on each road.\n");
  for (auto& road : road_to_vehicles_table) {
    std::sort(road.second.begin(), road.second.end(),
        [this, &vehicle_transforms](const size_t v0, const size_t v1)->bool{
          const double d0 = waypointToRoadStartDistance(
              vehicleWaypoint(vehicle_transforms[v0]));
          const double d1 = waypointToRoadStartDistance(
              vehicleWaypoint(vehicle_transforms[v1]));
          return d0 < d1;
        });
  }

  // Connect the roads into a chain.
  //std::printf("Connect the roads into a chain.\n");
  std::unordered_set<size_t> roads;
  for (const auto& road : road_to_vehicles_table)
    roads.insert(road.first);

  std::deque<size_t> sorted_roads = sortRoads(roads);

  // Find the first (minimum distance) and last (maximum distance)
  // waypoint of the input vehicles.
  //std::printf("Find the first and last vehicle and their waypoints.\n");
  const size_t first_vehicle = road_to_vehicles_table[sorted_roads.front()].front();
  const size_t last_vehicle  = road_to_vehicles_table[sorted_roads.back()].back();

  boost::shared_ptr<CarlaWaypoint> first_waypoint = vehicleRearWaypoint(
      vehicle_transforms[first_vehicle], vehicle_bounding_boxes[first_vehicle]);
  boost::shared_ptr<CarlaWaypoint> last_waypoint = vehicleHeadWaypoint(
      vehicle_transforms[last_vehicle], vehicle_bounding_boxes[last_vehicle]);

  // Set the output start.
  start = first_waypoint;

  // Find the range of the traffic lattice
  // (the distance between the rear of the first vehicle and
  //  the front of the last vehicle).
  //
  // Some special care is required since the first and last waypoints
  // may not be on the existing roads. If not, just extend the range
  // a bit (5m in this case).
  //std::printf("Find the range of the traffic lattice.\n");
  range = 0.0;
  for (const size_t id : sorted_roads) {
    range += map_->GetMap().GetMap().GetRoad(id).GetLength();
  }

  if (first_waypoint->GetRoadId() == sorted_roads.front()) {
    range -= waypointToRoadStartDistance(first_waypoint);
  } else {
    range+= 5.0;
  }

  if (last_waypoint->GetRoadId() == sorted_roads.back()) {
    range -= map_->GetMap().GetMap().GetRoad(sorted_roads.back()).GetLength() -
             waypointToRoadStartDistance(last_waypoint);
  } else {
    range += 5.0;
  }

  return;
}

template<typename Router>
bool TrafficLattice<Router>::registerVehicles(
    const std::vector<VehicleTuple>& vehicles,
    boost::optional<std::unordered_set<size_t>&> disappear_vehicles) {

  // Clear the \c vehicle_to_node_table_.
  vehicle_to_nodes_table_.clear();

  // Add vehicles onto the lattice, keep track of the disappearred/removed vehicles as well.
  std::unordered_set<size_t> removed_vehicles;
  for (const auto& vehicle : vehicles) {
    const int32_t valid = addVehicle(vehicle);
    if (valid == 0) removed_vehicles.insert(std::get<0>(vehicle));
    else if (valid == -1) return false;
  }

  if (disappear_vehicles) *disappear_vehicles = removed_vehicles;
  return true;
}

template<typename Router>
std::deque<size_t> TrafficLattice<Router>::sortRoads(
    const std::unordered_set<size_t>& roads) const {

  // Keep track of the road IDs we have not dealt with.
  std::unordered_set<size_t> remaining_roads(roads);

  // Keep track of the sorted roads.
  std::deque<size_t> sorted_roads;

  // Start from a random road in the given set.
  sorted_roads.push_back(*(remaining_roads.begin()));
  remaining_roads.erase(remaining_roads.begin());

  // We will only expand 5 times.
  for (size_t i = 0; i < 5; ++i) {
    // Current first and last road in the chain.
    const size_t first_road = sorted_roads.front();
    const size_t last_road = sorted_roads.back();

    // New first and last road in the chain.
    boost::optional<size_t> new_first_road = this->router_->prevRoad(first_road);
    boost::optional<size_t> new_last_road = this->router_->nextRoad(last_road);

    if (new_first_road) {
      sorted_roads.push_front(*new_first_road);
      if (!remaining_roads.empty()) remaining_roads.erase(*new_first_road);
    }
    if (new_last_road) {
      sorted_roads.push_back(*new_last_road);
      if (!remaining_roads.empty()) remaining_roads.erase(*new_last_road);
    }
    if (remaining_roads.empty()) break;
  }

  // If for some weired reason, there is still some road remaining
  // which cannot be sorted, throw a runtime error.
  if (!remaining_roads.empty()) {
    throw std::runtime_error(
        "The given roads cannot be sorted."
        "This is probably because the given vehicles does not construct a local traffic.");
  }

  // Trim the sorted road vector so that both the first and last road
  // in the vector are within the given roads.
  while (roads.count(sorted_roads.front()) == 0)
    sorted_roads.pop_front();
  while (roads.count(sorted_roads.back()) == 0)
    sorted_roads.pop_back();

  return sorted_roads;
}

template<typename Router>
boost::shared_ptr<typename TrafficLattice<Router>::CarlaWaypoint>
  TrafficLattice<Router>::vehicleHeadWaypoint(
    const CarlaTransform& transform,
    const CarlaBoundingBox& bounding_box) const {

  const double sin = std::sin(transform.rotation.yaw/180.0*M_PI);
  const double cos = std::cos(transform.rotation.yaw/180.0*M_PI);

  // Be careful here! We are dealing with the left hand coordinates.
  // Do not really care about the z-axis.
  carla::geom::Location waypoint_location;
  waypoint_location.x = cos*bounding_box.extent.x + transform.location.x;
  waypoint_location.y = sin*bounding_box.extent.x + transform.location.y;
  waypoint_location.z = transform.location.z;

  //std::printf("vehicle waypoint location: x:%f y:%f z:%f\n",
  //    transform.location.x, transform.location.y, transform.location.z);
  //std::printf("head waypoint location: x:%f y:%f z:%f\n",
  //    waypoint_location.x, waypoint_location.y, waypoint_location.z);

  return map_->GetWaypoint(waypoint_location);
}

template<typename Router>
boost::shared_ptr<typename TrafficLattice<Router>::CarlaWaypoint>
  TrafficLattice<Router>::vehicleRearWaypoint(
    const CarlaTransform& transform,
    const CarlaBoundingBox& bounding_box) const {

  const double sin = std::sin(transform.rotation.yaw/180.0*M_PI);
  const double cos = std::cos(transform.rotation.yaw/180.0*M_PI);

  // Be careful here! We are dealing with the left hand coordinates.
  // Do not really care about the z-axis.
  carla::geom::Location waypoint_location;
  waypoint_location.x = -cos*bounding_box.extent.x + transform.location.x;
  waypoint_location.y = -sin*bounding_box.extent.x + transform.location.y;
  waypoint_location.z = transform.location.z;

  //std::printf("vehicle waypoint location: x:%f y:%f z:%f\n",
  //    transform.location.x, transform.location.y, transform.location.z);
  //std::printf("rear waypoint location: x:%f y:%f z:%f\n",
  //    waypoint_location.x, waypoint_location.y, waypoint_location.z);
  return map_->GetWaypoint(waypoint_location);
}

template<typename Router>
boost::optional<std::pair<size_t, double>>
  TrafficLattice<Router>::frontVehicle(
      const boost::shared_ptr<const Node>& start) const {

  if (!start) throw std::runtime_error("Node no longer exists.");

  boost::shared_ptr<const Node> front = start->front();
  while (front) {
    // If we found a vehicle at the front node, this is it.
    if (front->vehicle())
      return std::make_pair(*(front->vehicle()), front->distance()-start->distance());
    // Otherwise, keep moving forward.
    front = front->front();
  }

  // There is no front vehicle from the given node.
  return boost::none;
}

template<typename Router>
boost::optional<std::pair<size_t, double>>
  TrafficLattice<Router>::backVehicle(
      const boost::shared_ptr<const Node>& start) const {

  if (!start) throw std::runtime_error("Node no longer exists.");

  boost::shared_ptr<const Node> back = start->back();
  while (back) {
    // If we found a vehicle at the front node, this is it.
    if (back->vehicle())
      return std::make_pair(*(back->vehicle()), start->distance()-back->distance());
    // Otherwise, keep moving backward.
    back = back->back();
  }

  // There is no back vehicle from the given node.
  return boost::none;
}

} // End namespace planner.

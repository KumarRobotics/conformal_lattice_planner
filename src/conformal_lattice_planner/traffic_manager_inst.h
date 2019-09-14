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

#include <conformal_lattice_planner/traffic_manager.h>

namespace planner {

template<typename Router>
TrafficManager<Router>::TrafficManager(
    const boost::shared_ptr<CarlaWaypoint>& start,
    const double range,
    const boost::shared_ptr<Router>& router,
    const boost::shared_ptr<CarlaMap>& map) {
  // The \c longitudinal_resolution_ is fixed to 1.0m.
  this->map_ = map;
  this->baseConstructor(start, range, 1.0, router);
  return;
}

template<typename Router>
bool TrafficManager<Router>::moveTrafficForward(
    const std::vector<VehicleTuple>& vehicles,
    const double shift_distance,
    boost::optional<std::unordered_set<size_t>&> disappear_vehicles) {

  // We require there is an update for every vehicle that is
  // currently being tracked, not more or less.
  std::unordered_set<size_t> existing_vehicles;
  for (const auto& item : this->vehicle_to_nodes_table_)
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
  for (auto& item : this->vehicle_to_nodes_table_) {
    for (auto& node : item.second) {
      if (node.lock()) node.lock()->vehicle() = boost::none;
    }
  }
  this->vehicle_to_nodes_table_.clear();

  // Shift the whole lattice forward by the given distance.
  this->shift(shift_distance);

  // Register the vehicles onto the lattice.
  std::unordered_set<size_t> remove_vehicles;
  const bool valid = this->registerVehicles(vehicles, remove_vehicles);
  if (disappear_vehicles) *disappear_vehicles = remove_vehicles;

  return valid;
}

template<typename Router>
bool TrafficManager<Router>::moveTrafficForward(
    const std::vector<boost::shared_ptr<const CarlaVehicle>>& vehicles,
    const double shift_distance,
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
  const bool valid = moveTrafficForward(vehicle_tuples, shift_distance, remove_vehicles);
  if (disappear_vehicles) *disappear_vehicles = remove_vehicles;

  return valid;
}

template<typename Router>
boost::optional<std::pair<
  double,
  boost::shared_ptr<const typename TrafficManager<Router>::CarlaWaypoint>
  >>
  TrafficManager<Router>::frontSpawnWaypoint(const double min_range) const {

  // All lattice exits are candidates where we can spawn new vehicles.
  std::vector<boost::shared_ptr<const Node>> candidates;
  for (auto& exit : this->lattice_exits_)
    candidates.push_back(exit.lock());

  // Find the best candidate based on the distance to the vehicle on its back.
  boost::shared_ptr<const Node> best_candidate = nullptr;
  double best_distance = 0.0;

  for (const auto& candidate : candidates) {
    boost::optional<std::pair<size_t, double>> back = this->backVehicle(candidate);
    if (back) {
      if (back->second > best_distance) {
        best_distance = back->second;
        best_candidate = candidate;
      }
    } else {
      // FIXME : lattice range may be longer than the range of this lane.
      if (this->range() > best_distance) {
        best_distance = this->range();
        best_candidate = candidate;
      }
    }
  }

  // Check if the \c best_distance meets the \c min_range requirement.
  if (best_distance >= min_range) {
    return std::make_pair(best_distance, best_candidate->waypoint());
  } else return boost::none;
}

template<typename Router>
boost::optional<std::pair<
  double,
  boost::shared_ptr<const typename TrafficManager<Router>::CarlaWaypoint>
  >>
  TrafficManager<Router>::backSpawnWaypoint(const double min_range) const {

  // All lattice entries are candidates where we can spawn new vehicles.
  std::vector<boost::shared_ptr<const Node>> candidates;
  for (auto& entry : this->lattice_entries_)
    candidates.push_back(entry.lock());

  // Find the best candidate based on the distance to the vehicle on its front.
  boost::shared_ptr<const Node> best_candidate = nullptr;
  double best_distance = 0.0;

  for (const auto& candidate : candidates) {
    boost::optional<std::pair<size_t, double>> front = this->frontVehicle(candidate);
    if (front) {
      if (front->second > best_distance) {
        best_distance = front->second;
        best_candidate = candidate;
      }
    } else {
      // FIXME : lattice range may be longer than the range of this lane.
      if (this->range() > best_distance) {
        best_distance = this->range();
        best_candidate = candidate;
      }
    }
  }

  // Check if the \c best_distance meets the \c min_range requirement.
  if (best_distance >= min_range) {
    return std::make_pair(best_distance, best_candidate->waypoint());
  } else return boost::none;
}

} // End namespace planner.


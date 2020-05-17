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

#include <algorithm>
#include <random>
#include <chrono>

#include <planner/common/traffic_manager.h>

namespace planner {

TrafficManager::TrafficManager(
    const boost::shared_ptr<CarlaWaypoint>& start,
    const double range,
    const boost::shared_ptr<router::Router>& router,
    const boost::shared_ptr<CarlaMap>& map,
    const boost::shared_ptr<utils::FastWaypointMap>& fast_map) {
  // The \c longitudinal_resolution_ is fixed to 1.0m.
  this->map_ = map;
  this->fast_map_ = fast_map;
  this->baseConstructor(start, range, 1.0, router);
  return;
}

bool TrafficManager::moveTrafficForward(
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
    std::string error_msg(
        "TrafficManager::moveTrafficForward(): "
        "update vehicles does not match existing vehicles.\n");

    std::string existing_vehicles_msg("Existing vehicles: ");
    for (const auto id : existing_vehicles)
      existing_vehicles_msg += std::to_string(id) + " ";
    existing_vehicles_msg += "\n";

    std::string update_vehicles_msg("Update vehicles: ");
    for (const auto id : update_vehicles)
      update_vehicles_msg += std::to_string(id) + " ";
    update_vehicles_msg += "\n";

    throw std::runtime_error(error_msg + existing_vehicles_msg + update_vehicles_msg);
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

  // Find waypoints for each of the input vehicle.
  std::unordered_map<size_t, VehicleWaypoints>
    vehicle_waypoints = this->vehicleWaypoints(vehicles);

  // Register the vehicles onto the lattice.
  std::unordered_set<size_t> remove_vehicles;
  const bool valid = this->registerVehicles(vehicles, vehicle_waypoints, remove_vehicles);
  if (disappear_vehicles) *disappear_vehicles = remove_vehicles;

  return valid;
}

bool TrafficManager::moveTrafficForward(
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

boost::optional<std::pair<
  double,
  boost::shared_ptr<const typename TrafficManager::CarlaWaypoint>
  >>
  TrafficManager::frontSpawnWaypoint(const double min_range) const {

  // All lattice exits are candidates where we can spawn new vehicles.
  std::vector<boost::shared_ptr<const Node>> candidates;
  for (auto& exit : this->lattice_exits_)
    candidates.push_back(exit.lock());

  // Collect candidates that meet the requirement.
  std::vector<std::pair<double, boost::shared_ptr<const CarlaWaypoint>>> valid_candidates;
  for (const auto& candidate : candidates) {
    boost::optional<std::pair<size_t, double>> back = this->backVehicle(candidate);
    if (back && back->second < min_range) continue;

    if (!back) valid_candidates.push_back(
        std::make_pair(this->range(), candidate->waypoint()));
    else       valid_candidates.push_back(
        std::make_pair(back->second, candidate->waypoint()));
  }

  // Return \c boost::none if there is no valid candidate.
  if (valid_candidates.size() == 0) return boost::none;

  // Otherwise, return a random candidate.
  size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::shuffle(valid_candidates.begin(),
               valid_candidates.end(),
               std::default_random_engine(seed));
  return valid_candidates.front();
}

boost::optional<std::pair<
  double,
  boost::shared_ptr<const typename TrafficManager::CarlaWaypoint>
  >>
  TrafficManager::backSpawnWaypoint(const double min_range) const {

  // All lattice entries are candidates where we can spawn new vehicles.
  std::vector<boost::shared_ptr<const Node>> candidates;
  for (auto& entry : this->lattice_entries_)
    candidates.push_back(entry.lock());

  // Collect candidates that meet the requirement.
  std::vector<std::pair<double, boost::shared_ptr<const CarlaWaypoint>>> valid_candidates;
  for (const auto& candidate : candidates) {
    boost::optional<std::pair<size_t, double>> front = this->frontVehicle(candidate);
    if (front && front->second < min_range) continue;

    if (!front) valid_candidates.push_back(
        std::make_pair(this->range(), candidate->waypoint()));
    else       valid_candidates.push_back(
        std::make_pair(front->second, candidate->waypoint()));
  }

  // Return \c boost::none if there is no valid candidate.
  if (valid_candidates.size() == 0) return boost::none;

  // Otherwise, return a random candidate.
  size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::shuffle(valid_candidates.begin(),
               valid_candidates.end(),
               std::default_random_engine(seed));
  return valid_candidates.front();
}

} // End namespace planner.


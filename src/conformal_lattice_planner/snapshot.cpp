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

#include <stdexcept>
#include <unordered_set>
#include <conformal_lattice_planner/snapshot.h>

namespace planner {

Snapshot::Snapshot(
    const Vehicle& ego,
    const std::unordered_map<size_t, Vehicle>& agents,
    const boost::shared_ptr<router::LoopRouter>& router,
    const boost::shared_ptr<CarlaMap>& map) :
  ego_(ego),
  agents_(agents) {

  // Collect all vehicles into an array of tuples.
  std::vector<std::tuple<size_t, CarlaTransform, CarlaBoundingBox>> vehicles;
  vehicles.push_back(ego_.tuple());
  for (const auto& agent : agents_) vehicles.push_back(agent.second.tuple());

  // Generate the waypoint lattice.
  std::unordered_set<size_t> disappear_vehicles;
  traffic_lattice_ = boost::make_shared<TrafficLattice<router::LoopRouter>>(
      vehicles, map, router, disappear_vehicles);

  // Remove the disappeared vehicles.
  if (disappear_vehicles.count(ego_.id()) != 0)
    throw std::runtime_error("The ego vehicle is removed from the snapshot");

  for (const size_t agent : disappear_vehicles)
    agents_.erase(agent);

  return;
}

Snapshot::Snapshot(const Snapshot& other) :
  ego_(other.ego_),
  agents_(other.agents_),
  traffic_lattice_(boost::make_shared<TrafficLattice<router::LoopRouter>>(*(other.traffic_lattice_))) {}

Snapshot& Snapshot::operator=(const Snapshot& other) {
  ego_ = other.ego_;
  agents_ = other.agents_;
  traffic_lattice_ = boost::make_shared<TrafficLattice<router::LoopRouter>>(*(other.traffic_lattice_));
  return *this;
}

const Vehicle& Snapshot::agent(const size_t id) const {
  std::unordered_map<size_t, Vehicle>::const_iterator iter = agents_.find(id);
  if (iter == agents_.end())
    throw std::out_of_range("The required agent vehicle does not exist in the snapshot");
  return iter->second;
}

Vehicle& Snapshot::agent(const size_t id) {
  std::unordered_map<size_t, Vehicle>::iterator iter = agents_.find(id);
  if (iter == agents_.end())
    throw std::out_of_range("The required agent vehicle does not exist in the snapshot");

  return iter->second;
}


bool Snapshot::updateTraffic(
    const std::vector<std::tuple<size_t, CarlaTransform, double, double>>& updates) {

  // Check the input vector has the updates for every vehicle in the snapshot.
  std::unordered_set<size_t> updated_vehicles;
  for (const auto& update : updates)
    updated_vehicles.insert(std::get<0>(update));

  if (updated_vehicles.count(ego_.id()) == 0)
    throw std::runtime_error("Update for the ego vehicle is missing.");

  for (const auto& agent : agents_) {
    if (updated_vehicles.count(agent.first) == 0)
      throw std::runtime_error("Update for an agent vehicle is missing.");
  }

  // Update the transform, speed, and acceleration for all vehicles.
  for (const auto& update : updates) {
    size_t id; CarlaTransform update_transform;
    double update_speed; double update_acceleration;
    std::tie(id, update_transform, update_speed, update_acceleration) = update;

    // Update the status for the ego vehicle.
    if (id == ego_.id()) {
      ego_.transform() = update_transform;
      ego_.speed() = update_speed;
      ego_.acceleration() = update_acceleration;
      continue;
    }

    // Update the status for the agent vehicles.
    std::unordered_map<size_t, Vehicle>::iterator agent_iter = agents_.find(id);
    // This vehicle is not in the snapshot.
    if (agent_iter == agents_.end()) continue;

    agent_iter->second.transform() = update_transform;
    agent_iter->second.speed() = update_speed;
    agent_iter->second.acceleration() = update_acceleration;
  }

  // Update the traffic lattice.
  std::vector<std::tuple<size_t, CarlaTransform, CarlaBoundingBox>> vehicles;
  vehicles.push_back(ego_.tuple());
  for (const auto& agent : agents_)
    vehicles.push_back(agent.second.tuple());

  std::unordered_set<size_t> disappear_vehicles;
  const bool no_collision = traffic_lattice_->moveTrafficForward(vehicles, disappear_vehicles);

  // Remove the \c disappear_vehicles from the snapshot.
  if (disappear_vehicles.count(ego_.id()) != 0)
    throw std::runtime_error("The ego vehicle is removed from the lattice.");

  for (const size_t disappear_vehicle : disappear_vehicles)
    agents_.erase(disappear_vehicle);

  return no_collision;
}
} // End namespace planner.

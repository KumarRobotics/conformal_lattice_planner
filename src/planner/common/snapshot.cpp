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

#include <string>
#include <stdexcept>
#include <unordered_set>
#include <boost/format.hpp>

#include <planner/common/snapshot.h>

namespace planner {

Snapshot::Snapshot(
    const Vehicle& ego,
    const std::unordered_map<size_t, Vehicle>& agents,
    const boost::shared_ptr<router::Router>& router,
    const boost::shared_ptr<CarlaMap>& map,
    const boost::shared_ptr<utils::FastWaypointMap>& fast_map) :
  ego_(ego),
  agents_(agents) {

  // Collect all vehicles into an array of tuples.
  std::vector<std::tuple<size_t, CarlaTransform, CarlaBoundingBox>> vehicles;
  vehicles.push_back(ego_.tuple());
  for (const auto& agent : agents_) vehicles.push_back(agent.second.tuple());

  // Generate the waypoint lattice.
  std::unordered_set<size_t> disappear_vehicles;
  traffic_lattice_ = boost::make_shared<TrafficLattice>(
      vehicles, map, fast_map, router, disappear_vehicles);

  // Remove the disappeared vehicles.
  if (disappear_vehicles.count(ego_.id()) != 0) {
    std::string error_msg(
        "Snapshot::Snapshot(): "
        "the ego vehicle is removed from the snapshot.\n");
    std::string snapshot_msg = this->string();
    throw std::runtime_error(error_msg + snapshot_msg);
  }

  for (const size_t agent : disappear_vehicles)
    agents_.erase(agent);

  return;
}

Snapshot::Snapshot(const Snapshot& other) :
  ego_(other.ego_),
  agents_(other.agents_),
  traffic_lattice_(boost::make_shared<TrafficLattice>(*(other.traffic_lattice_))) {}

Snapshot& Snapshot::operator=(const Snapshot& other) {
  ego_ = other.ego_;
  agents_ = other.agents_;
  traffic_lattice_ = boost::make_shared<TrafficLattice>(*(other.traffic_lattice_));
  return *this;
}

const Vehicle& Snapshot::agent(const size_t id) const {
  std::unordered_map<size_t, Vehicle>::const_iterator iter = agents_.find(id);
  if (iter == agents_.end()) {
    std::string error_msg = (boost::format(
          "Snapshot::agent(): "
          "the required agent %1% does not exist in the snapshot.\n") % id).str();
    std::string snapshot_msg = this->string();
    throw std::runtime_error(error_msg + snapshot_msg);
  }
  return iter->second;
}

Vehicle& Snapshot::agent(const size_t id) {
  std::unordered_map<size_t, Vehicle>::iterator iter = agents_.find(id);
  if (iter == agents_.end()) {
    std::string error_msg = (boost::format(
          "Snapshot::agent(): "
          "the required agent %1% does not exist in the snapshot.\n") % id).str();
    std::string snapshot_msg = this->string();
    throw std::runtime_error(error_msg + snapshot_msg);
  }
  return iter->second;
}

const Vehicle& Snapshot::vehicle(const size_t id) const {
  if (id == ego_.id()) return ego_;
  else return agent(id);
}

Vehicle& Snapshot::vehicle(const size_t id) {
  if (id == ego_.id()) return ego_;
  else return agent(id);
}

bool Snapshot::updateTraffic(
    const std::vector<std::tuple<size_t, CarlaTransform, double, double, double>>& updates) {

  // The tuple consists of the vehicle ID, transform, speed, acceleration, curvature.

  // Check the input vector has the updates for every vehicle in the snapshot.
  std::unordered_set<size_t> updated_vehicles;
  for (const auto& update : updates)
    updated_vehicles.insert(std::get<0>(update));

  // Update the transform, speed, and acceleration for all vehicles.
  for (const auto& update : updates) {
    size_t id; CarlaTransform update_transform;
    double update_speed; double update_acceleration; double update_curvature;
    std::tie(id, update_transform, update_speed, update_acceleration, update_curvature) = update;

    // Update the status for the ego vehicle.
    if (id == ego_.id()) {
      ego_.transform() = update_transform;
      ego_.speed() = update_speed;
      ego_.acceleration() = update_acceleration;
      ego_.curvature() = update_curvature;
      continue;
    }

    // Update the status for the agent vehicles.
    std::unordered_map<size_t, Vehicle>::iterator agent_iter = agents_.find(id);
    // This vehicle is not in the snapshot.
    if (agent_iter == agents_.end()) continue;

    agent_iter->second.transform() = update_transform;
    agent_iter->second.speed() = update_speed;
    agent_iter->second.acceleration() = update_acceleration;
    agent_iter->second.curvature() = update_curvature;
  }

  // Update the traffic lattice.
  std::vector<std::tuple<size_t, CarlaTransform, CarlaBoundingBox>> vehicles;
  vehicles.push_back(ego_.tuple());
  for (const auto& agent : agents_)
    vehicles.push_back(agent.second.tuple());

  std::unordered_set<size_t> disappear_vehicles;
  const bool no_collision = traffic_lattice_->moveTrafficForward(vehicles, disappear_vehicles);

  // Remove the \c disappear_vehicles from the snapshot.
  if (disappear_vehicles.count(ego_.id()) != 0) {
    std::string error_msg(
        "Snapshot::UpdateTraffic(): "
        "the ego vehicle is removed from the snapshot.\n");
    std::string snapshot_msg = this->string();
    throw std::runtime_error(error_msg + snapshot_msg);
  }

  for (const size_t disappear_vehicle : disappear_vehicles)
    agents_.erase(disappear_vehicle);

  return no_collision;
}
} // End namespace planner.

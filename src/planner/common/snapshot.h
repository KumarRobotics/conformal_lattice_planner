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

#include <unordered_map>
#include <carla/client/Map.h>
#include <carla/client/Vehicle.h>
#include <carla/geom/Transform.h>

#include <router/common/router.h>
#include <planner/common/vehicle.h>
#include <planner/common/traffic_lattice.h>

namespace planner {

/**
 * \brief Snapshot records the status of all vehicles within a micro traffic at
 *        one time instance.
 *
 * The snapshot objects uses TrafficLattice to bookkeep the relative locations
 * of the vehicles.
 */
class Snapshot {

protected:

  using CarlaMap         = carla::client::Map;
  using CarlaVehicle     = carla::client::Vehicle;
  using CarlaTransform   = carla::geom::Transform;
  using CarlaBoundingBox = carla::geom::BoundingBox;

protected:

  /// The ego vehicle.
  Vehicle ego_;

  /// Agents in the micro traffic, i.e. all vechiles other than the ego.
  //std::vector<Vehicle> agents_;
  std::unordered_map<size_t, Vehicle> agents_;

  /// Traffic lattice which is used to keep track of the relative
  /// location among the vehicles.
  boost::shared_ptr<TrafficLattice> traffic_lattice_;

public:

  Snapshot(const Vehicle& ego,
           const std::unordered_map<size_t, Vehicle>& agents,
           const boost::shared_ptr<router::Router>& router,
           const boost::shared_ptr<CarlaMap>& map,
           const boost::shared_ptr<utils::FastWaypointMap>& fast_map);

  Snapshot(const Snapshot& other);

  Snapshot& operator=(const Snapshot& other);

  const Vehicle& ego() const { return ego_; }
  Vehicle& ego() { return ego_; }

  const std::unordered_map<size_t, Vehicle>& agents() const { return agents_; }
  std::unordered_map<size_t, Vehicle>& agents() { return agents_; }

  const Vehicle& agent(const size_t id) const;
  Vehicle& agent(const size_t id);

  const Vehicle& vehicle(const size_t id) const;
  Vehicle& vehicle(const size_t id);

  const boost::shared_ptr<const TrafficLattice>
    trafficLattice() const { return traffic_lattice_; }
  const boost::shared_ptr<TrafficLattice>
    trafficLattice() { return traffic_lattice_; }

  // The input \c new_transforms should cover every vehicle in the snapshot.
  // The tuple consists of the vehicle ID, transform, speed, acceleration, curvature.
  bool updateTraffic(
      const std::vector<std::tuple<size_t, CarlaTransform, double, double, double>>& transforms);

  std::string string(const std::string& prefix = "") const {
    std::string output = prefix;
    output += ego_.string("ego ");
    for (const auto& agent : agents_)
      output += agent.second.string("agent ");
    output += "waypoint lattice range: " + std::to_string(traffic_lattice_->range()) + "\n";
    return output;
  }

};
} // End namespace planner.


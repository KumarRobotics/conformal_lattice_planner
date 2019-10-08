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

#include <unordered_map>
#include <carla/client/Map.h>
#include <carla/client/Vehicle.h>
#include <carla/geom/Transform.h>
#include <conformal_lattice_planner/vehicle.h>
#include <conformal_lattice_planner/traffic_lattice.h>
#include <conformal_lattice_planner/loop_router.h>

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
  boost::shared_ptr<TrafficLattice<router::LoopRouter>> traffic_lattice_;

public:

  Snapshot(const Vehicle& ego,
           const std::unordered_map<size_t, Vehicle>& agents,
           const boost::shared_ptr<router::LoopRouter>& router,
           const boost::shared_ptr<CarlaMap>& map);

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

  const boost::shared_ptr<const TrafficLattice<router::LoopRouter>>
    trafficLattice() const { return traffic_lattice_; }
  const boost::shared_ptr<TrafficLattice<router::LoopRouter>>
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


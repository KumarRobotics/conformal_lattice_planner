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

#include <conformal_lattice_planner/traffic_lattice.h>

namespace planner {

template<typename Router>
class TrafficManager : public TrafficLattice<Router> {

protected:

  using CarlaVehicle     = carla::client::Vehicle;
  using CarlaMap         = carla::client::Map;
  using CarlaWaypoint    = carla::client::Waypoint;
  using CarlaBoundingBox = carla::geom::BoundingBox;
  using CarlaTransform   = carla::geom::Transform;
  using CarlaRoad        = carla::road::Road;
  using CarlaLane        = carla::road::Lane;
  using CarlaRoadMap     = carla::road::Map;
  using CarlaMapData     = carla::road::MapData;

  using Node = WaypointNodeWithVehicle;
  /// FIXME: Don't really want to define a new struct for this.
  ///        Is there a better solution than \c tuple?
  using VehicleTuple = std::tuple<size_t, CarlaTransform, CarlaBoundingBox>;

private:

  using Base = TrafficLattice<Router>;
  //using GrandBase = Lattice<WaypointNodeWithVehicle, Router>;

public:

  using Base::front;
  using Base::back;
  using Base::leftFront;
  using Base::frontLeft;
  using Base::leftBack;
  using Base::backLeft;
  using Base::rightFront;
  using Base::frontRight;
  using Base::rightBack;
  using Base::backRight;

  TrafficManager(const boost::shared_ptr<CarlaWaypoint>& start,
                 const double range,
                 const boost::shared_ptr<Router>& router);

  /// Move traffic forward.
  /// The function does not adapt the lattice range to the input vehicles,
  /// but shift the lattice forward according to the given distance. All vehicles
  /// that are outside the lattice are returned in the \c disappear_vehicles.
  bool moveTrafficForward(
      const std::vector<VehicleTuple>& vehicles,
      const double shift_distance,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles = boost::none);

  bool moveTrafficForward(
      const std::vector<boost::shared_ptr<const CarlaVehicle>>& vehicles,
      const double shift_distance,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles = boost::none);

  /// Suggest location to spawn a new vehicle at the front of the lattice.
  boost::optional<std::pair<double, boost::shared_ptr<const CarlaWaypoint>>>
    frontSpawnWaypoint(const double min_range) const;

  /// Suggest location to spawn a new vehicle at the back of the lattice.
  boost::optional<std::pair<double, boost::shared_ptr<const CarlaWaypoint>>>
    backSpawnWaypoint(const double min_range) const;

}; // End class TrafficManager.

} // End namespace planner.

#include <conformal_lattice_planner/traffic_manager_inst.h>

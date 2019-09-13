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

#include <cstdint>
#include <tuple>
#include <boost/optional.hpp>
#include <carla/client/Vehicle.h>
#include <carla/client/Map.h>
#include <conformal_lattice_planner/waypoint_lattice.h>

namespace planner {

class WaypointNodeWithVehicle : public LatticeNode<WaypointNodeWithVehicle> {

protected:

  using CarlaWaypoint = carla::client::Waypoint;
  using CarlaVehicle = carla::client::Vehicle;

protected:

  /// Carla waypoint of this node.
  boost::shared_ptr<CarlaWaypoint> waypoint_ = nullptr;

  /**
   * The distance of this waypoint in the lattice.
   *
   * Note this is different than the \c s attribute of a carla waypoint,
   * which is the distance of the waypoint on the road it belongs to.
   */
  double distance_ = 0.0;

  /// Id of the vehicle that occupies this node.
  boost::optional<size_t> vehicle_ = boost::none;

public:

  WaypointNodeWithVehicle() = default;

  WaypointNodeWithVehicle(const boost::shared_ptr<CarlaWaypoint>& waypoint) :
    waypoint_(waypoint) {}

  boost::shared_ptr<CarlaWaypoint>& waypoint() {
    return waypoint_;
  }

  boost::shared_ptr<const CarlaWaypoint> waypoint() const {
    return boost::const_pointer_cast<const CarlaWaypoint>(waypoint_);
  }

  double& distance() { return distance_; }

  const double distance() const { return distance_; }

  /// Get the vehicle Id registered at this node.
  boost::optional<size_t> vehicle() const { return vehicle_; }

  /// Get or set the vehicle at this node.
  boost::optional<size_t>& vehicle() { return vehicle_; }

}; // End class \c WaypointNodeWithVehicle.

/**
 * \brief TrafficLattice is a helper class used to track local traffic,
 *        i.e. the vehicles within a finite range neighborhood.
 *
 * \note Have to change carla/road/Map.h to compile this class.
 *       Remove the guard of LIBCARLA_WITH_GETEST, and set the
 *       function prototype from
 *       MapData& carla::road::Map::GetMap() to
 *       const MapData& carla::road::Map::GetMap() const.
 */
template<typename Router>
class TrafficLattice : public Lattice<WaypointNodeWithVehicle, Router> {

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

  using Base = Lattice<WaypointNodeWithVehicle, Router>;

protected:

  /// A mapping from vehicle ID to its occupied nodes in the lattice.
  std::unordered_map<size_t, std::vector<boost::weak_ptr<Node>>> vehicle_to_nodes_table_;

  /// Carla map.
  boost::shared_ptr<CarlaMap> map_;

public:

  TrafficLattice(
      const std::vector<boost::shared_ptr<const CarlaVehicle>>& vehicles,
      const boost::shared_ptr<CarlaMap>& map,
      const boost::shared_ptr<Router>& router,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles = boost::none);

  TrafficLattice(
      const std::vector<VehicleTuple>& vehicles,
      const boost::shared_ptr<CarlaMap>& map,
      const boost::shared_ptr<Router>& router,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles = boost::none);

  TrafficLattice(const TrafficLattice& other);

  TrafficLattice& operator=(const TrafficLattice other) {
    this->swap(other);
    return *this;
  }

  /// Find the front vehicle of the given one.
  boost::optional<std::pair<size_t, double>> front(const size_t vehicle) const;

  /// Find the back vehicle of the given one.
  boost::optional<std::pair<size_t, double>> back(const size_t vehicle) const;

  /// Find the left front vehicle of the given one.
  boost::optional<std::pair<size_t, double>> leftFront(const size_t vehicle) const;

  /// Find the left back vehicle of the given one.
  boost::optional<std::pair<size_t, double>> leftBack(const size_t vehicle) const;

  /// Find the right front vehicle of the given one.
  boost::optional<std::pair<size_t, double>> rightFront(const size_t vehicle) const;

  /// Find the right back vehicle of the given one.
  boost::optional<std::pair<size_t, double>> rightBack(const size_t vehicle) const;

  /// Return the IDs of the vehicles that are currently being tracked.
  std::unordered_set<size_t> vehicles() const;

  /// Delete a vehicle on the current lattice.
  int32_t deleteVehicle(const size_t vehicle);

  /// Add a vehicle on the current lattice.
  int32_t addVehicle(const VehicleTuple& vehicle);

  /// Update the vehicle positions to the next time instance.
  /// The object is no longer valid if this function returns false.
  bool moveTrafficForward(
      const std::vector<VehicleTuple>& vehicles,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles = boost::none);

  bool moveTrafficForward(
      const std::vector<boost::shared_ptr<const CarlaVehicle>>& vehicles,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles = boost::none);

protected:

  using Base::extend;
  using Base::shorten;
  using Base::shift;

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

  TrafficLattice() = default;

  void swap(TrafficLattice& other);

  void latticeStartAndRange(
      const std::vector<VehicleTuple>& vehicles,
      boost::shared_ptr<CarlaWaypoint>& start,
      double& range) const;

  void baseConstructor(
      const boost::shared_ptr<CarlaWaypoint>& start,
      const double range,
      const double longitudinal_resolution,
      const boost::shared_ptr<Router>& router);

  /**
   * \brief Sort the given roads into a chain.
   *
   * The function assumes the roads can be chained, i.e. there is
   * no parallel road within the input. Furthermore, the function assumes
   * the input roads are close to each other. Starting from a random road
   * in the input, all roads should be found by looking forward or backward
   * five steps.
   *
   * \param[in] The road to be sorted.
   * \return A vector of sorted roads (new roads may be added to fill in the gaps).
   */
  std::deque<size_t> sortRoads(const std::unordered_set<size_t>& roads) const;

  /// Find the waypoint correspoinding to the center of the vehicle.
  boost::shared_ptr<CarlaWaypoint> vehicleWaypoint(
      const CarlaTransform& transform) const {
    return map_->GetWaypoint(transform.location);
  }

  /// Find the waypoint corresponding to the head of the vehicle.
  boost::shared_ptr<CarlaWaypoint> vehicleHeadWaypoint(
      const CarlaTransform& transform,
      const CarlaBoundingBox& bounding_box) const;

  /// Find the waypoint corresponding to the rear of the vehicle.
  boost::shared_ptr<CarlaWaypoint> vehicleRearWaypoint(
      const CarlaTransform& transform,
      const CarlaBoundingBox& bounding_box) const;

  /**
   * \brief Register vehicles onto nodes of the lattice.
   *
   * Each vehicle may occupy several nodes in the lattice.
   *
   * \note If this function returns false, it will leave the object
   *       at an invalid state. One should not use the object anymore.
   */
  bool registerVehicles(
      const std::vector<VehicleTuple>& vehicles,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles);

  /// Get the distance from the waypoint to the starting of the road.
  double waypointToRoadStartDistance(
      const boost::shared_ptr<CarlaWaypoint>& waypoint) const {

    if (waypoint->GetLaneId() == 0)
      throw std::runtime_error("Waypoint has lane ID 0.");

    const CarlaRoad& road = map_->GetMap().GetMap().GetRoad(waypoint->GetRoadId());
    if (waypoint->GetLaneId() > 0) return road.GetLength() - waypoint->GetDistance();
    else return waypoint->GetDistance();
  }

  /// Find a front vehicle starting from a given node.
  boost::optional<std::pair<size_t, double>>
    frontVehicle(const boost::shared_ptr<const Node>& start) const;

  /// Find a back vehicle starting from a given node.
  boost::optional<std::pair<size_t, double>>
    backVehicle(const boost::shared_ptr<const Node>& start) const;

  boost::shared_ptr<const Node> vehicleHeadNode(const size_t vehicle) const {
    return vehicle_to_nodes_table_.find(vehicle)->second.back().lock();
  }

  boost::shared_ptr<const Node> vehicleRearNode(const size_t vehicle) const {
    return vehicle_to_nodes_table_.find(vehicle)->second.front().lock();
  }

}; // End class \c TrafficLattice.

} // End namespace planner.

#include <conformal_lattice_planner/traffic_lattice_inst.h>

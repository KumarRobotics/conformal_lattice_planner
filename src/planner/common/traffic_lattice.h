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

#include <cstdint>
#include <tuple>
#include <array>
#include <string>

#include <boost/format.hpp>
#include <boost/optional.hpp>

#include <carla/client/Vehicle.h>
#include <carla/client/Map.h>

#include <planner/common/utils.h>
#include <planner/common/fast_waypoint_map.h>
#include <planner/common/waypoint_lattice.h>
#include <router/common/router.h>

namespace planner {

/**
 * \brief WaypointNodeWithVehicle is similar to WaypointNode, but also keeps
 *        tracks of the vehicle at this node.
 *
 * Each node is at most associated with one vehicle.
 */
class WaypointNodeWithVehicle : public LatticeNode<WaypointNodeWithVehicle> {

protected:

  using CarlaWaypoint = carla::client::Waypoint;
  using CarlaVehicle  = carla::client::Vehicle;
  using CarlaMap      = carla::client::Map;

  using Base = LatticeNode<WaypointNodeWithVehicle>;
  using This = WaypointNodeWithVehicle;

protected:

  /// ID of the vehicle that occupies this node.
  boost::optional<size_t> vehicle_ = boost::none;

public:

  /// Default constructor.
  WaypointNodeWithVehicle() = default;

  /**
   * \brief Construct a node with a carla waypoint.
   * \param[in] waypoint A carla waypoint at which a node should be created.
   */
  WaypointNodeWithVehicle(const boost::shared_ptr<const CarlaWaypoint>& waypoint) :
    Base(waypoint) {}

  /// Get the vehicle ID registered at this node.
  boost::optional<size_t> vehicle() const { return vehicle_; }

  /// Get or set the vehicle ID at this node.
  boost::optional<size_t>& vehicle() { return vehicle_; }

  // Get the string describing the node.
  std::string string(const std::string& prefix="") const {
    boost::format waypoint_format(
        "waypoint %1% x:%2% y:%3% z:%4% r:%5% p:%6% y:%7% road:%8% lane:%9%.\n");
    std::string waypoint_msg = (waypoint_format
        % this->waypoint_->GetId()
        % this->waypoint_->GetTransform().location.x
        % this->waypoint_->GetTransform().location.y
        % this->waypoint_->GetTransform().location.z
        % this->waypoint_->GetTransform().rotation.roll
        % this->waypoint_->GetTransform().rotation.pitch
        % this->waypoint_->GetTransform().rotation.yaw
        % this->waypoint_->GetRoadId()
        % this->waypoint_->GetLaneId()).str();

    std::string distance_msg = (boost::format("node distance: %1%\n") % this->distance_).str();

    std::string vehicle_msg;
    if (!vehicle_)
      vehicle_msg = "vehicle at this node: \n";
    else
      vehicle_msg = (boost::format("vehicle at this node: %1%\n") % (*vehicle_)).str();

    return prefix + waypoint_msg + distance_msg + vehicle_msg;
    // TODO: Add the info for neighbor waypoints as well.
  }

}; // End class WaypointNodeWithVehicle.

/**
 * \brief TrafficLattice is a helper class used to track local traffic,
 *        i.e. the vehicles within a finite range neighborhood.
 *
 * \note Have to change carla/road/Map.h to compile this class.
 *       Remove the guard of LIBCARLA_WITH_GETEST, and set the
 *       function prototype from
 *       \c MapData& carla::road::Map::GetMap() to
 *       \c const MapData& carla::road::Map::GetMap() const.
 */
class TrafficLattice : public Lattice<WaypointNodeWithVehicle> {

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

  /// Stores the three waypoints for a vehicle. From index 0-2 are the
  /// waypoints corresponding to the vehicle rear, middle, and head.
  /// In the case a waypoint is missing, leave \c nullptr at the correspoinding
  /// entry.
  using VehicleWaypoints = std::array<boost::shared_ptr<CarlaWaypoint>, 3>;

private:

  using Base = Lattice<WaypointNodeWithVehicle>;

protected:

  /**
   * A mapping from vehicle ID to its occupied nodes in the lattice.
   *
   * For each entry, the key is the vehicle ID, the value is the nodes
   * occupied by the vehicle. The nodes are sorted from the vehicle rear to head.
   */
  std::unordered_map<size_t, std::vector<boost::weak_ptr<Node>>> vehicle_to_nodes_table_;

  /// Carla map, used to road and lanes.
  boost::shared_ptr<CarlaMap> map_;

  /// Fast waypoint map, used to find carla waypoints based on locations.
  boost::shared_ptr<utils::FastWaypointMap> fast_map_;

public:

  /**
   * \brief Class constructor.
   * \param[in] vehicles Vehicles to be registered onto the lattice.
   * \param[in] map The carla map used to find roads and lanes.
   * \param[in] fast_map Fast waypoint map used to find waypoints based on locations.
   * \param[in] router A router object used to find road sequences.
   * \param[out] disappear_vehicles The vehicles that cannot be registered onto the lattice.
   *                                This can be due to that the road which the vehicle is on
   *                                is not within the road sequences of the \c router.
   */
  TrafficLattice(
      const std::vector<boost::shared_ptr<const CarlaVehicle>>& vehicles,
      const boost::shared_ptr<CarlaMap>& map,
      const boost::shared_ptr<utils::FastWaypointMap>& fast_map,
      const boost::shared_ptr<router::Router>& router,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles = boost::none);

  /**
   * \brief Class constructor.
   * \param[in] vehicles Vehicles to be registered onto the lattice.
   * \param[in] map The carla map used to find roads and lanes.
   * \param[in] fast_map Fast waypoint map used to find waypoints based on locations.
   * \param[in] router A router object used to find road sequences.
   * \param[out] disappear_vehicles The vehicles that cannot be registered onto the lattice.
   *                                This can be due to that the road which the vehicle is on
   *                                is not within the road sequences of the \c router.
   */
  TrafficLattice(
      const std::vector<VehicleTuple>& vehicles,
      const boost::shared_ptr<CarlaMap>& map,
      const boost::shared_ptr<utils::FastWaypointMap>& fast_map,
      const boost::shared_ptr<router::Router>& router,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles = boost::none);

  /// Copy constructor.
  TrafficLattice(const TrafficLattice& other);

  /// Copy assignment operator.
  TrafficLattice& operator=(TrafficLattice other) {
    this->swap(other);
    return *this;
  }

  /**
   * @name Vehicle Query
   *
   * Functions in this group are used to query the vehicles around a given vehicle.
   * The interface of the functions follow the same pattern:
   *
   * \c vehicle The ID of the query vehicle.
   *
   * A target vehicle will be returned if it is found, otherwise the functions
   * returns \c nullptr.
   */
  /// @{
  boost::optional<std::pair<size_t, double>> front(const size_t vehicle) const;

  boost::optional<std::pair<size_t, double>> back(const size_t vehicle) const;

  boost::optional<std::pair<size_t, double>> leftFront(const size_t vehicle) const;

  boost::optional<std::pair<size_t, double>> leftBack(const size_t vehicle) const;

  boost::optional<std::pair<size_t, double>> rightFront(const size_t vehicle) const;

  boost::optional<std::pair<size_t, double>> rightBack(const size_t vehicle) const;
  /// @}

  /// Return the IDs of the vehicles that are currently being tracked.
  std::unordered_set<size_t> vehicles() const;

  /**
   * \brief Check if a vehicle is in the process of lane changing.
   *
   * In the case the input vehicle is not found on the lattice, the function
   * throws \c std::runtime_error exception.
   *
   * \param[in] vehicle The ID of the vehicle to be checked.
   * \return
   *  -  0 If the vehicle is keep lane
   *  - -1 If the vehicle is changing to the left lane.
   *  -  1 If the vehicle is changing to the right lane.
   */
  int32_t isChangingLane(const size_t vehicle) const;

  /**
   * \brief Delete a vehicle on the lattice.
   *
   * \param[in] vehicle The ID of the vehicle to be deleted.
   * \return
   *  - 1 If the given vehicle is deleted successfully.
   *  - 0 If the given vehicle is not being tracked on the lattice,
   *      therefore cannot be deleted.
   */
  int32_t deleteVehicle(const size_t vehicle);

  /**
   * \brief Add a vehicle on the current lattice.
   * \param[in] vehicle The vehicle to be added.
   * \return
   *  - 1 If the given vehicle is added successfully.
   *  - 0 If the vehicle already exists, the vehicle will be left untouched.
   *      It won't be updated with the input vehicle data.
   *      In the case that the head, middle, or rear waypoint of the vehicle
   *      is not on the lattice, the vehicle cannot be registered on
   *  - -1 If after adding this vehicle, a collision is detected. In this case,
   *       the vehicle won't be added, and the object will be left unchanged.
   */
  int32_t addVehicle(const VehicleTuple& vehicle);

  /**
   * \brief Add a vehicle on the current lattice.
   * \param[in] vehicle The vehicle to be added.
   * \param[in] waypoints Waypoints of this vehicle from rear to head.
   * \return
   *  - 1 If the given vehicle is added successfully.
   *  - 0 If the vehicle already exists, the vehicle will be left untouched.
   *      It won't be updated with the input vehicle data.
   *      In the case that the head, middle, or rear waypoint of the vehicle
   *      is not on the lattice, the vehicle cannot be registered on
   *  - -1 If after adding this vehicle, a collision is detected. In this case,
   *       the vehicle won't be added, and the object will be left unchanged.
   */
  int32_t addVehicle(const VehicleTuple& vehicle, const VehicleWaypoints& waypoints);

  /**
   * \brief Update all vehicles on the lattice with the new states.
   *
   * The lattice may be modified to accomodate the updated locations of all vehicles.
   *
   * \param[in] vehicles Contains the updated states of the vehicles. The IDs of the
   *                     input vehicles should match exactly the vehicles on the lattice.
   * \param[out] disappear_vehicles The vehicles that are no longer on the lattice.
   * \return False if collision is detected with the updated vehicle locations. In this
   *         case, the state of the object is left invalid and should no longer be used.
   */
  bool moveTrafficForward(
      const std::vector<VehicleTuple>& vehicles,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles = boost::none);

  /**
   * \brief Update all vehicles on the lattice with the new states.
   *
   * The lattice may be modified to accomodate the updated locations of all vehicles.
   *
   * \param[in] vehicles Contains the updated states of the vehicles. The IDs of the
   *                     input vehicles should match exactly the vehicles on the lattice.
   * \param[out] disappear_vehicles The vehicles that are no longer on the lattice.
   * \return False if collision is detected with the updated vehicle locations. In this
   *         case, the state of the object is left invalid and should no longer be used.
   */
  bool moveTrafficForward(
      const std::vector<boost::shared_ptr<const CarlaVehicle>>& vehicles,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles = boost::none);

  /// Get the string describing the lattice.
  std::string string(const std::string& prefix="") const;

protected:

  // Hide some of the inherited public functions.
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

  /// Hide the default constructor.
  TrafficLattice() = default;

  /**
   * \brief Swap the content of \c this and the given object.
   *
   * This function is used by the copy assignment operator. So that
   * the copy assigment operator can re-use the copy constructor in
   * order to avoid code duplication.
   *
   * \param[in] other Another object to swap with. After swapping,
   *                  \c other may be in an invalid state.
   */
  void swap(TrafficLattice& other);

  /**
   * \brief Find the start waypint and range of the lattice using
   *        the given vehicles.
   *
   * \param[in] vehicles The vehicles to be registered onto the lattice.
   * \param[in] vehicle_waypoints The waypoints on each vehicle.
   * \param[out] start The start waypoint of the lattice.
   * \param[out] range The range of the lattice.
   */
  void latticeStartAndRange(
      const std::vector<VehicleTuple>& vehicles,
      const std::unordered_map<size_t, VehicleWaypoints>& vehicle_waypoints,
      boost::shared_ptr<CarlaWaypoint>& start,
      double& range) const;

  void baseConstructor(
      const boost::shared_ptr<const CarlaWaypoint>& start,
      const double range,
      const double longitudinal_resolution,
      const boost::shared_ptr<router::Router>& router);

  /**
   * \brief Sort the given roads into a chain according to the
   *        road sequence given by the \c router.
   *
   * The function assumes the roads can be chained, i.e. there is
   * no parallel road within the input. Furthermore, the function assumes
   * the input roads are close to each other. Starting from a random road
   * in the input, all roads should be found by looking forward or backward
   * five steps.
   *
   * \param[in] roads The road to be sorted.
   * \return A vector of sorted roads (new roads may be added to fill in the gaps).
   */
  std::deque<size_t> sortRoads(const std::unordered_set<size_t>& roads) const;

  /**
   * \brief Find the waypoint at the center of the vehicle.
   * \param[in] transform The carla transform at the center of the vehicle.
   * \return The carla waypoint at the location of the transform.
   */
  boost::shared_ptr<CarlaWaypoint> vehicleWaypoint(
      const CarlaTransform& transform) const {
    //return map_->GetWaypoint(transform.location);
    return fast_map_->waypoint(transform.location);
  }

  /**
   * \brief Find the waypoint at the head of the vehicle.
   * \param[in] transform The carla transform of the vehicle.
   * \param[in] bounding_box The bounding box of the vehicle used to
   *                         identify the head.
   * \return the carla waypoint at the head of the vehicle.
   */
  boost::shared_ptr<CarlaWaypoint> vehicleHeadWaypoint(
      const CarlaTransform& transform,
      const CarlaBoundingBox& bounding_box) const;

  /**
   * \brief Find the waypoint at the rear of the vehicle.
   * \param[in] transform The carla transform of the vehicle.
   * \param[in] bounding_box The bounding box of the vehicle used to
   *                         identify the head.
   * \return the carla waypoint at the rear of the vehicle.
   */
  boost::shared_ptr<CarlaWaypoint> vehicleRearWaypoint(
      const CarlaTransform& transform,
      const CarlaBoundingBox& bounding_box) const;

  /**
   * \brief Find the three waypoints for each of the input vehicle.
   * \param[in] vechiles The vehicles to find waypoints for.
   * \return An unordered map with keys as vehicle IDs, and values as the
   *         waypoints for the correspoinding vehicle from rear to head.
   */
  std::unordered_map<size_t, VehicleWaypoints> vehicleWaypoints(
      const std::vector<VehicleTuple>& vehicles) const;

  /**
   * \brief Register vehicles onto nodes of the lattice.
   *
   * Each vehicle may occupy several nodes in the lattice.
   *
   * \note If this function returns false, it will leave the object
   *       at an invalid state. One should not use the object anymore.
   *
   * \param[in] vehicles The vehicles to be registered.
   * \param[in] vehicle_waypoints Waypoints for the vehicles.
   * \param[out] disappear_vehicles The vehicles which cannot be registered.
   *                                \see addVehicle() for when a vehicle cannot
   *                                be added.
   * \return False If there is collision detected after registering the vehicles.
   */
  bool registerVehicles(
      const std::vector<VehicleTuple>& vehicles,
      const std::unordered_map<size_t, VehicleWaypoints>& vehicle_waypoints,
      boost::optional<std::unordered_set<size_t>&> disappear_vehicles);

  /**
   * \brief Compute the distance of the waypoint to the start of the road.
   *
   * Although carla waypoint provide the \c GetDistance() API, the function
   * returns the distance of the waypoint relative to a fixed start regardless
   * of the direction of the waypoint. This function corrects this by
   * considering the lane ID of the query waypoint.
   *
   * \param[in] waypoint The query waypoint.
   * \return The distance of the waypoint to the start of the road.
   */
  double waypointToRoadStartDistance(
      const boost::shared_ptr<CarlaWaypoint>& waypoint) const {

    if (waypoint->GetLaneId() == 0) {
      std::string error_msg(
          "TrafficLattice::waypointToRoadStartDistance(): "
          "waypoint had lane Id 0.\n");
      std::string waypoint_msg =
        (boost::format("waypoint %1% x:%2% y:%3% z:%4% r:%5% p:%6% y:%7% road:%8% lane:%9%.\n")
         % waypoint->GetId()
         % waypoint->GetTransform().location.x
         % waypoint->GetTransform().location.y
         % waypoint->GetTransform().location.z
         % waypoint->GetTransform().rotation.roll
         % waypoint->GetTransform().rotation.pitch
         % waypoint->GetTransform().rotation.yaw
         % waypoint->GetRoadId()
         % waypoint->GetLaneId()).str();
      throw std::runtime_error(error_msg + waypoint_msg);
    }

    const CarlaRoad& road = map_->GetMap().GetMap().GetRoad(waypoint->GetRoadId());
    if (waypoint->GetLaneId() > 0) return road.GetLength() - waypoint->GetDistance();
    else return waypoint->GetDistance();
  }

  /**
   * \brief Find a front vehicle starting from a given node.
   * \param[in] start The query node
   * \return \c nullptr if a front vehicle does not exist on the lattice.
   */
  boost::optional<std::pair<size_t, double>>
    frontVehicle(const boost::shared_ptr<const Node>& start) const;

  /**
   * \brief Find a back vehicle starting from a given node.
   * \param[in] start The query node
   * \return \c nullptr if a back vehicle does not exist on the lattice.
   */
  boost::optional<std::pair<size_t, double>>
    backVehicle(const boost::shared_ptr<const Node>& start) const;

  /**
   * \brief Find the head node of a vehicle.
   * \param[in] vehicle The query vehicle ID.
   * \return The node on the lattice corresponds to the head of the vehicle.
   */
  boost::shared_ptr<const Node> vehicleHeadNode(const size_t vehicle) const {
    return vehicle_to_nodes_table_.find(vehicle)->second.back().lock();
  }

  /**
   * \brief Find the rear node of a vehicle.
   * \param[in] vehicle The query vehicle ID.
   * \return The node on the lattice corresponds to the rear of the vehicle.
   */
  boost::shared_ptr<const Node> vehicleRearNode(const size_t vehicle) const {
    return vehicle_to_nodes_table_.find(vehicle)->second.front().lock();
  }

}; // End class TrafficLattice.

} // End namespace planner.

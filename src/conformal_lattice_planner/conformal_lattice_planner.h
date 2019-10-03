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

#include <tuple>
#include <queue>
#include <string>
#include <unordered_map>
#include <boost/optional.hpp>
#include <boost/core/noncopyable.hpp>


#include <conformal_lattice_planner/traffic_lattice.h>
#include <conformal_lattice_planner/loop_router.h>
#include <conformal_lattice_planner/snapshot.h>
#include <conformal_lattice_planner/vehicle_path.h>
#include <conformal_lattice_planner/utils.h>
#include <conformal_lattice_planner/vehicle_path_planner.h>

namespace planner {

/**
 * \brief Station stores the information of the end points on a path/trajectory.
 */
class Station {

protected:

  using CarlaMap       = carla::client::Map;
  using CarlaWaypoint  = carla::client::Waypoint;
  using CarlaTransform = carla::geom::Transform;

  /**
   * \brief Stores a parent station of this station.
   *
   * The pair stores the cost-to-come at this station and the parent station.
   */
  using Parent = std::pair<double, boost::weak_ptr<Station>>;

  /**
   * \brief Stores a child station of this station.
   *
   * The tuple stores path to the child station, the cost of the path, and
   * the child station.
   */
  using Child = std::tuple<ContinuousPath, double, boost::weak_ptr<Station>>;

protected:

  /// The node that the station is most close to on the waypoint lattice.
  boost::weak_ptr<const WaypointNode> node_;

  /// The snapshot of the traffic when the ego vehicle reaches this station.
  Snapshot snapshot_;

  /**
   * \name Parent stations of this station.
   *
   * There are at most three parent stations, from the left, same, and right lanes
   * at the back of this station.
   *
   * The \c optimal_parent is a copy of one of the three parents which has the
   * minimum cost-to-come. This is mainly used to backtrace the optimal
   * path/trajectory.
   */
  /// @{
  boost::optional<Parent> left_parent_;
  boost::optional<Parent> back_parent_;
  boost::optional<Parent> right_parent_;
  boost::optional<Parent> optimal_parent_;
  /// @}

  /**
   * \name Child stations if this station.
   *
   * There are at most three child stations, to the left, same, and right lanes
   * at the front of this station.
   */
  /// @{
  boost::optional<Child> left_child_;
  boost::optional<Child> front_child_;
  boost::optional<Child> right_child_;
  /// @}

public:

  //Station(const boost::shared_ptr<const WaypointNode>& node, const Snapshot& snapshot) :
  //  node_(node), snapshot_(snapshot) {}

  Station(const Snapshot& snapshot,
          const boost::shared_ptr<const WaypointLattice<router::LoopRouter>>& waypoint_lattice,
          const boost::shared_ptr<CarlaMap>& map) :
    snapshot_(snapshot) {
      boost::shared_ptr<const WaypointNode> node = waypoint_lattice->closestNode(
          map->GetWaypoint(snapshot.ego().transform().location),
          waypoint_lattice->longitudinalResolution());
      if (!node) throw std::runtime_error("Cannot find a node on the waypoint lattice for the station");
      node_ = node;
      return;
    }

  boost::shared_ptr<const WaypointNode> node() const { return node_.lock(); }
  boost::weak_ptr<const WaypointNode>& node() { return node_; }

  const size_t id() const {
    if (!node_.lock())
      throw std::runtime_error("The station is not associated with any node.");
    return node_.lock()->id();
  }

  const CarlaTransform transform() const { return snapshot_.ego().transform(); }

  const Snapshot& snapshot() const { return snapshot_; }
  Snapshot& snapshot() { return snapshot_; }

  /// Accessors for the parent stations.
  const boost::optional<Parent>& leftParent() const { return left_parent_; }
  const boost::optional<Parent>& backParent() const { return back_parent_; }
  const boost::optional<Parent>& rightParent() const { return right_parent_; }
  const boost::optional<Parent>& optimalParent() const { return optimal_parent_; }

  /// Check whether a parent exists or not.
  const bool hasLeftParent() const { return static_cast<bool>(left_parent_); }
  const bool hasRightParent() const { return static_cast<bool>(right_parent_); }
  const bool hasBackParent() const { return static_cast<bool>(back_parent_); }
  const bool hasParent() const {
    return hasLeftParent() | hasRightParent() | hasBackParent();
  }

  /// Accessors for the child stations.
  const boost::optional<Child>& leftChild() const { return left_child_; }
  const boost::optional<Child>& rightChild() const { return right_child_; }
  const boost::optional<Child>& frontChild() const { return front_child_; }

  /// Check whether a child exists or not.
  const bool hasLeftChild() const { return static_cast<bool>(left_child_); }
  const bool hasRightChild() const { return static_cast<bool>(right_child_); }
  const bool hasFrontChild() const { return static_cast<bool>(front_child_); }
  const bool hasChild() const {
    return hasLeftChild() | hasRightChild() | hasFrontChild();
  }

  /// Update a parent station.
  /// The \c optimal_parent_ station will be updated if necessary.
  void updateLeftParent(const double cost_to_come,
                        const boost::shared_ptr<Station>& parent_station);
  void updateBackParent(const double cost_to_come,
                        const boost::shared_ptr<Station>& parent_station);
  void updateRightParent(const double cost_to_come,
                         const boost::shared_ptr<Station>& parent_station);

  /// Update a child station.
  void updateLeftChild(const ContinuousPath& path,
                       const double stage_cost,
                       const boost::shared_ptr<Station>& child_station);
  void updateFrontChild(const ContinuousPath& path,
                        const double stage_cost,
                        const boost::shared_ptr<Station>& child_station);
  void updateRightChild(const ContinuousPath& path,
                        const double stage_cost,
                        const boost::shared_ptr<Station>& child_station);

  std::string string(const std::string& prefix = "") const;

protected:

  /// Update the optimal parent station, which has the minimum cost-to-come.
  void updateOptimalParent();

}; // End class Station.

/**
 * \brief ConformalLatticePlanner implements the actual algorithm.
 */
class ConformalLatticePlanner : public VehiclePathPlanner,
                                private boost::noncopyable{

private:

  using Base = VehiclePathPlanner;
  using This = ConformalLatticePlanner;

protected:

  using CarlaWaypoint    = carla::client::Waypoint;
  using CarlaTransform   = carla::geom::Transform;
  using CarlaBoundingBox = carla::geom::BoundingBox;

protected:

  /// Simulation time step.
  double sim_time_step_;

  /// The spatio planning horizon.
  /// There is no strict temporal planning horizon, which is determined implicitly
  /// by the spatio horizion, and traffic scenario.
  double spatio_horizon_;

  /// The router to be used.
  boost::shared_ptr<router::LoopRouter> router_ = nullptr;

  /// The waypoint lattice used to find nodes for stations.
  boost::shared_ptr<WaypointLattice<router::LoopRouter>> waypoint_lattice_ = nullptr;

  /// Stores all the constructed stations indexed by the corresponding node
  /// ID on the waypoint lattice.
  std::unordered_map<size_t, boost::shared_ptr<Station>> node_to_station_table_;

  /**
   * \brief The root station in the station graph.
   *
   * The node within the root station corresponds to the location of the ego vehicle
   * at the start of the planning. Meanwhile, the snapshot within the root station
   * stores the micro traffic scenario at the start of the planning.
   */
  boost::weak_ptr<Station> root_;

public:

  /// Constructor of the class.
  ConformalLatticePlanner(
      const double sim_time_step,
      const double spatio_horizon,
      const boost::shared_ptr<router::LoopRouter>& router,
      const boost::shared_ptr<CarlaMap>& map) :
    Base(map),
    sim_time_step_(sim_time_step),
    spatio_horizon_(spatio_horizon),
    router_(router) {}

  /// Destructor of the class.
  virtual ~ConformalLatticePlanner() {}

  /// Get the root station.
  boost::shared_ptr<const Station> rootStation() const { return root_.lock(); }

  /// Get all the stations constructed by the planner. The order of the
  /// stations are not guaranteed.
  std::vector<boost::shared_ptr<const Station>> stations() const;

  /// Get the waypoint lattice constructed by the planner.
  boost::shared_ptr<const WaypointLattice<router::LoopRouter>> waypointLattice() const {
    return waypoint_lattice_;
  }

  /// Get the router used by the planner.
  boost::shared_ptr<const router::LoopRouter> router() const { return router_; }

  /// Get the nodes on the lattice, corresponding to the stations.
  std::vector<boost::shared_ptr<const Station>> nodes() const;

  /// Get the edges on the lattice, corresponding to the path.
  std::vector<ContinuousPath> edges() const;

  /**
   * \brief The main interface of the path planner.
   *
   * \param[in] ego The ID of the target vehicle.
   * \param[in] snapshot Snapshot of the current traffic scenario.
   * \return The planned path.
   */
  template<typename Path>
  Path plan(const size_t ego, const Snapshot& snapshot);

  /**
   * \brief The main interface of the path planner.
   *
   * \param[in] ego The ID of the target vehicle.
   * \param[in] snapshot Snapshot of the current traffic scenario.
   * \param[out] path The planned path.
   */
  template<typename Path>
  void plan(const size_t ego, const Snapshot& snapshot, Path& path) {
    path = plan<Path>(ego, snapshot);
    return;
  }

protected:

  /// Initialize the root station.
  void initializeRootStation(const Snapshot& snapshot);

  /// Initialize the waypoint lattice.
  void initializeWaypointLattice(const Vehicle& ego);

  /// Construct the station graph.
  void constructStationGraph();

  void exploreFrontStation(const boost::shared_ptr<Station>& station,
                           std::queue<boost::shared_ptr<Station>>& station_queue);
  void exploreLeftStation(const boost::shared_ptr<Station>& station,
                          std::queue<boost::shared_ptr<Station>>& station_queue);
  void exploreRightStation(const boost::shared_ptr<Station>& station,
                           std::queue<boost::shared_ptr<Station>>& station_queue);


  /// Get the carla vehicle by the vehicle ID.
  //boost::shared_ptr<CarlaVehicle> carlaVehicle(const size_t id) const {
  //  boost::shared_ptr<CarlaVehicle> vehicle =
  //    boost::static_pointer_cast<CarlaVehicle>(world_->GetActor(id));
  //  if (!vehicle) throw std::runtime_error("Cannot get the required vehicle in the carla server.");
  //  return vehicle;
  //}

  /// Get the carla vehicle transform by the vehicle ID.
  //CarlaTransform carlaVehicleTransform(const size_t id) const {
  //  return carlaVehicle(id)->GetTransform();
  //}

  /// Get the carla waypoint a vehicle is at by its ID.
  //boost::shared_ptr<CarlaWaypoint> carlaVehicleWaypoint(const size_t id) const {
  //  return map_->GetWaypoint(carlaVehicleTransform(id).location);
  //}

}; // End class ConformalLatticePlanner.

template<typename Path>
Path ConformalLatticePlanner::plan(const size_t ego, const Snapshot& snapshot) {
  if (ego != snapshot.ego().id())
    throw std::runtime_error("The conformal lattice planner should only plan for the ego.");

  initializeWaypointLattice(snapshot.ego());
  initializeRootStation(snapshot);
  constructStationGraph();

  // For now just return the lane following path from the root station,
  // assuming it exists.
  return std::get<0>(*(root_.lock()->frontChild()));
}

} // End namespace planner.

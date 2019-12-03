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
#include <deque>
#include <string>
#include <unordered_map>
#include <boost/optional.hpp>
#include <boost/core/noncopyable.hpp>

#include <router/common/router.h>
#include <router/loop_router/loop_router.h>
#include <planner/common/traffic_lattice.h>
#include <planner/common/snapshot.h>
#include <planner/common/vehicle_path.h>
#include <planner/common/utils.h>
#include <planner/common/vehicle_path_planner.h>
#include <planner/common/traffic_simulator.h>
#include <planner/common/intelligent_driver_model.h>

namespace planner {
namespace idm_lattice_planner {

/**
 * \brief IDMTrafficSimulator simulates the traffic forward by a given
 *        period of time.
 *
 * In contrast to TrafficSimulator, IDMTrafficSimulator assumes all vehicles
 * (including the ego) follows the intelligent driver model, which adjust
 * acceleration adaptively based on the current traffic scenario.
 */
class IDMTrafficSimulator : public TrafficSimulator {

private:

  using Base = TrafficSimulator;
  using This = IDMTrafficSimulator;

protected:

  /// Intelligent driver model.
  boost::shared_ptr<IntelligentDriverModel> idm_ = nullptr;

public:

  IDMTrafficSimulator(
      const Snapshot& snapshot,
      const boost::shared_ptr<CarlaMap>& map,
      const boost::shared_ptr<utils::FastWaypointMap>& fast_map) :
    Base(snapshot, map, fast_map),
    idm_(boost::make_shared<IntelligentDriverModel>()) {}

  const boost::shared_ptr<const IntelligentDriverModel> idm() const { return idm_; }
  boost::shared_ptr<IntelligentDriverModel>& idm() { return idm_; }

protected:

  virtual const double egoAcceleration() const override;

  virtual const double agentAcceleration(const size_t agent) const override;

}; // End class IDMTrafficSimulator.

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
   * The tuple stores the snapshot and the cost-to-come if come form this parent station.
   */
  using Parent = std::tuple<Snapshot, double, boost::weak_ptr<Station>>;

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
  boost::optional<Parent> left_parent_    = boost::none;
  boost::optional<Parent> back_parent_    = boost::none;
  boost::optional<Parent> right_parent_   = boost::none;
  boost::optional<Parent> optimal_parent_ = boost::none;
  /// @}

  /**
   * \name Child stations if this station.
   *
   * There are at most three child stations, to the left, same, and right lanes
   * at the front of this station.
   */
  /// @{
  boost::optional<Child> left_child_  = boost::none;
  boost::optional<Child> front_child_ = boost::none;
  boost::optional<Child> right_child_ = boost::none;
  /// @}

public:

  Station(const Snapshot& snapshot, const boost::shared_ptr<const WaypointNode>& node) :
    node_    (node),
    snapshot_(snapshot) {
    if (!node) {
      throw std::runtime_error(
          "Station::Station(): input node = nullptr.\n");
    }
    return;
  }

  Station(const Snapshot& snapshot,
          const boost::shared_ptr<const WaypointLattice>& waypoint_lattice,
          const boost::shared_ptr<utils::FastWaypointMap>& fast_map) :
    snapshot_(snapshot) {
    boost::shared_ptr<const WaypointNode> node = waypoint_lattice->closestNode(
        fast_map->waypoint(snapshot.ego().transform().location),
        waypoint_lattice->longitudinalResolution());
    if (!node) {
      std::string error_msg(
          "Station::Station(): "
          "cannot find a node on the waypoint lattice corresponding to the ego location.\n");
      throw std::runtime_error(
          error_msg +
          snapshot.string("snapshot: \n") +
          waypoint_lattice->string("waypoint lattice: \n"));
    }
    node_ = node;
    return;
  }

  boost::shared_ptr<const WaypointNode> node() const { return node_.lock(); }
  boost::weak_ptr<const WaypointNode>& node() { return node_; }

  const size_t id() const {
    if (!node_.lock()) {
      throw std::runtime_error(
          "Station::id(): "
          "The station is not associated with any node.");
    }
    return node_.lock()->id();
  }

  const CarlaTransform transform() const { return snapshot_.ego().transform(); }

  const Snapshot& snapshot() const { return snapshot_; }

  const double costToCome() const {
    if (!optimal_parent_) {
      throw std::runtime_error(
          "Station::costToCome(): "
          "optimal parent is not available for this station.");
    }
    return std::get<1>(*optimal_parent_);
  }

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
  void updateLeftParent(const Snapshot& snapshot,
                        const double cost_to_come,
                        const boost::shared_ptr<Station>& parent_station);
  void updateBackParent(const Snapshot& snapshot,
                        const double cost_to_come,
                        const boost::shared_ptr<Station>& parent_station);
  void updateRightParent(const Snapshot& snapshot,
                         const double cost_to_come,
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
 * \brief IDMLatticePlanner implements a planner that uses IDM to model the
 *        response of other agent vehicles. In order to reduce the number of
 *        cases to be considered, the algorithm allows only one traffic state/snapshot
 *        at each station, violating the principle of optimality.
 */
class IDMLatticePlanner : public VehiclePathPlanner,
                          private boost::noncopyable{

private:

  using Base = VehiclePathPlanner;
  using This = IDMLatticePlanner;

protected:

  using CarlaWaypoint    = carla::client::Waypoint;
  using CarlaTransform   = carla::geom::Transform;
  using CarlaBoundingBox = carla::geom::BoundingBox;

protected:

  /// Simulation time step.
  double sim_time_step_;

  /// The spatial planning horizon.
  /// There is no strict temporal planning horizon, which is determined implicitly
  /// by the spatial horizion, and traffic scenario.
  double spatial_horizon_;

  /// The router to be used.
  boost::shared_ptr<router::Router> router_ = nullptr;

  /// The waypoint lattice used to find nodes for stations.
  boost::shared_ptr<WaypointLattice> waypoint_lattice_ = nullptr;

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

  /**
   * The immediate next station to be reached by the ego.
   *
   * This is the station at the end of the first piece of last-time planned trajectory.
   */
  boost::weak_ptr<Station> cached_next_station_;

public:

  /// Constructor of the class.
  IDMLatticePlanner(
      const double sim_time_step,
      const double spatial_horizon,
      const boost::shared_ptr<router::Router>& router,
      const boost::shared_ptr<CarlaMap>& map,
      const boost::shared_ptr<utils::FastWaypointMap>& fast_map) :
    Base(map, fast_map),
    sim_time_step_(sim_time_step),
    spatial_horizon_(spatial_horizon),
    router_(router) {}

  /// Destructor of the class.
  virtual ~IDMLatticePlanner() {}

  /// Get the root station.
  boost::shared_ptr<const Station> rootStation() const { return root_.lock(); }

  /// Get all the stations constructed by the planner. The order of the
  /// stations are not guaranteed.
  //std::vector<boost::shared_ptr<const Station>> stations() const;

  /// Get the waypoint lattice constructed by the planner.
  boost::shared_ptr<const WaypointLattice> waypointLattice() const {
    return waypoint_lattice_;
  }

  /// Get the router used by the planner.
  boost::shared_ptr<const router::Router> router() const { return router_; }

  /// Get the nodes on the lattice, corresponding to the stations.
  std::vector<boost::shared_ptr<const WaypointNode>> nodes() const;

  /// Get the edges on the lattice, corresponding to the path.
  std::vector<ContinuousPath> edges() const;

  virtual DiscretePath planPath(const size_t ego, const Snapshot& snapshot) override;

protected:

  /// Check if the any of the child stations has been reached.
  bool immediateNextStationReached(const Snapshot& snapshot) const;

  /// Update the waypoint lattice.
  void updateWaypointLattice(const Snapshot& snapshot);

  /// Prune/update the station graph of last step.
  std::deque<boost::shared_ptr<Station>> pruneStationGraph(const Snapshot& snapshot);

  /// Construct the station graph.
  void constructStationGraph(std::deque<boost::shared_ptr<Station>>& station_queue);

  boost::shared_ptr<Station> connectStationToFrontNode(
      const boost::shared_ptr<Station>& station,
      const boost::shared_ptr<const WaypointNode>& target_node);
  boost::shared_ptr<Station> connectStationToLeftFrontNode(
      const boost::shared_ptr<Station>& station,
      const boost::shared_ptr<const WaypointNode>& target_node);
  boost::shared_ptr<Station> connectStationToRightFrontNode(
      const boost::shared_ptr<Station>& station,
      const boost::shared_ptr<const WaypointNode>& target_node);

  /// Compute the speed cost for a terminal station.
  const double terminalSpeedCost(const boost::shared_ptr<Station>& station) const;

  /// Compute the distance cost for a terminal station.
  const double terminalDistanceCost(const boost::shared_ptr<Station>& station) const;

  /// Compute the cost from root to this terminal, including the terminal costs.
  const double costFromRootToTerminal(const boost::shared_ptr<Station>& terminal) const;

  /// Select the optimal path sequence based on the constructed station graph.
  void selectOptimalPath(
      std::list<ContinuousPath>& path_sequence,
      std::list<boost::weak_ptr<Station>>& station_sequence) const;

  /// Merge the path segements from \c selectOptimalPath() into a single discrete path.
  DiscretePath mergePaths(const std::list<ContinuousPath>& paths) const;

}; // End class IDMLatticePlanner.

} // End namespace conformal_lattice_idm_planner.

using IDMLatticePlanner = idm_lattice_planner::IDMLatticePlanner;
} // End namespace planner.

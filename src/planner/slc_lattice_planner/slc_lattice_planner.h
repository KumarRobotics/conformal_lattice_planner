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
namespace slc_lattice_planner {

/**
 * \brief SLCTrafficSimulator simulates the traffic forward by a given
 *        period of time.
 *
 * In contrast to TrafficSimulator, SLCTrafficSimulator assumes all vehicles
 * (including the ego) follows the intelligent driver model, which adjust
 * acceleration adaptively based on the current traffic scenario.
 */
class SLCTrafficSimulator : public TrafficSimulator {

private:

  using Base = TrafficSimulator;
  using This = SLCTrafficSimulator;

protected:

  /// Intelligent driver model.
  boost::shared_ptr<IntelligentDriverModel> idm_ = nullptr;

public:

  SLCTrafficSimulator(
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
 * \brief Vertex stores the information of the end points on a trajectory.
 */
class Vertex {

protected:

  using CarlaMap       = carla::client::Map;
  using CarlaWaypoint  = carla::client::Waypoint;
  using CarlaTransform = carla::geom::Transform;

  /**
   * \brief Stores a parent vertex of this vertex.
   *
   * The tuple stores the snapshot and the cost-to-come if come form this parent veterx.
   */
  using Parent = std::tuple<Snapshot, double, boost::weak_ptr<Vertex>>;

  /**
   * \brief Stores a child vertex of this vertex.
   *
   * The tuple stores path to the child vertex, the cost of the path, and
   * the child vertex.
   */
  using Child = std::tuple<ContinuousPath, double, boost::weak_ptr<Vertex>>;

protected:

  /// The waypoint node that the vertex is most close to on the waypoint lattice.
  boost::weak_ptr<const WaypointNode> node_;

  /// The snapshot of the traffic when the ego vehicle reaches this vertex.
  /// FIXME: this snapshot is the same as the one stores in \c parent_. However, there
  ///        is no parent for the root vertex, which also needs a variable to stores
  ///        the traffic snapshot.
  Snapshot snapshot_;

  /**
   * \brief The parent of this vertex.
   *
   * There is at most one parent for each vertex. In the case that the vertex has
   * no parents, the variable is left as \c boost::none.
   */
  boost::optional<Parent> parent_ = boost::none;

  /**
   * \brief Child vertices of this vertex.
   *
   * There are at most three child vertices of each vertex. In the case that a child
   * vertex is missing, the corresponding child is left as \c boost::none.
   */
  boost::optional<Child> left_child_  = boost::none;
  boost::optional<Child> front_child_ = boost::none;
  boost::optional<Child> right_child_ = boost::none;

public:

  Vertex(const Snapshot& snapshot, const boost::shared_ptr<const WaypointNode>& node) :
    node_    (node),
    snapshot_(snapshot) {
    if (!node) {
      throw std::runtime_error(
          "Vertex::Vertex(): input node = nullptr.\n");
    }
    return;
  }

  Vertex(const Snapshot& snapshot,
         const boost::shared_ptr<const WaypointLattice>& waypoint_lattice,
         const boost::shared_ptr<utils::FastWaypointMap>& fast_map) :
    snapshot_(snapshot) {
    boost::shared_ptr<const WaypointNode> node = waypoint_lattice->closestNode(
        fast_map->waypoint(snapshot.ego().transform().location),
        waypoint_lattice->longitudinalResolution());
    if (!node) {
      std::string error_msg(
          "Vertex::Vertex(): "
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

  const CarlaTransform transform() const { return snapshot_.ego().transform(); }

  const Snapshot& snapshot() const { return snapshot_; }

  const double costToCome() const {
    if (!parent_) return 0.0;
    else return std::get<1>(*parent_);
  }

  /// Accessor for the parent vertex.
  const boost::optional<Parent>& parent() const { return parent_; }

  /// Check whether a parent vertex exists.
  const bool hasParent() const { return static_cast<bool>(parent_); }

  /// Accessors for the child vertices.
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

  /// Update the parent vertex.
  void updateParent(const Snapshot& snapshot,
                    const double cost_to_come,
                    const boost::shared_ptr<Vertex>& parent_vertex);

  /// Update a child vertex.
  void updateLeftChild(const ContinuousPath& path,
                       const double stage_cost,
                       const boost::shared_ptr<Vertex>& child_vertex);
  void updateFrontChild(const ContinuousPath& path,
                        const double stage_cost,
                        const boost::shared_ptr<Vertex>& child_vertex);
  void updateRightChild(const ContinuousPath& path,
                        const double stage_cost,
                        const boost::shared_ptr<Vertex>& child_vertex);

  /**
   * \brief Check if the relative lane position between this vertex and the given vertex.
   *
   * \param[in] other The other vertex, the node of which should be on the
   *                  back of this vertex on the waypoint lattice.
   * \return  0 If the two vertices are on the same lane.
   *          1 If this vertex if on the left of the other vertex.
   *         -1 If this vertex if on the right of the other vertex.
   *
   * The function throws a runtime exception if the relative lane positions of the
   * two vertices cannot be identified.
   */
  const int relativeLanePosition(
      const boost::shared_ptr<const Vertex>& other) const;

  std::string string(const std::string& prefix = "") const;

}; // End class Vertex.

/**
 * \brief SLCLatticePlanner implements the planner that uses IDM to model the
 *        response of other agent vehicles. The difference to the \c IDMLatticePlanner
 *        is that the \c SLCLatticePlanner allows only one lane change during the
 *        planning horizon. This greatly reduce the cases to be considered while
 *        maintaining the principle of optimality.
 */
class SLCLatticePlanner : public VehiclePathPlanner,
                          private boost::noncopyable {

private:

  using Base = VehiclePathPlanner;
  using This = SLCLatticePlanner;

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

  /// Stores all the constructed vertices.
  std::vector<boost::shared_ptr<Vertex>> all_vertices_;

  /**
   * \brief The root vertex in the vertex graph.
   *
   * The node within the root vertex corresponds to the location of the ego vehicle
   * at the start of the planning. Meanwhile, the snapshot within the root vertex
   * stores the micro traffic scenario at the start of the planning.
   */
  boost::weak_ptr<Vertex> root_;

  /**
   * The immediate next vertex to be reached by the ego.
   *
   * This is the vertex at the end of the first piece of last-time planned trajectory.
   */
  boost::weak_ptr<Vertex> cached_next_vertex_;

public:

  /// Constructor of the class.
  SLCLatticePlanner(
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
  virtual ~SLCLatticePlanner() {}

  /// Get the root vertex.
  boost::shared_ptr<const Vertex> rootVertex() const { return root_.lock(); }

  /// Get the waypoint lattice constructed by the planner.
  boost::shared_ptr<const WaypointLattice> waypointLattice() const {
    return waypoint_lattice_;
  }

  /// Get the router used by the planner.
  boost::shared_ptr<const router::Router> router() const { return router_; }

  /// Get the waypoint nodes used in the planner.
  std::vector<boost::shared_ptr<const WaypointNode>> nodes() const;

  /// Get all paths connecting the waypoint nodes in the planner.
  std::vector<ContinuousPath> edges() const;

  virtual DiscretePath planPath(const size_t ego, const Snapshot& snapshot) override;

protected:

  /// Check if the any of the child vertices has been reached.
  bool immediateNextVertexReached(const Snapshot& snapshot) const;

  /// Update the waypoint lattice.
  void updateWaypointLattice(const Snapshot& snapshot);

  /// Prune/update the vertex graph of last step.
  std::deque<boost::shared_ptr<Vertex>> pruneVertexGraph(const Snapshot& snapshot);

  /// Construct the vertex graph.
  void constructVertexGraph(std::deque<boost::shared_ptr<Vertex>>& vertex_queue);

  boost::shared_ptr<Vertex> connectVertexToFrontNode(
      const boost::shared_ptr<Vertex>& vertex,
      const boost::shared_ptr<const WaypointNode>& target_node);
  boost::shared_ptr<Vertex> connectVertexToLeftFrontNode(
      const boost::shared_ptr<Vertex>& vertex,
      const boost::shared_ptr<const WaypointNode>& target_node);
  boost::shared_ptr<Vertex> connectVertexToRightFrontNode(
      const boost::shared_ptr<Vertex>& vertex,
      const boost::shared_ptr<const WaypointNode>& target_node);

  /// Compute the speed cost for a terminal vertex
  const double terminalSpeedCost(const boost::shared_ptr<Vertex>& vertex) const;

  /// Compute the distance cost for a terminal vertex.
  const double terminalDistanceCost(const boost::shared_ptr<Vertex>& vertex) const;

  /// Compute the cost from root to this terminal, including the terminal costs.
  const double costFromRootToTerminal(const boost::shared_ptr<Vertex>& terminal) const;

  /// Select the optimal path sequence based on the constructed vertex graph.
  void selectOptimalPath(
      std::list<ContinuousPath>& path_sequence,
      std::list<boost::weak_ptr<Vertex>>& vertex_sequence) const;

  /// Merge the path segements from \c selectOptimalPath() into a single discrete path.
  DiscretePath mergePaths(const std::list<ContinuousPath>& paths) const;

}; // End class SLCLatticePlanner.

} // End namespace slc_lattice_planner.

using SLCLatticePlanner = slc_lattice_planner::SLCLatticePlanner;
} // End namespace planner.

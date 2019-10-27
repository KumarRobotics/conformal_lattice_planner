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
#include <array>
#include <string>
#include <unordered_map>
#include <boost/optional.hpp>
#include <boost/core/noncopyable.hpp>

#include <router/loop_router/loop_router.h>
#include <planner/common/traffic_lattice.h>
#include <planner/common/snapshot.h>
#include <planner/common/vehicle_path.h>
#include <planner/common/utils.h>
#include <planner/common/vehicle_path_planner.h>
#include <planner/common/traffic_simulator.h>
#include <planner/common/intelligent_driver_model.h>

namespace planner {
namespace spatiotemporal_lattice_planner {

class Vertex {

protected:

  using CarlaMap       = carla::client::Map;
  using CarlaWaypoint  = carla::client::Waypoint;
  using CarlaTransform = carla::geom::Transform;

  /**
   * \brief Stores a parent vertex of this vertex.
   *
   * The tuple stores the snapshot and the cost-to-come if come form this parent vertex.
   */
  using Parent = std::tuple<Snapshot, double, boost::weak_ptr<Vertex>>;

  /**
   * \brief Stores a child vertex of this vertex.
   *
   * The tuple stores path to the child vertex, the constant acceleration over the path,
   * the stage cost, and the child vertex.
   */
  using Child = std::tuple<ContinuousPath, double, double, boost::weak_ptr<Vertex>>;

public:

  /**
   * \brief The hard-coded interval of velocities at a station.
   *
   * At reaching the station, if the ego velocity is outside all the intervals,
   * it will be considered as an invalid trajectory option.
   *
   * Each interval is left-closed and right-open, i.e. [a, b).
   *
   * In the paper, M. McNaughton, et al, "Motion Planning for Autonomous Driving with
   * a Conformal Spatiotemporal Lattice", there is also discretizetion of time at each
   * station. In this implementation, we assume there is just one interval for the time.
   *
   * This is subject to discussion. What should be recorded at each station/vertex
   * reprents some sort of the "state" of the ego. Having time as a variable in the
   * state might indicates a flawed design of an algorithm. Time is often not a part
   * of the state vector.
   */
  static constexpr std::array<std::pair<double, double>, 3>
    kVelocityIntervalsPerStation_ {{ {0.0, 15.0}, {15.0, 30.0}, {30.0, 45.0} }};

protected:

  /// The node that the vertex is most close to on the waypoint lattice.
  boost::weak_ptr<const WaypointNode> node_;

  /// The snapshot of the traffic when the ego vehicle reaches this vertex.
  Snapshot snapshot_;

  /**
   * \name Parent vertices of this vertex.
   *
   * Each vertex may have more than one parents from the same lane, left lane,
   * or the right lane. The maximum number of parents from each lane depends on
   * the size of \c kVelocityIntervalsPerStation_.
   *
   * The \c optimal_parent is a copy of the parent which has the
   * minimum cost-to-come. This is mainly used to backtrace the optimal
   * path/trajectory.
   */
  /// @{
  std::array<boost::optional<Parent>, kVelocityIntervalsPerStation_.size()>
    left_parents_ {boost::none, boost::none, boost::none};

  std::array<boost::optional<Parent>, kVelocityIntervalsPerStation_.size()>
    back_parents_ {boost::none, boost::none, boost::none};

  std::array<boost::optional<Parent>, kVelocityIntervalsPerStation_.size()>
    right_parents_ {boost::none, boost::none, boost::none};

  boost::optional<Parent> optimal_parent_ = boost::none;
  /// @}

  /**
   * \name Child vertices if this vertex.
   *
   * Each vertex may have more than one child vertices from the same lane, left lane,
   * or the right lane. The maximum number of children from each lane depends on the
   * size of \c kVelocityIntervalsPerStation_.
   */
  /// @{
  std::array<boost::optional<Child>, kVelocityIntervalsPerStation_.size()>
    left_children_ {boost::none, boost::none, boost::none};

  std::array<boost::optional<Child>, kVelocityIntervalsPerStation_.size()>
    front_children_ {boost::none, boost::none, boost::none};

  std::array<boost::optional<Child>, kVelocityIntervalsPerStation_.size()>
    right_children_ {boost::none, boost::none, boost::none};
  /// @}

public:

  Vertex(const Snapshot& snapshot, const boost::shared_ptr<const WaypointNode>& node) :
    node_(node), snapshot_(snapshot) {
    if (!node) {
      throw std::runtime_error(
          "Vertex::Vertex(): input node = nullptr.\n");
    }
    return;
  }

  Vertex(const Snapshot& snapshot,
         const boost::shared_ptr<const WaypointLattice<router::LoopRouter>>& waypoint_lattice,
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

  const double speed() const { return snapshot_.ego().speed(); }

  const Snapshot& snapshot() const { return snapshot_; }

  const double costToCome() const {
    if (!optimal_parent_) {
      throw std::runtime_error(
          "Vertex::costToCome(): "
          "optimal parent is not available for this vertex.");
    }
    return std::get<1>(*optimal_parent_);
  }

  /// Accessors for the parent vertices.
  const std::array<boost::optional<Parent>, kVelocityIntervalsPerStation_.size()>&
    leftParents() const { return left_parents_; }

  const std::array<boost::optional<Parent>, kVelocityIntervalsPerStation_.size()>&
    backParents() const { return back_parents_; }

  const std::array<boost::optional<Parent>, kVelocityIntervalsPerStation_.size()>&
    rightParents() const { return right_parents_; }

  const boost::optional<Parent>& optimalParent() const {
    return optimal_parent_;
  }

  std::vector<Parent> validLeftParents() const { return validParents(left_parents_); }
  std::vector<Parent> validBackParents() const { return validParents(back_parents_); }
  std::vector<Parent> validRightParents() const { return validParents(right_parents_); };

  /// Check the number of parents.
  /// FIXME: There is some unnecessary copying (not much) going on here.
  ///        Should be easy to get rid of this by counting the valid parents directly.
  const size_t leftParentsSize() const { return validLeftParents().size(); }
  const size_t backParentsSize() const { return validBackParents().size(); }
  const size_t rightParentsSize() const { return validRightParents().size(); }
  const size_t parentsSize() const {
    return leftParentsSize() + backParentsSize() + rightParentsSize();
  }

  /// Check whether a parent vertex exists.
  const bool hasLeftParents() const { return leftParentsSize() > 0; }
  const bool hasBackParents() const { return backParentsSize() > 0; }
  const bool hasRightParents() const { return rightParentsSize() > 0; }
  const bool hasParents() const { return parentsSize() > 0; }

  /// Accessors for the child vertices.
  const std::array<boost::optional<Child>, kVelocityIntervalsPerStation_.size()>&
    leftChildren() const { return left_children_; }

  const std::array<boost::optional<Child>, kVelocityIntervalsPerStation_.size()>&
    frontChildren() const { return front_children_; }

  const std::array<boost::optional<Child>, kVelocityIntervalsPerStation_.size()>&
    rightChildren() const { return right_children_; }

  std::vector<Child> validLeftChildren() const { return validChildren(left_children_); }
  std::vector<Child> validFrontChildren() const { return validChildren(front_children_); }
  std::vector<Child> validRightChildren() const { return validChildren(right_children_); }

  /// Check the number of children.
  /// FIXME: There is some unnecessary copying (not much) going on here.
  ///        Should be easy to get rid of this by counting the valid parents directly.
  const size_t leftChildrenSize() const { return validLeftChildren().size(); }
  const size_t frontChildrenSize() const { return validFrontChildren().size(); }
  const size_t rightChildrenSize() const { return validRightChildren().size(); }
  const size_t childrenSize() const {
    return leftChildrenSize() + frontChildrenSize() + rightChildrenSize();
  }

  /// Check whether a child vertex exists.
  const bool hasLeftChildren() const { return leftChildrenSize() > 0; }
  const bool hasFrontChildren() const { return frontChildrenSize() > 0; }
  const bool hasRightChildren() const { return rightChildrenSize() > 0; }
  const bool hasChildren() const { return childrenSize() > 0; }

  /// Update parent vertices.
  void updateLeftParent(const Snapshot& snapshot,
                        const double cost_to_come,
                        const boost::shared_ptr<Vertex>& parent_vertex);

  void updateBackParent(const Snapshot& snapshot,
                        const double cost_to_come,
                        const boost::shared_ptr<Vertex>& parent_vertex);

  void updateRightParent(const Snapshot& snapshot,
                         const double cost_to_come,
                         const boost::shared_ptr<Vertex>& parent_vertex);

  /// Update child vertices.
  void updateLeftChild(const ContinuousPath& path,
                       const double acceleration,
                       const double stage_cost,
                       const boost::shared_ptr<Vertex>& child_vertex);

  void updateFrontChild(const ContinuousPath& path,
                        const double acceleration,
                        const double stage_cost,
                        const boost::shared_ptr<Vertex>& child_vertex);

  void updateRightChild(const ContinuousPath& path,
                        const double acceleration,
                        const double stage_cost,
                        const boost::shared_ptr<Vertex>& child_vertex);

  std::string string(const std::string& prefix = "") const;

  /// Figure out the speed interval index for the given speed.
  static boost::optional<size_t> speedIntervalIdx(const double speed) {
    // Return \c boost::none if the input speed is less than 0.
    if (speed < 0.0) return boost::none;

    for (size_t i = 0; i < kVelocityIntervalsPerStation_.size(); ++i) {
      if (speed < kVelocityIntervalsPerStation_[i].second) return i;
    }

    // Return \c boost::none if the speed is too large.
    return boost::none;
  }

protected:

  /// Update the optimal parent vertex, which has the minimum cost-to-come.
  void updateOptimalParent();

  std::vector<Parent> validParents(
      const std::array<boost::optional<Parent>,
                       kVelocityIntervalsPerStation_.size()>& parents) const {
    std::vector<Parent> valid_parents;
    for (const auto& parent : parents) {
      if (!parent) continue;
      valid_parents.push_back(*parent);
    }
    return valid_parents;
  }

  std::vector<Child> validChildren(
      const std::array<boost::optional<Child>,
                       kVelocityIntervalsPerStation_.size()>& children) const {
    std::vector<Child> valid_children;
    for (const auto& child : children) {
      if (!child) continue;
      valid_children.push_back(*child);
    }
    return valid_children;
  }

}; // End class Vertex.

class SpatiotemporalLatticePlanner : public VehiclePathPlanner,
                                     private boost::noncopyable{

private:

  using Base = VehiclePathPlanner;
  using This = SpatiotemporalLatticePlanner;

protected:

  using CarlaWaypoint    = carla::client::Waypoint;
  using CarlaTransform   = carla::geom::Transform;
  using CarlaBoundingBox = carla::geom::BoundingBox;

protected:

  static constexpr std::array<double, 7> kAccelerationOptions_ {-8.0, -4.0, -2.0, -1.0, 0.0, 1.0, 2.0};

  /// Simulation time step.
  double sim_time_step_;

  /// The spatial planning horizon.
  /// There is no strict temporal planning horizon, which is determined implicitly
  /// by the spatial horizion, and traffic scenario.
  double spatial_horizon_;

  /// The router to be used.
  boost::shared_ptr<router::LoopRouter> router_ = nullptr;

  /// The waypoint lattice used to find nodes for stations.
  boost::shared_ptr<WaypointLattice<router::LoopRouter>> waypoint_lattice_ = nullptr;

  /// Stores all the constructed vertices.
  /// The vetices are indexed by the node ID. Each node may link upto three vertices.
  std::unordered_map<
    size_t,
    std::array<boost::shared_ptr<Vertex>, Vertex::kVelocityIntervalsPerStation_.size()>>
      node_to_vertices_table_;

  /**
   * \brief The root vertex in the station graph.
   *
   * The node within the root vertex corresponds to the location of the ego vehicle
   * at the start of the planning. Meanwhile, the snapshot within the root vertex
   * stores the micro traffic scenario at the start of the planning.
   */
  boost::weak_ptr<Vertex> root_;

public:

  /// Constructor of the class.
  SpatiotemporalLatticePlanner(
      const double sim_time_step,
      const double spatial_horizon,
      const boost::shared_ptr<router::LoopRouter>& router,
      const boost::shared_ptr<CarlaMap>& map,
      const boost::shared_ptr<utils::FastWaypointMap>& fast_map) :
    Base(map, fast_map),
    sim_time_step_(sim_time_step),
    spatial_horizon_(spatial_horizon),
    router_(router) {}

  /// Destructor of the class.
  virtual ~SpatiotemporalLatticePlanner() {}

  /// Get the root vertex.
  boost::shared_ptr<const Vertex> rootVertex() const { return root_.lock(); }

  /// Get the waypoint lattice constructed by the planner.
  boost::shared_ptr<const WaypointLattice<router::LoopRouter>> waypointLattice() const {
    return waypoint_lattice_;
  }

  /// Get the router used by the planner.
  boost::shared_ptr<const router::LoopRouter> router() const { return router_; }

  // FIXME: How to get the acceleration out.
  virtual DiscretePath plan(const size_t ego, const Snapshot& snapshot) override;

protected:

  /// Check if the any of the child vertices has been reached.
  bool immediateNextVertexReached(const Snapshot& snapshot) const;

  /// Update the waypoint lattice.
  void updateWaypointLattice(const Snapshot& snapshot);

  /// Prune/update the vertex graph of last step.
  std::deque<boost::shared_ptr<Vertex>> pruneVertexGraph(const Snapshot& snapshot);

  /// Construct the vertex graph.
  void constructVertexGraph(std::deque<boost::shared_ptr<Vertex>>& vertex_queue);

  std::vector<boost::shared_ptr<Vertex>> connectVertexToFrontNode(
        const boost::shared_ptr<Vertex>& vertex,
        const boost::shared_ptr<const WaypointNode>& target_node);

  std::vector<boost::shared_ptr<Vertex>> connectVertexToLeftFrontNode(
      const boost::shared_ptr<Vertex>& vertex,
      const boost::shared_ptr<const WaypointNode>& target_node);

  std::vector<boost::shared_ptr<Vertex>> connectVertexToRightFrontNode(
      const boost::shared_ptr<Vertex>& vertex,
      const boost::shared_ptr<const WaypointNode>& target_node);

  /// Compute the speed cost for a terminal vertex.
  const double terminalSpeedCost(const boost::shared_ptr<Vertex>& vertex) const;

  /// Compute the distance cost for a terminal vertex.
  const double terminalDistanceCost(const boost::shared_ptr<Vertex>& vertex) const;

  /// Compute the cost from root to this terminal, including the terminal costs.
  const double costFromRootToTerminal(const boost::shared_ptr<Vertex>& terminal) const;

  /// Select the optimal path sequence based on the constructed vertex graph.
  std::list<std::pair<ContinuousPath, double>> selectOptimalTraj() const;

  /// Merge the path segements from \c selectOptimalTraj() into a single discrete path.
  DiscretePath mergePaths(const std::list<ContinuousPath>& paths) const;

  /**
   * \brief Try to find a vertex in the table that shared the same station and
   *        the same speed interval with the given vertex.
   *
   * The function throws runtime error if the ego speed within the input vertex
   * is not within the valid range. The valid range is defined by
   * \c Vertex::kVelocityIntervalsPerStation_.
   *
   * \param[in] vertex The query vertex.
   * \return \c nullptr if no vertex satisfying the requirement is found. Otherwise,
   *         the corresponding vertex in the table is returned.
   */
  boost::shared_ptr<Vertex> findVertexInTable(
      const boost::shared_ptr<Vertex>& vertex);

  /**
   * \brief Add a given vertex to the table.
   *
   * The function won't check wheter a similar vertex already exists in the table.
   * If this is the case, the old vertex will just be replaced with the input
   * new vertex.
   *
   * \param[in] vertex The vertex to be added to the table.
   */
  void addVertexToTable(const boost::shared_ptr<Vertex>& vertex) {
    boost::optional<size_t> idx = Vertex::speedIntervalIdx(vertex->speed());
    if (!idx) {
      std::string error_msg(
          "SpatiotemporalLatticePlanner::addVertexToTable(): ",
          "The speed of the input vertex is invalid.\n");
      error_msg += vertex->string();
      throw std::runtime_error(error_msg);
    }

    node_to_vertices_table_[vertex->node().lock()->id()][*idx] = vertex;
    return;
  }

  /**
   * \brief Find the trajectory (path + acceleration) from a parent vertex
   *        to a child vertex.
   *
   * \param[in] parent The parent vertex.
   * \param[in] child The child vertex.
   * \return \c boost::none if the given child vertex is not actually a child
   *         of the given parent vertex.
   */
  boost::optional<std::pair<ContinuousPath, double>> findTrajFromParentToChild(
      const boost::shared_ptr<Vertex>& parent,
      const boost::shared_ptr<Vertex>& child) const;

}; // End class SpatiotemporalLatticePlanner.

} // End namespace spatiotemporal_lattice_planner.
} // End namespace planner.

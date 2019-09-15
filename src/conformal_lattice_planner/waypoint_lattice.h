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
#include <vector>
#include <queue>
#include <unordered_map>

#include <boost/smart_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <carla/client/World.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/geom/Transform.h>
#include <carla/road/Road.h>
#include <carla/road/Lane.h>

#include <conformal_lattice_planner/utils.h>
#include <conformal_lattice_planner/lattice_node.h>

namespace planner {

/**
 * \brief WaypointNode Keeps track of the waypoints on a lattice.
 *
 * WaypointNode is used together with Lattice class template to
 * form the WaypointLattice class template.
 *
 * \note The copy constructor of this class performs a shallow copy,
 *       i.e. only the shared and weak pointers are copied. This makes
 *       sense since the class copy constructor won't know which piece
 *       of memory the pointers should point to. In case one would like
 *       to redirect the pointers in the class to other objects, use
 *       the accessor interfaces.
 */
class WaypointNode : public LatticeNode<WaypointNode> {

protected:

  using CarlaWaypoint = carla::client::Waypoint;

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

public:

  /// Default constructor.
  WaypointNode() = default;

  /**
   * \brief Construct a node with a carla waypoint.
   * \param[in] waypoint A carla waypoint at which a node should be created.
   */
  WaypointNode(const boost::shared_ptr<CarlaWaypoint>& waypoint) :
    waypoint_(waypoint) {}

  /// Get or set the pointer to the carla waypoint of the node.
  boost::shared_ptr<CarlaWaypoint>& waypoint() {
    return waypoint_;
  }

  /// Get the const pointer to the carla waypoint of the node.
  boost::shared_ptr<const CarlaWaypoint> waypoint() const {
    return boost::const_pointer_cast<const CarlaWaypoint>(waypoint_);
  }

  /// Get or set the distance of the node.
  double& distance() { return distance_; }

  // Get the distance of the node.
  const double distance() const { return distance_; }
}; // End class WaypointNode.

/**
 * \brief Lattice is a 2-D graph compliant to the road structure.
 *
 * The lattice is constructed follow the road structure. Nodes on the lattice
 * corresponds to the waypoints on each lane of the road. Left and right edges
 * of a node are directed, indicating whether a left or right lane change is
 * possible. The lattice is paved following the road sequence given by the router.
 *
 * See \c WaypointNode to find the interface required for the \c Node template.
 * See \c Router to find the interface required for the \c Router template.
 */
template<typename Node, typename Router>
class Lattice {

protected:

  using CarlaWorld     = carla::client::World;
  using CarlaMap       = carla::client::Map;
  using CarlaWaypoint  = carla::client::Waypoint;
  using CarlaLane      = carla::road::Lane;
  using CarlaTransform = carla::geom::Transform;
  using CarlaVector3D  = carla::geom::Vector3D;

protected:

  /// Router used to query the roads and front waypoints.
  boost::shared_ptr<Router> router_;

  /// Entry nodes of the lattice (nodes that do not have back nodes).
  std::vector<boost::weak_ptr<Node>> lattice_entries_;

  /// Exit nodes of the lattice (nodes that do not have front nodes).
  std::vector<boost::weak_ptr<Node>> lattice_exits_;

  /// A mapping from carla waypoint ID to the corresponding node in the lattice.
  std::unordered_map<size_t, boost::shared_ptr<Node>> waypoint_to_node_table_;

  /**
   * A mapping from road+lane IDs to the carla waypoint IDs on this road+lane.
   *
   * This variable is used to quickly find the closest node given a carla waypoint.
   *
   * For each element in the map, the key is the hash value combining the road
   * ID and the lane ID, the value is the waypoints on this road and lane.
   */
  std::unordered_map<size_t, std::vector<size_t>> roadlane_to_waypoints_table_;

  /// Range resolution (distance between two connected nodes) in the
  /// longitudinal direction.
  double longitudinal_resolution_;

public:

  /**
   * \brief Constructor of the class.
   *
   * \param[in] start The starting point of the lattice.
   * \param[in] range The desired range of the lattice.
   * \param[in] longitudinal_resolution
   *            The distance between two consecutive nodes of the lattice on the same lane.
   * \param[in] router Used to tell roads and waypoints.
   */
  Lattice(const boost::shared_ptr<CarlaWaypoint>& start,
          const double range,
          const double longitudinal_resolution,
          const boost::shared_ptr<Router>& router);

  /// Copy constructor.
  Lattice(const Lattice& other);

  /// Copy assignment operator.
  Lattice& operator=(Lattice other) {
    this->swap(other);
    return *this;
  }

  /// Get the entry nodes of the lattice.
  std::vector<boost::shared_ptr<const Node>> latticeEntries() const {
    std::vector<boost::shared_ptr<const Node>> output;
    for (const auto& node : lattice_entries_) output.push_back(node.lock());
    return output;
  }

  /// Get the exit nodes of the lattice.
  std::vector<boost::shared_ptr<const Node>> latticeExits() const {
    std::vector<boost::shared_ptr<const Node>> output;
    for (const auto& node : lattice_exits_) output.push_back(node.lock());
    return output;
  }

  /// Return all nodes maintained by the lattice.
  std::unordered_map<size_t, boost::shared_ptr<const Node>> nodes() const;

  /** \brief Return all edges maintained by the lattice.
   *
   * The returned edges are directed. For example edge <1, 2> and <2, 1>
   * may both appear in the returned vector.
   */
  std::vector<std::pair<size_t, size_t>> edges() const;

  /**
   * \brief Return the range of the lattice.
   *
   * The range is defined as the largest distance difference among
   * the nodes in the lattice.
   */
  double range() const;

  /**
   * \brief Extend the range of the lattice.
   *
   * The lattice will always be extended in the forward direction, which
   * is defined by the road sequence given by the router.
   *
   * \param[in] range The new range of the lattice. If this is less than
   *                  the current range, no operation is performed.
   */
  void extend(const double range);

  /**
   * \brief Shorten the range of the current lattice.
   *
   * The lattice will always be shortened from the back.
   *
   * \param[in] range The new range of the lattice. If this is more than
   *                  the current range, no operation is performed.
   */
  void shorten(const double range);

  /**
   * \brief Shift the lattice forward by some distance.
   *
   * The forward direction is defined by the road sequence in the router.
   *
   * \param[in] movement How much distance to shift the lattice forward.
   */
  void shift(const double movement) {
    const double range = this->range();
    extend(range + movement);
    shorten(range);
    return;
  }

  /**
   * \brief Find the closest node on the lattice given a carla waypoint.
   *
   * The function will only search for the nodes that are on the same
   * road and lane of the given carla waypoint. If such node does not
   * exist, or the distance between the found node and the query waypoint
   * is larger than \c tolerance, \c nullptr is returned.
   *
   * \param[in] waypoint The query carla waypoint.
   * \param[in] tolerance The maximum tolerable distance between
   *                      the waypoint and the found node.
   * \return The node closest to the query waypoint.
   */
  boost::shared_ptr<const Node> closestNode(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double tolerance) const {

    Lattice* unconst_this = const_cast<Lattice*>(this);
    return unconst_this->closestNode(waypoint, tolerance);
  }

  /**
   * @name Node Query
   *
   * This group of functions search for a target node relative
   * to the given \c query waypoint on the lattice. In the case that nothing
   * is found (e.g. the given \c range exceeds the range of the lattice,
   * the query waypoint cannot be found on the lattice),
   * \c nullptr is returned. The interface of the functions follows the same
   * pattern:
   *
   * \c query The query waypoint.
   * \c range The range to search ahead or back.
   *
   * \note Functions like \c leftFront() is different from \c frontLeft().
   * \c leftFront() finds the left waypoint first and then searches forward.
   * \c frontLeft() searches forward first and then finds the left waypoint.
   * If neither of the functions returns \c nullptr, they should return the
   * same node on the lattice.
   *
   */
  /// @{
  boost::shared_ptr<const Node> front(
      const boost::shared_ptr<const CarlaWaypoint>& query,
      const double range) const;

  boost::shared_ptr<const Node> back(
      const boost::shared_ptr<const CarlaWaypoint>& query,
      const double range) const;

  boost::shared_ptr<const Node> leftFront(
      const boost::shared_ptr<const CarlaWaypoint>& query,
      const double range) const;

  boost::shared_ptr<const Node> frontLeft(
      const boost::shared_ptr<const CarlaWaypoint>& query,
      const double range) const;

  boost::shared_ptr<const Node> leftBack(
      const boost::shared_ptr<const CarlaWaypoint>& query,
      const double range) const;

  boost::shared_ptr<const Node> backLeft(
      const boost::shared_ptr<const CarlaWaypoint>& query,
      const double range) const;

  boost::shared_ptr<const Node> rightFront(
      const boost::shared_ptr<const CarlaWaypoint>& query,
      const double range) const;

  boost::shared_ptr<const Node> frontRight(
      const boost::shared_ptr<const CarlaWaypoint>& query,
      const double range) const;

  boost::shared_ptr<const Node> rightBack(
      const boost::shared_ptr<const CarlaWaypoint>& query,
      const double range) const;

  boost::shared_ptr<const Node> backRight(
      const boost::shared_ptr<const CarlaWaypoint>& query,
      const double range) const;
  /// @}

protected:

  /**
   * \brief Hide the default constructor.
   *
   * A default constructor does not make sense if this class is used directly
   * by an end user. However, a derived class might need this to implement
   * its constructor. It is then the derived class's constructor's
   * responsibility to initialize the variables in this (base) class.
   *
   * FIXME: Is there a better solution for this situation?
   *        Idealy, I would like to reuse the custom constructor of this class
   *        in the derived class's constructor to avoid duplication of code.
   *        The second best answer in the following link can be an option:
   *        <https://stackoverflow.com/questions/3821064/call-a-base-class-constructor-later-not-in-the-initializer-list-in-c>
   */
  Lattice() = default;

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
  void swap(Lattice& other);

  /**
   * \brief Add new element to the \c waypoint_to_node_ table.
   * \param[in] waypoint_id The id of the carla waypoint.
   * \param[in] node The node corresponding to the waypoint.
   */
  void augmentWaypointToNodeTable(
      const size_t waypoint_id,
      const boost::shared_ptr<Node>& node) {
    waypoint_to_node_table_[waypoint_id] = node;
    return;
  }

  /**
   * \brief Remove a node from the \c waypoint_to_node table.
   * \param[in] waypoint_id The ID of the waypoint contained within
   *                        the node to be removed.
   */
  void reduceWaypointToNodeTable(const size_t waypoint_id) {
    waypoint_to_node_table_.erase(waypoint_id);
    return;
  }

  /**
   * \brief Add new element to the \c roadlane_to_waypoints_table_.
   * \param[in] waypoint The waypoint to be added to the table.
   */
  void augmentRoadlaneToWaypointsTable(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint);

  /**
   * \brief Remove a element from the \c roadlane_to_waypoints_table_.
   * \param[in] waypoint The waypoint to be removed from the table.
   */
  void reduceRoadlaneToWaypointsTable(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint);

  /**
   * \brief Find the closest node on the lattice given a carla waypoint.
   *
   * The function will only search for the nodes that are on the same
   * road and lane of the given carla waypoint. If such node does not
   * exist, or the distance between the found node and the query waypoint
   * is larger than \c tolerance, \c nullptr is returned.
   *
   * \param[in] waypoint The query carla waypoint.
   * \param[in] tolerance The maximum tolerable distance between
   *                      the waypoint and the found node.
   * \return The node closest to the query waypoint.
   */
  boost::shared_ptr<Node> closestNode(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double tolerance);

  /// Find the entry and exit nodes on the lattice.
  void findLatticeEntriesAndExits();

  /**
   * \brief Find the front waypoint of the query waypoint.
   * \param[in] waypoint The query waypoint.
   * \param[in] range The distance to search forward.
   */
  boost::shared_ptr<CarlaWaypoint> findFrontWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double range) const {
    return router_->frontWaypoint(waypoint, range);
  }

  /// Find the left waypoint of the query one.
  boost::shared_ptr<CarlaWaypoint> findLeftWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const {
    return waypoint->GetLeft();
  }

  /// Find the right waypoint of the query one.
  boost::shared_ptr<CarlaWaypoint> findRightWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const {
    return waypoint->GetRight();
  }

  /**
   * \brief Extend the lattice in the forward direction.
   *
   * Used by \c extend() function.
   *
   * \param[in] node The node of which a front node is added to the lattice.
   * \param[in] range The range of the lattice.
   * \param[out] nodes_queue If the distance of the front node is within range
   *                         the lattice, and this front node is actually a new
   *                         node, it will be pushed into this queue.
   */
  void extendFront(const boost::shared_ptr<Node>& node,
                   const double range,
                   std::queue<boost::shared_ptr<Node>>& nodes_queue);

  /**
   * \brief Extend the lattice to the left.
   *
   * Used by \c extend() function.
   *
   * \param[in] node The node of which a left node is added to the lattice.
   * \param[out] nodes_queue If the found left node is new, it will be pushed
   *                         into this queue.
   */
  void extendLeft(const boost::shared_ptr<Node>& node,
                  std::queue<boost::shared_ptr<Node>>& nodes_queue);

  /**
   * \brief Extend the lattice to the right.
   *
   * Used by \c extend() function.
   *
   * \param[in] node The node of which a right node is added to the lattice.
   * \param[out] nodes_queue If the found right node is new, it will be pushed
   *                         into this queue.
   */
  void extendRight(const boost::shared_ptr<Node>& node,
                   std::queue<boost::shared_ptr<Node>>& nodes_queue);

  /**
   * \brief Update the distance of all nodes.
   *
   * This function is only used by the \c shorten() function. After which,
   * the distance of the nodes requires to be updated. After updating, the
   * minimum distance of all nodes is always 0.
   */
  void updateNodeDistance();

}; // End class Lattice.

/**
 * \brief WaypointLattice is a helper class used to query
 *        the relative waypoints of the given waypoint.
 */
template<typename Router>
using WaypointLattice = Lattice<WaypointNode, Router>;
} // End namespace planner.

#include <conformal_lattice_planner/waypoint_lattice_inst.h>

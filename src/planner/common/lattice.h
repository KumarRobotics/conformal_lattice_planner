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

#include <vector>
#include <queue>
#include <unordered_map>
#include <string>

#include <boost/smart_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <carla/client/World.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/geom/Transform.h>
#include <carla/road/Road.h>
#include <carla/road/Lane.h>

#include <router/common/router.h>

namespace planner {

/**
 * \brief LatticeNode is supposed to be the base class of all nodes
 *        to be used with the the Lattice class.
 *
 * This class provides the interface for accessing and setting the
 * nodes around a node object.
 */
template<typename Derived>
class LatticeNode {

protected:

  using CarlaMap      = carla::client::Map;
  using CarlaWaypoint = carla::client::Waypoint;

protected:

  /// Carla waypoint of this node.
  boost::shared_ptr<const CarlaWaypoint> waypoint_ = nullptr;

  /**
   * The distance of this waypoint in the lattice.
   *
   * Note this is different than the \c s attribute of a carla waypoint,
   * which is the distance of the waypoint on the road it belongs to.
   */
  double distance_ = 0.0;

  /// Front node.
  boost::weak_ptr<Derived> front_;

  /// Back node.
  boost::weak_ptr<Derived> back_;

  /// Left node.
  boost::weak_ptr<Derived> left_;

  /// Right node.
  boost::weak_ptr<Derived> right_;

public:

  // Default constructor.
  LatticeNode() = default;

  /**
   * \brief Construct a node with a carla waypoint.
   * \param[in] waypoint A carla waypoint at which a node should be created.
   */
  LatticeNode(const boost::shared_ptr<const CarlaWaypoint>& waypoint) :
    waypoint_(waypoint) {}

  /// Get the ID of the underlying waypoint, which can also be used as the
  /// hash tag for the node objects.
  const size_t id() const {
    return waypoint_->GetId();
  }

  /// Get or set the pointer to the carla waypoint of the node.
  boost::shared_ptr<const CarlaWaypoint>& waypoint() {
    return waypoint_;
  }

  /// Get the const pointer to the carla waypoint of the node.
  boost::shared_ptr<const CarlaWaypoint> waypoint() const {
    return boost::const_pointer_cast<const CarlaWaypoint>(waypoint_);
  }

  /// Get the curvature at the node.
  const double curvature(const boost::shared_ptr<CarlaMap>& map) const {
    return utils::curvatureAtWaypoint(waypoint_, map);
  }

  /// Get or set the distance of the node.
  double& distance() { return distance_; }

  // Get the distance of the node.
  const double distance() const { return distance_; }

  /** @name Accessors
   *
   * front(), back(), left(), right() returns reference
   * of the boost weak pointers stored in the object, so that one can
   * update the weak pointers directly.
   */
  /// @{

  boost::weak_ptr<Derived>& front() {
    return front_;
  }

  boost::weak_ptr<Derived>& back() {
    return back_;
  }

  boost::weak_ptr<Derived>& left() {
    return left_;
  }

  boost::weak_ptr<Derived>& right() {
    return right_;
  }

  /// @}

  /** @name const Accessors
   *
   * front(), back(), left(), right() returns boost shared pointers
   * pointering to const LatticeNode objects.
   */
  /// @{

  boost::shared_ptr<const Derived> front() const {
    return boost::const_pointer_cast<const Derived>(front_.lock());
  }

  boost::shared_ptr<const Derived> back() const {
    return boost::const_pointer_cast<const Derived>(back_.lock());
  }

  boost::shared_ptr<const Derived> left() const {
    return boost::const_pointer_cast<const Derived>(left_.lock());
  }

  boost::shared_ptr<const Derived> right() const {
    return boost::const_pointer_cast<const Derived>(right_.lock());
  }

  /// @}

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
 */
template<typename Node>
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
  boost::shared_ptr<router::Router> router_;

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
  Lattice(const boost::shared_ptr<const CarlaWaypoint>& start,
          const double range,
          const double longitudinal_resolution,
          const boost::shared_ptr<router::Router>& router);

  /// Copy constructor.
  Lattice(const Lattice& other);

  /// Copy assignment operator.
  Lattice& operator=(Lattice other) {
    this->swap(other);
    return *this;
  }

  const double longitudinalResolution() const { return longitudinal_resolution_; }

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
  void extend(double range);

  /**
   * \brief Shorten the range of the current lattice.
   *
   * The lattice will always be shortened from the back.
   *
   * \param[in] range The new range of the lattice. If this is more than
   *                  the current range, no operation is performed.
   */
  void shorten(double range);

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

  /// Get the string describing the lattice.
  std::string string(const std::string& prefix="") const;

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
} // End namespace planner.

#include <planner/common/lattice_inst.h>

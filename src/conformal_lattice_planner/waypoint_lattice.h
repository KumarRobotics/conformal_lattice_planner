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

#include <conformal_lattice_planner/utils.h>

namespace planner {

/**
 * \brief Keeps track of the waypoints on a lattice.
 *
 * \note The copy constructor of this class performs a shallow copy,
 *       i.e. only the shared and weak pointers are copied. This makes
 *       sense since the class copy constructor won't know which piece
 *       of memory the pointers should point to. In case one would like
 *       to redirect the pointers in the class to other objects, use
 *       the accessor interfaces.
 */
class WaypointNode {

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

  /// Front node.
  boost::weak_ptr<WaypointNode> front_;

  /// Back node.
  boost::weak_ptr<WaypointNode> back_;

  /// Left node.
  boost::weak_ptr<WaypointNode> left_;

  /// Right node.
  boost::weak_ptr<WaypointNode> right_;

public:

  WaypointNode() = default;

  WaypointNode(const boost::shared_ptr<CarlaWaypoint>& waypoint) :
    waypoint_(waypoint) {}

  /** @name Accessors
   *
   * \c front(), \c back(), \c left(), \c right() returns reference
   * of the boost weak pointers stored in the object, so that one can
   * update the weak pointers directly.
   */
  /// @{

  boost::shared_ptr<CarlaWaypoint>& waypoint() {
    return waypoint_;
  }

  double& distance() { return distance_; }

  boost::weak_ptr<WaypointNode>& front() {
    return front_;
  }

  boost::weak_ptr<WaypointNode>& back() {
    return back_;
  }

  boost::weak_ptr<WaypointNode>& left() {
    return left_;
  }

  boost::weak_ptr<WaypointNode>& right() {
    return right_;
  }

  /// @}

  /** @name const Accessors
   *
   * \c front(), \c back(), \c left(), \c right() returns boost shared pointers
   * pointering to const \c WaypointNode objects.
   */
  /// @{

  boost::shared_ptr<const CarlaWaypoint> waypoint() const {
    return boost::const_pointer_cast<const CarlaWaypoint>(waypoint_);
  }

  const double distance() const { return distance_; }

  boost::shared_ptr<const WaypointNode> front() const {
    return boost::const_pointer_cast<const WaypointNode>(front_.lock());
  }

  boost::shared_ptr<const WaypointNode> back() const {
    return boost::const_pointer_cast<const WaypointNode>(back_.lock());
  }

  boost::shared_ptr<const WaypointNode> left() const {
    return boost::const_pointer_cast<const WaypointNode>(left_.lock());
  }

  boost::shared_ptr<const WaypointNode> right() const {
    return boost::const_pointer_cast<const WaypointNode>(right_.lock());
  }

  /// @}

}; // End class WaypointNode.

/**
 * \brief Conformal lattice compliant to the road structure.
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

  /// A beginning node of the lattice, i.e. a (not the) node with distance 0.0.
  boost::shared_ptr<Node> lattice_entry_;

  /// A end node of the lattice, i.e. a (not the) node with maximum distance.
  boost::shared_ptr<Node> lattice_exit_;

  /// A mapping from carla waypoint ID to the corresponding node in the lattice.
  std::unordered_map<size_t, boost::shared_ptr<Node>> waypoint_to_node_table_;

  /// A mapping from road+lane IDs to the carla waypoint IDs on this road+lane.
  std::unordered_map<size_t, std::vector<size_t>> roadlane_to_waypoints_table_;

  /// Range resolution in the longitudinal direction.
  double longitudinal_resolution_;

public:

  /**
   * \brief Constructor of the class.
   *
   * \param[in] start The starting point of the lattice.
   * \param[in] range The desired range of the lattice.
   * \param[in] longitudinal_resolution
   *            The distance between two consecutive nodes of the lattice on the same lane.
   */
  Lattice(const boost::shared_ptr<CarlaWaypoint>& start,
          const double range,
          const double longitudinal_resolution);

  /// Copy constructor.
  Lattice(const Lattice& other);

  /// Copy assignment operator.
  Lattice& operator=(Lattice other);

  /// Get the entry node of the lattice, which has distance 0.0.
  boost::shared_ptr<const Node> latticeEntry() const {
    return lattice_entry_;
  }

  /// Get the exit node of the lattice, which corresponds to the range of the lattice.
  boost::shared_ptr<const Node> latticeExit() const {
    return lattice_exit_;
  }

  /**
   * \brief Extend the range of the lattice.
   *
   * \param[in] range The new range of the lattice. If this is less than
   *                  the current range, no operation is performed.
   */
  void extend(const double range);

  /**
   * \brief Shorten the range of the current lattice.
   *
   * \param[in] range The new range of the lattice. If this is more than
   *                  the current range, no operation is performed.
   */
  void shorten(const double range);

  /**
   * \brief Shift the lattice forward by some distance.
   *
   * \param[in] movement How much distance to shift the lattice forward.
   */
  void shift(const double movement) {
    const double range = lattice_exit_->distance() - lattice_entry_->distance();
    extend(range + movement);
    shorten(range);
    return;
  }

  /**
   * @name Node Query
   *
   * This group of functions search for a target node relative
   * to the given \c query waypoint on the lattice. In the case that nothing
   * is found (e.g. the given \c range exceeds the range of the lattice),
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

  /// Used by copy assigment operator.
  void swap(Lattice& other);

  /// @name Maintaining the tables within the class.
  /// @{
  void augmentWaypointToNodeTable(
      const size_t waypoint_id,
      const boost::shared_ptr<Node>& node) {
    waypoint_to_node_table_[waypoint_id] = node;
    return;
  }

  void reduceWaypointToNodeTable(const size_t waypoint_id) {
    waypoint_to_node_table_.erase(waypoint_id);
    return;
  }

  void augmentRoadlaneToWaypointsTable(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) {

    //const size_t roadlane_id = hashRoadLaneIds(
    //    waypoint->GetRoadId(), waypoint->GetLaneId());
    size_t roadlane_id = 0;
    utils::hashCombine(roadlane_id, waypoint->GetRoadId(), waypoint->GetLaneId());

    // Initialize this entry in the table.
    if (roadlane_to_waypoints_table_.find(roadlane_id) ==
        roadlane_to_waypoints_table_.end())
      roadlane_to_waypoints_table_[roadlane_id] = std::vector<size_t>();

    // Add the waypoint ID to this road+lane.
    roadlane_to_waypoints_table_[roadlane_id].push_back(waypoint->GetId());
    return;
  }

  void reduceRoadlaneToWaypointsTable(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) {

    size_t roadlane_id = 0;
    utils::hashCombine(roadlane_id, waypoint->GetRoadId(), waypoint->GetLaneId());

    if (roadlane_to_waypoints_table_.find(roadlane_id) !=
        roadlane_to_waypoints_table_.end()) {
      std::vector<size_t>& waypoints = roadlane_to_waypoints_table_[roadlane_id];
      std::vector<size_t>::iterator end_iter = std::remove_if(
          waypoints.begin(), waypoints.end(),
          [&waypoint](const size_t id)->bool{
            return (id == waypoint->GetId());
          });

      waypoints.erase(end_iter, waypoints.end());
    }
    return;
  }
  /// @}

  /// Find the node in the lattice that is closest to the given \c waypoint.
  /// If no such node is found within the given \c tolerance, \c nullptr
  /// is returned.
  boost::shared_ptr<Node> closestNode(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double tolerance) const;

  /// @name Functions required by \c extend() or \c shorten()
  /// @{
  boost::shared_ptr<CarlaWaypoint> findFrontWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double range) const;

  boost::shared_ptr<CarlaWaypoint> findLeftWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const {
    boost::shared_ptr<CarlaWaypoint> left_waypoint = waypoint->GetRight();
    if (left_waypoint && left_waypoint->GetType()==CarlaLane::LaneType::Driving) return left_waypoint;
    else return nullptr;
  }

  boost::shared_ptr<CarlaWaypoint> findRightWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const {
    boost::shared_ptr<CarlaWaypoint> right_waypoint = waypoint->GetLeft();
    if (right_waypoint && right_waypoint->GetType()==CarlaLane::LaneType::Driving) return right_waypoint;
    else return nullptr;
  }

  void extendFront(const boost::shared_ptr<Node>& node,
                   const double range,
                   std::queue<boost::shared_ptr<Node>>& nodes_queue);

  void extendLeft(const boost::shared_ptr<Node>& node,
                  const double range,
                  std::queue<boost::shared_ptr<Node>>& nodes_queue);

  void extendRight(const boost::shared_ptr<Node>& node,
                   const double range,
                   std::queue<boost::shared_ptr<Node>>& nodes_queue);

  void updateNodeDistance();
  /// @}

}; // End class Lattice.

using WaypointLattice = Lattice<WaypointNode>;
} // End namespace planner.

#include <conformal_lattice_planner/waypoint_lattice_inst.h>

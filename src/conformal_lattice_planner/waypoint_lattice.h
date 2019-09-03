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
#include <limits>
#include <stdexcept>
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>

#include <boost/format.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/functional/hash.hpp>

#include <carla/client/World.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/geom/Transform.h>

#include <conformal_lattice_planner/utils.h>

namespace planner {

class WaypointNode {

protected:

  using CarlaWaypoint = carla::client::Waypoint;

protected:

  boost::shared_ptr<CarlaWaypoint> waypoint_ = nullptr;

  /**
   * The distance of this waypoint in the lattice.
   *
   * Note this is different than the \c s attribute of a carla waypoint,
   * which is the distance of the waypoint on the road it belongs to.
   */
  double distance_ = 0.0;

  boost::weak_ptr<WaypointNode> front_;
  boost::weak_ptr<WaypointNode> back_;
  boost::weak_ptr<WaypointNode> left_;
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
  const double longitudinal_resolution_;

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
   * \param[in] The new range of the lattice. If this is less than
   *            the current range, no operation is performed.
   */
  void extend(const double range);

  /**
   * \brief Shorten the range of the current lattice.
   *
   * \param[in] The new range of the lattice. If this is more than
   *            the current range, no operation is performed.
   */
  void shorten(const double range);

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

  void augmentWaypointToNodeTable(
      const size_t waypoint_id,
      const boost::shared_ptr<Node>& node) {
    waypoint_to_node_table_[waypoint_id] = node;
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

  boost::shared_ptr<Node> closestNode(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double tolerance) const;

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

}; // End class Lattice.

template<typename Node>
Lattice<Node>::Lattice(
    const boost::shared_ptr<CarlaWaypoint>& start,
    const double range,
    const double longitudinal_resolution) :
  longitudinal_resolution_(longitudinal_resolution) {

  if (range <= 5*longitudinal_resolution_) {
    throw std::runtime_error(
        (boost::format("The given range [%1%] is too small."
                       "Resolution should be at least 5xlongitudinal_resolution.") % range).str());
  }

  if (longitudinal_resolution_ > 5.0) {
    throw std::runtime_error(
        (boost::format("The given longitudinal resolution [%1%] is too large."
                       "Resolution should be within (0m, 5.0).") % longitudinal_resolution_).str());
  }

  // Create the start node.
  boost::shared_ptr<Node> start_node = boost::make_shared<Node>(start);
  start_node->distance() = 0.0;
  lattice_entry_ = start_node;
  lattice_exit_ = start_node;

  augmentWaypointToNodeTable(start->GetId(), start_node);
  augmentRoadlaneToWaypointsTable(start);

  // Construct the lattice.
  extend(range);

  //std::printf("Total nodes # on lattice: %lu\n", waypoint_to_node_table_.size());
  return;
}

template<typename Node>
void Lattice<Node>::extend(const double range) {

  // A queue of nodes to be explored, starting from the current
  // lattice exit.
  std::queue<boost::shared_ptr<Node>> nodes_queue;
  nodes_queue.push(lattice_exit_);

  while (!nodes_queue.empty()) {
    // Get the next node to explore and remove it from the queue.
    boost::shared_ptr<Node> node = nodes_queue.front();
    nodes_queue.pop();

    extendFront(node, range, nodes_queue);
    extendLeft(node, range, nodes_queue);
    extendRight(node, range, nodes_queue);
  }

  // All nodes that has been added now must have front, left, and right neighbors.
  // We still have to fill in the back neighbor of each node.
  for (auto& item : waypoint_to_node_table_) {
    boost::shared_ptr<Node> node = item.second;
    if (node->front().lock()) {
      const size_t front_waypoint_id = (node->front()).lock()->waypoint()->GetId();
      waypoint_to_node_table_[front_waypoint_id]->back() = node;
    }
  }

  return;
}

template<typename Node>
void Lattice<Node>::extendFront(
    const boost::shared_ptr<Node>& node,
    const double range,
    std::queue<boost::shared_ptr<Node>>& nodes_queue) {

  // Find the front waypoint.
  boost::shared_ptr<CarlaWaypoint> front_waypoint =
    findFrontWaypoint(node->waypoint(), longitudinal_resolution_);

  if (front_waypoint) {
    // Find the front node correspoinding to the front waypoint if it exists.
    boost::shared_ptr<Node> front_node = closestNode(front_waypoint, 0.2);

    if (!front_node) {
      // This front node does not exist yet.
      front_node = boost::make_shared<Node>(front_waypoint);
      front_node->distance() = node->distance() + longitudinal_resolution_;

      // Add this new node to the queue if it is not beyond the max range.
      if (front_node->distance() < range) {
        // Add the new node to the tables.
        augmentWaypointToNodeTable(front_waypoint->GetId(), front_node);
        augmentRoadlaneToWaypointsTable(front_waypoint);
        // Add the new node to the queue.
        nodes_queue.push(front_node);
      }
    }

    // If this front node is added to the graph, it is set to the
    // front node of the current node.
    if (waypoint_to_node_table_.find(front_node->waypoint()->GetId()) !=
        waypoint_to_node_table_.end()) {
      node->front() = front_node;
      // Keep track of the lattice exit, which is the node with the maximum distance.
      if (front_node->distance() > lattice_exit_->distance())
        lattice_exit_ = front_node;
    }
  }

  return;
}

template<typename Node>
void Lattice<Node>::extendLeft(
    const boost::shared_ptr<Node>& node,
    const double range,
    std::queue<boost::shared_ptr<Node>>& nodes_queue) {
  // Find the left waypoint.
  boost::shared_ptr<CarlaWaypoint> left_waypoint =
    findLeftWaypoint(node->waypoint());

  if (left_waypoint) {
    // Find the left node corresponds to the waypoint.
    boost::shared_ptr<Node> left_node = closestNode(left_waypoint, 0.2);

    if (!left_node) {
      // This left node does not exist yet, add it to the tables and queue.
      left_node = boost::make_shared<Node>(left_waypoint);
      left_node->distance() = node->distance();

      augmentWaypointToNodeTable(left_waypoint->GetId(), left_node);
      augmentRoadlaneToWaypointsTable(left_waypoint);
      nodes_queue.push(left_node);
    }

    // Set the left node of the input node.
    node->left() = left_node;
  }

  return;
}

template<typename Node>
void Lattice<Node>::extendRight(
    const boost::shared_ptr<Node>& node,
    const double range,
    std::queue<boost::shared_ptr<Node>>& nodes_queue) {

  // Find the right waypoint.
  boost::shared_ptr<CarlaWaypoint> right_waypoint =
    findRightWaypoint(node->waypoint());

  if (right_waypoint) {
    // Find the right node corresponds to the waypoint.
    boost::shared_ptr<Node> right_node = closestNode(right_waypoint, 0.2);

    if (!right_node) {
      // This right node does not exist yet, add it to the tables and queue.
      right_node = boost::make_shared<Node>(right_waypoint);
      right_node->distance() = node->distance();

      augmentWaypointToNodeTable(right_waypoint->GetId(), right_node);
      augmentRoadlaneToWaypointsTable(right_waypoint);
      nodes_queue.push(right_node);
    }

    // Set the right node of the input node.
    node->right() = right_node;
  }
  return;
}

template<typename Node>
boost::shared_ptr<const Node> Lattice<Node>::front(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  // Find the node on the lattice that is closest to the given way point.
  // If we cannot find node on the lattice that is close enough,
  // the query waypoint is too far from the lattice, and we return nullptr.
  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Start from the found node, we search forward until the given range is met.
  const double start_distance = node->distance();
  double current_range = 0.0;
  while (current_range < range) {
    node = node->front();
    // There is no futher front node, the given range exceeds the lattice.
    if (!node) return nullptr;
    current_range = node->distance() - start_distance;
  }

  return node;
}

template<typename Node>
boost::shared_ptr<const Node> Lattice<Node>::back(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  // Find the node on the lattice that is closest to the given way point.
  // If we cannot find node on the lattice that is close enough,
  // the query waypoint is too far from the lattice, and we return nullptr.
  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Start from the found node, we search backwards until the given range is met.
  const double start_distance = node->distance();
  double current_range = 0.0;
  while (current_range < range) {
    node = node->back();
    // There is no futher back node, the given range exceeds the lattice.
    if (!node) return nullptr;
    current_range = start_distance - node->distance();
  }

  return node;
}

template<typename Node>
boost::shared_ptr<const Node> Lattice<Node>::leftFront(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the left node of the founded one, and search forward from that.
  boost::shared_ptr<const Node> left_node = node->left();
  if (!left_node) return nullptr;

  return front(left_node->waypoint(), range);
}

template<typename Node>
boost::shared_ptr<const Node> Lattice<Node>::frontLeft(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the front node of the founded one, and return the left of that.
  boost::shared_ptr<const Node> front_node = front(node->waypoint(), range);
  if (!front_node) return nullptr;

  return front_node->left();
}

template<typename Node>
boost::shared_ptr<const Node> Lattice<Node>::leftBack(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the left node of the founded one, and search bacwards from that.
  boost::shared_ptr<const Node> left_node = node->left();
  if (!left_node) return nullptr;

  return back(left_node->waypoint(), range);
}

template<typename Node>
boost::shared_ptr<const Node> Lattice<Node>::backLeft(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the back node of the founded one, and return the left of that.
  boost::shared_ptr<const Node> back_node = back(node->waypoint(), range);
  if (!back_node) return nullptr;

  return back_node->left();
}

template<typename Node>
boost::shared_ptr<const Node> Lattice<Node>::rightFront(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the right node of the founded one, and search forward from that.
  boost::shared_ptr<const Node> right_node = node->right();
  if (!right_node) return nullptr;

  return front(right_node->waypoint(), range);
}

template<typename Node>
boost::shared_ptr<const Node> Lattice<Node>::frontRight(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the front node of the found one, and return the right of that.
  boost::shared_ptr<const Node> front_node = front(node->waypoint(), range);
  if (!front_node) return nullptr;

  return front_node->right();
}

template<typename Node>
boost::shared_ptr<const Node> Lattice<Node>::rightBack(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the right node of the founded one, and search backwards from that.
  boost::shared_ptr<const Node> right_node = node->right();
  if (!right_node) return nullptr;

  return back(right_node->waypoint(), range);
}

template<typename Node>
boost::shared_ptr<const Node> Lattice<Node>::backRight(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the back node of the found one, and return the right of that.
  boost::shared_ptr<const Node> back_node = back(node->waypoint(), range);
  if (!back_node) return nullptr;

  return back_node->right();
}

template<typename Node>
boost::shared_ptr<Node> Lattice<Node>::closestNode(
    const boost::shared_ptr<const CarlaWaypoint>& waypoint,
    const double tolerance) const {

  // Return nullptr is the input waypoint is invalid.
  if (!waypoint) return nullptr;

  // If there is a node in the lattice exactly matches the given waypoint,
  // just return the node.
  if (waypoint_to_node_table_.find(waypoint->GetId()) !=
      waypoint_to_node_table_.end()) {
    return waypoint_to_node_table_.find(waypoint->GetId())->second;
  }

  // Otherwise, we have to do a bit more work.
  // Compare the given waypoint with the waypoints on the same road+lane.
  // Find the closest waypoint node on the same road+lane.
  //const size_t roadlane_id = hashRoadLaneIds(
  //    waypoint->GetRoadId(), waypoint->GetLaneId());
  size_t roadlane_id = 0;
  utils::hashCombine(roadlane_id, waypoint->GetRoadId(), waypoint->GetLaneId());

  if (roadlane_to_waypoints_table_.find(roadlane_id) !=
      roadlane_to_waypoints_table_.end()) {

    // Candidate waypoints on the same road and lane.
    const std::vector<size_t>& candidate_waypoint_ids =
      roadlane_to_waypoints_table_.find(roadlane_id)->second;

    // Find the closest waypoint node.
    double closest_distance = std::numeric_limits<double>::max();
    boost::shared_ptr<Node> closest_node = nullptr;
    for (const size_t id : candidate_waypoint_ids) {
      const boost::shared_ptr<Node> node = waypoint_to_node_table_.find(id)->second;
      const double distance = std::fabs(
          node->waypoint()->GetDistance() - waypoint->GetDistance());

      if (distance < closest_distance) {
        closest_distance = distance;
        closest_node = node;
      }
    }

    // Check if the closest distance is within the tolerance.
    if (closest_distance <= tolerance) return closest_node;
    else return nullptr;
  }

  // If we still cannot find anything, return nullptr.
  return nullptr;
}

template<typename Node>
boost::shared_ptr<typename Lattice<Node>::CarlaWaypoint>
  Lattice<Node>::findFrontWaypoint(
    const boost::shared_ptr<const CarlaWaypoint>& waypoint,
    const double range) const {

  if (range > 5.0) {
    throw std::runtime_error(
        (boost::format("The given range [%1%] is large than 5.0m,"
                       "which may results in an inaccurate front waypoint") % range).str());
  }

  // Direction of the given waypoint.
  //const CarlaVector3D direction = waypoint->GetTransform().GetForwardVector();

  // The front waypoint to be returned.
  boost::shared_ptr<CarlaWaypoint> front_waypoint = nullptr;
  //double best_score = -10.0;

  // Get some candidates using the carla API.
  std::vector<boost::shared_ptr<CarlaWaypoint>> candidates = waypoint->GetNext(range);

  //if (candidates.size() > 1) {
  //  std::printf("current waypoint: road:%d lane:%d junction:%d lane_change:%d\n",
  //      waypoint->GetRoadId(), waypoint->GetLaneId(), waypoint->IsJunction(), waypoint->GetLaneChange());
  //  for (const auto& candidate : candidates)
  //    std::printf("candidate front waypoint: road:%d lane:%d junction:%d lane_change:%d\n",
  //        candidate->GetRoadId(), candidate->GetLaneId(), candidate->IsJunction(), candidate->GetLaneChange());
  //}

  // The basic idea is to loop through all candidate next waypoint.
  // Find the candidate that is at the front (in the sense of lane following)
  // of the current waypoint.
  for (const auto& candidate : candidates) {

    // Ignore the candidate if it is not drivable.
    if (candidate->GetType() != CarlaLane::LaneType::Driving) continue;

    // If we find a match candidate by ID, this is it.
    if (candidate->GetRoadId() == waypoint->GetRoadId() &&
        candidate->GetLaneId() == waypoint->GetLaneId()) {
      front_waypoint = candidate;
      break;
    }

    // FIXME: This might not be correct.
    // Use the lane property to find the next waypoint.
    // Based on the experiments, this can prevent the case of using the waypoint
    // on the off ramp as the next waypoint.
    if (std::abs(candidate->GetLaneId()) == std::abs(waypoint->GetLaneId())) {
      front_waypoint = candidate;
      break;
    }

    // If we cannot find a match based on the IDs,
    // the forward direction of the waypoint is used.
    // The candidate whose forward direction matches the given waypoint
    // is the one we want.
    //const CarlaVector3D candidate_direction = candidate->GetTransform().GetForwardVector();
    //const double score =
    //  direction.x*candidate_direction.x +
    //  direction.y*candidate_direction.y +
    //  direction.z*candidate_direction.z;

    //if (score > best_score) {
    //  best_score = score;
    //  front_waypoint = candidate;
    //}
  }

  return front_waypoint;
}

using WaypointLattice = Lattice<WaypointNode>;
} // End namespace planner.

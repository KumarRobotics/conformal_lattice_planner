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

#include <limits>
#include <utility>
#include <stdexcept>
#include <unordered_set>
#include <algorithm>
#include <boost/format.hpp>

#include <conformal_lattice_planner/waypoint_lattice.h>

namespace planner {

template<typename Node, typename Router>
Lattice<Node, Router>::Lattice(
    const boost::shared_ptr<CarlaWaypoint>& start,
    const double range,
    const double longitudinal_resolution,
    const boost::shared_ptr<Router>& router) :
  router_(router),
  longitudinal_resolution_(longitudinal_resolution) {

  if (range <= longitudinal_resolution_) {
    throw std::runtime_error(
        (boost::format("The given range [%1%] is too small."
                       "Range should be at least 1xlongitudinal_resolution.") % range).str());
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

  return;
}

template<typename Node, typename Router>
Lattice<Node, Router>::Lattice(const Lattice<Node, Router>& other) :
  router_(other.router_),
  roadlane_to_waypoints_table_(other.roadlane_to_waypoints_table_),
  longitudinal_resolution_(other.longitudinal_resolution_) {

  // Copy the \c waypoint_to_node_table_. Make sure this object
  // owns its own copy of the nodes pointed by shared pointers.
  for (const auto& item : other.waypoint_to_node_table_) {
    waypoint_to_node_table_[item.first] = boost::make_shared<Node>(*(item.second));
  }

  // For each node in the table, make sure the connected nodes are pointing
  // to the copy stored within this object.
  for (auto& item : waypoint_to_node_table_) {
    if (item.second->front().lock()) {
      const size_t front_node = item.second->front().lock()->waypoint()->GetId();
      item.second->front() = waypoint_to_node_table_[front_node];
    }
    if (item.second->back().lock()) {
      const size_t back_node = item.second->back().lock()->waypoint()->GetId();
      item.second->back() = waypoint_to_node_table_[back_node];
    }
    if (item.second->left().lock()) {
      const size_t left_node = item.second->left().lock()->waypoint()->GetId();
      item.second->left() = waypoint_to_node_table_[left_node];
    }
    if (item.second->right().lock()) {
      const size_t right_node = item.second->right().lock()->waypoint()->GetId();
      item.second->right() = waypoint_to_node_table_[right_node];
    }
  }

  // Redirect the entry and exit pointers.
  lattice_entry_ = waypoint_to_node_table_[other.lattice_entry_->waypoint()->GetId()];
  lattice_exit_ = waypoint_to_node_table_[other.lattice_exit_->waypoint()->GetId()];

  return;
}

template<typename Node, typename Router>
void Lattice<Node, Router>::swap(Lattice<Node, Router>& other) {
  std::swap(lattice_entry_, other.lattice_entry);
  std::swap(lattice_exit_, other.lattice_exit);
  std::swap(waypoint_to_node_table_, other.waypoint_to_node_table_);
  std::swap(roadlane_to_waypoints_table_, other.roadlane_to_waypoints_table_);
  std::swap(longitudinal_resolution_, other.longitudinal_resolution_);
  std::swap(router_, other.router_);
  return;
}

template<typename Node, typename Router>
std::unordered_map<size_t, boost::shared_ptr<const Node>>
  Lattice<Node, Router>::nodes() const {

  std::unordered_map<size_t, boost::shared_ptr<const Node>> nodes;
  for (const auto& item : waypoint_to_node_table_)
    nodes[item.first] = item.second;

  return nodes;
}

template<typename Node, typename Router>
std::vector<std::pair<size_t, size_t>>
  Lattice<Node, Router>::edges() const {

  std::vector<std::pair<size_t, size_t>> edges;

  for (const auto& item : waypoint_to_node_table_) {
    const boost::shared_ptr<const Node> this_node = item.second;

    if (this_node->front())
      edges.push_back(std::make_pair(
            this_node->waypoint()->GetId(),
            this_node->front()->waypoint()->GetId()));

    if (this_node->left())
      edges.push_back(std::make_pair(
            this_node->waypoint()->GetId(),
            this_node->left()->waypoint()->GetId()));

    if (this_node->right())
      edges.push_back(std::make_pair(
            this_node->waypoint()->GetId(),
            this_node->right()->waypoint()->GetId()));

    if (this_node->back())
      edges.push_back(std::make_pair(
            this_node->waypoint()->GetId(),
            this_node->back()->waypoint()->GetId()));
  }

  return edges;
}

template<typename Node, typename Router>
void Lattice<Node, Router>::extend(const double range) {

  // If the current range of the lattice exceeds the given range,
  // no operation is performed.
  if (lattice_exit_->distance()-lattice_entry_->distance() >= range)
    return;

  // A queue of nodes to be explored.
  // The queue is started from the lattice exit and all nodes that
  // has the same distance with lattice.
  std::queue<boost::shared_ptr<Node>> nodes_queue;
  //nodes_queue.push(lattice_exit_);

  for (auto& item : waypoint_to_node_table_) {
    if (item.second->distance() >= lattice_exit_->distance())
      nodes_queue.push(item.second);
  }

  while (!nodes_queue.empty()) {
    // Get the next node to explore and remove it from the queue.
    boost::shared_ptr<Node> node = nodes_queue.front();
    nodes_queue.pop();

    extendFront(node, range, nodes_queue);
    extendLeft(node, range, nodes_queue);
    extendRight(node, range, nodes_queue);
  }

  //std::printf("Total nodes # on lattice: %lu\n", waypoint_to_node_table_.size());
  return;
}

//template<typename Node, typename Router>
//void Lattice<Node, Router>::shorten(const double range) {
//  // If the current lattice range is already smaller than the given range,
//  // no operation is performed.
//  if (lattice_exit_->distance()-lattice_entry_->distance() <= range)
//    return;
//
//  // The distance before which nodes should be removed.
//  const double safe_distance =
//    lattice_exit_->distance() - lattice_entry_->distance() - range;
//
//  // Save the ids for the waypoint to be removed.
//  std::unordered_set<size_t> removed_waypoint_ids;
//
//  for (const auto& node : waypoint_to_node_table_) {
//    if (node.second->distance() < safe_distance)
//      removed_waypoint_ids.insert(node.first);
//  }
//
//  // Removed the nodes that have been recorded.
//  for (const size_t waypoint_id : removed_waypoint_ids) {
//    reduceRoadlaneToWaypointsTable(waypoint_to_node_table_[waypoint_id]->waypoint());
//    reduceWaypointToNodeTable(waypoint_id);
//  }
//
//  // Set the new lattice entry, which is the one with shortest distance.
//  lattice_entry_ = waypoint_to_node_table_.begin()->second;
//  for (const auto& node : waypoint_to_node_table_) {
//    if (node.second->distance() < lattice_entry_->distance())
//      lattice_entry_ = node.second;
//  }
//
//  // Update the distance of all remaining nodes starting
//  // from the \c lattice_entry_.
//  updateNodeDistance();
//  return;
//}

template<typename Node, typename Router>
void Lattice<Node, Router>::shorten(const double range) {
  // If the current lattice range is already smaller than the given range,
  // no operation is performed.
  if (lattice_exit_->distance()-lattice_entry_->distance() <= range)
    return;

  // The distance before which nodes should be removed.
  const double safe_distance =
    lattice_exit_->distance() - lattice_entry_->distance() - range;

  // Save the ids for the waypoint to be removed.
  std::unordered_set<size_t> removed_waypoint_ids;
  // Save the nodes to be processed.
  std::queue<boost::shared_ptr<Node>> nodes_queue;

  for (auto& item : waypoint_to_node_table_) {
    if (item.second->distance() > lattice_entry_->distance()) continue;
    removed_waypoint_ids.insert(item.second->waypoint()->GetId());
    nodes_queue.push(item.second);
  }

  while (!nodes_queue.empty()) {
    // Get the next node to be processed.
    boost::shared_ptr<Node> node = nodes_queue.front();
    nodes_queue.pop();

    // If this node is the current lattice entry, we have to update
    // the lattice entry to its front node.
    if (node->waypoint()->GetId() == lattice_entry_->waypoint()->GetId() &&
        node->front().lock()) {
      lattice_entry_ = node->front().lock();
    }

    // Add the left node.
    if (node->left().lock()) {
      boost::shared_ptr<Node> left_node = node->left().lock();
      if (removed_waypoint_ids.count(left_node->waypoint()->GetId()) == 0) {
        removed_waypoint_ids.insert(left_node->waypoint()->GetId());
        nodes_queue.push(left_node);
      }
    }

    // Add the right node.
    if (node->right().lock()) {
      boost::shared_ptr<Node> right_node = node->right().lock();
      if (removed_waypoint_ids.count(right_node->waypoint()->GetId()) == 0) {
        removed_waypoint_ids.insert(right_node->waypoint()->GetId());
        nodes_queue.push(right_node);
      }
    }

    // Add the back node.
    if (node->back().lock()) {
      boost::shared_ptr<Node> back_node = node->back().lock();
      if (removed_waypoint_ids.count(back_node->waypoint()->GetId()) == 0) {
        removed_waypoint_ids.insert(back_node->waypoint()->GetId());
        nodes_queue.push(back_node);
      }
    }

    // Some special care is required for the front node.
    // We need to determine when to stop.
    if (node->front().lock()) {
      boost::shared_ptr<Node> front_node = node->front().lock();
      if (front_node->distance() < safe_distance &&
          removed_waypoint_ids.count(front_node->waypoint()->GetId()) == 0) {
        removed_waypoint_ids.insert(front_node->waypoint()->GetId());
        nodes_queue.push(front_node);
      }
    }
  }

  // Removed the nodes that have been recorded.
  for (const size_t waypoint_id : removed_waypoint_ids) {
    reduceRoadlaneToWaypointsTable(waypoint_to_node_table_[waypoint_id]->waypoint());
    reduceWaypointToNodeTable(waypoint_id);
  }

  // Update the distance of all remaining nodes starting
  // from the \c lattice_entry_.
  updateNodeDistance();

  //std::printf("Total nodes # on lattice: %lu\n", waypoint_to_node_table_.size());
  return;
}

template<typename Node, typename Router>
void Lattice<Node, Router>::updateNodeDistance() {

  std::unordered_set<size_t> updated_waypoint_ids;
  std::queue<boost::shared_ptr<Node>> nodes_queue;

  //lattice_entry_->distance() = 0.0;
  //nodes_queue.push(lattice_entry_);
  //updated_waypoint_ids.insert(lattice_entry_->waypoint()->GetId());
  const double lattice_entry_distance = lattice_entry_->distance();

  for (auto& item : waypoint_to_node_table_) {
    if (item.second->distance() > lattice_entry_distance) continue;
    item.second->distance() = 0.0;
    nodes_queue.push(item.second);
    updated_waypoint_ids.insert(item.second->waypoint()->GetId());
  }

  while (!nodes_queue.empty()) {
    // Get the next node to be processed.
    boost::shared_ptr<Node> node = nodes_queue.front();
    nodes_queue.pop();

    // Update the left node.
    if (node->left().lock()) {
      boost::shared_ptr<Node> left_node = node->left().lock();
      if (updated_waypoint_ids.count(left_node->waypoint()->GetId()) == 0) {
        updated_waypoint_ids.insert(left_node->waypoint()->GetId());
        nodes_queue.push(left_node);
        left_node->distance() = node->distance();
      }
    }

    // Update the right node.
    if (node->right().lock()) {
      boost::shared_ptr<Node> right_node = node->right().lock();
      if (updated_waypoint_ids.count(right_node->waypoint()->GetId()) == 0) {
        updated_waypoint_ids.insert(right_node->waypoint()->GetId());
        nodes_queue.push(right_node);
        right_node->distance() = node->distance();
      }
    }

    // Update the front node.
    if (node->front().lock()) {
      boost::shared_ptr<Node> front_node = node->front().lock();
      if (updated_waypoint_ids.count(front_node->waypoint()->GetId()) == 0) {
        updated_waypoint_ids.insert(front_node->waypoint()->GetId());
        nodes_queue.push(front_node);
        front_node->distance() = node->distance() + longitudinal_resolution_;
      }
    }
  }

  return;
}

template<typename Node, typename Router>
void Lattice<Node, Router>::extendFront(
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
      if (front_node->distance() <= range) {
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
      front_node->back() = node;
      // Keep track of the lattice exit, which is the node with the maximum distance.
      if (front_node->distance() > lattice_exit_->distance())
        lattice_exit_ = front_node;
    }
  }

  return;
}

template<typename Node, typename Router>
void Lattice<Node, Router>::extendLeft(
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

template<typename Node, typename Router>
void Lattice<Node, Router>::extendRight(
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

template<typename Node, typename Router>
boost::shared_ptr<const Node> Lattice<Node, Router>::front(
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

template<typename Node, typename Router>
boost::shared_ptr<const Node> Lattice<Node, Router>::back(
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

template<typename Node, typename Router>
boost::shared_ptr<const Node> Lattice<Node, Router>::leftFront(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the left node of the founded one, and search forward from that.
  boost::shared_ptr<const Node> left_node = node->left();
  if (!left_node) return nullptr;

  return front(left_node->waypoint(), range);
}

template<typename Node, typename Router>
boost::shared_ptr<const Node> Lattice<Node, Router>::frontLeft(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the front node of the founded one, and return the left of that.
  boost::shared_ptr<const Node> front_node = front(node->waypoint(), range);
  if (!front_node) return nullptr;

  return front_node->left();
}

template<typename Node, typename Router>
boost::shared_ptr<const Node> Lattice<Node, Router>::leftBack(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the left node of the founded one, and search bacwards from that.
  boost::shared_ptr<const Node> left_node = node->left();
  if (!left_node) return nullptr;

  return back(left_node->waypoint(), range);
}

template<typename Node, typename Router>
boost::shared_ptr<const Node> Lattice<Node, Router>::backLeft(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the back node of the founded one, and return the left of that.
  boost::shared_ptr<const Node> back_node = back(node->waypoint(), range);
  if (!back_node) return nullptr;

  return back_node->left();
}

template<typename Node, typename Router>
boost::shared_ptr<const Node> Lattice<Node, Router>::rightFront(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the right node of the founded one, and search forward from that.
  boost::shared_ptr<const Node> right_node = node->right();
  if (!right_node) return nullptr;

  return front(right_node->waypoint(), range);
}

template<typename Node, typename Router>
boost::shared_ptr<const Node> Lattice<Node, Router>::frontRight(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the front node of the found one, and return the right of that.
  boost::shared_ptr<const Node> front_node = front(node->waypoint(), range);
  if (!front_node) return nullptr;

  return front_node->right();
}

template<typename Node, typename Router>
boost::shared_ptr<const Node> Lattice<Node, Router>::rightBack(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the right node of the founded one, and search backwards from that.
  boost::shared_ptr<const Node> right_node = node->right();
  if (!right_node) return nullptr;

  return back(right_node->waypoint(), range);
}

template<typename Node, typename Router>
boost::shared_ptr<const Node> Lattice<Node, Router>::backRight(
    const boost::shared_ptr<const CarlaWaypoint>& query,
    const double range) const {

  boost::shared_ptr<const Node> node = closestNode(query, longitudinal_resolution_);
  if (!node) return nullptr;

  // Get the back node of the found one, and return the right of that.
  boost::shared_ptr<const Node> back_node = back(node->waypoint(), range);
  if (!back_node) return nullptr;

  return back_node->right();
}

template<typename Node, typename Router>
boost::shared_ptr<Node> Lattice<Node, Router>::closestNode(
    const boost::shared_ptr<const CarlaWaypoint>& waypoint,
    const double tolerance) {

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

} // End namespace planner.

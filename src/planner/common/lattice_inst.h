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
#include <string>
#include <boost/format.hpp>

#include <planner/common/lattice.h>

namespace planner {

template<typename Node>
Lattice<Node>::Lattice(
  const boost::shared_ptr<const CarlaWaypoint>& start,
  const double range,
  const double longitudinal_resolution,
  const boost::shared_ptr<router::Router>& router) :
    router_(router),
    longitudinal_resolution_(longitudinal_resolution) {

  if (range <= longitudinal_resolution_) {
    std::string error_msg = (boost::format(
            "Lattice::Lattice(): "
            "range [%1%] < longitudinal resolution [%2%].\n")
          % range
          % longitudinal_resolution).str();
    throw std::runtime_error(error_msg);
  }

  // Create the start node.
  boost::shared_ptr<Node> start_node = boost::make_shared<Node>(start);
  start_node->distance() = 0.0;
  lattice_exits_.push_back(start_node);

  augmentWaypointToNodeTable(start->GetId(), start_node);
  augmentRoadlaneToWaypointsTable(start);

  // Construct the lattice.
  extend(range);

  return;
}

template<typename Node>
Lattice<Node>::Lattice(const Lattice<Node>& other) :
  router_(other.router_),
  lattice_entries_(other.lattice_entries_),
  lattice_exits_(other.lattice_exits_),
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
  for (auto& entry : lattice_entries_)
    entry = waypoint_to_node_table_[entry.lock()->waypoint()->GetId()];
  for (auto& exit : lattice_exits_)
    exit = waypoint_to_node_table_[exit.lock()->waypoint()->GetId()];

  return;
}

template<typename Node>
void Lattice<Node>::swap(Lattice<Node>& other) {

  std::swap(lattice_entries_, other.lattice_entries_);
  std::swap(lattice_exits_, other.lattice_exits_);
  std::swap(waypoint_to_node_table_, other.waypoint_to_node_table_);
  std::swap(roadlane_to_waypoints_table_, other.roadlane_to_waypoints_table_);
  std::swap(longitudinal_resolution_, other.longitudinal_resolution_);
  std::swap(router_, other.router_);

  return;
}

template<typename Node>
void Lattice<Node>::augmentRoadlaneToWaypointsTable(
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

template<typename Node>
void Lattice<Node>::reduceRoadlaneToWaypointsTable(
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

template<typename Node>
std::unordered_map<size_t, boost::shared_ptr<const Node>>
  Lattice<Node>::nodes() const {

  std::unordered_map<size_t, boost::shared_ptr<const Node>> nodes;
  for (const auto& item : waypoint_to_node_table_)
    nodes[item.first] = item.second;

  return nodes;
}

template<typename Node>
std::vector<std::pair<size_t, size_t>>
  Lattice<Node>::edges() const {

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

template<typename Node>
double Lattice<Node>::range() const {

  if (lattice_entries_.empty() || lattice_exits_.empty()) return 0.0;

  double entry_distance = std::numeric_limits<double>::max();
  double exit_distance = 0.0;

  for (const auto& entry : lattice_entries_) {
    if (entry.lock()->distance() < entry_distance)
      entry_distance = entry.lock()->distance();
  }
  for (const auto& exit : lattice_exits_) {
    if (exit.lock()->distance() > exit_distance)
      exit_distance = exit.lock()->distance();
  }

  return exit_distance - entry_distance;
}

template<typename Node>
void Lattice<Node>::extend(double range) {

  if (range <= 0.0) {
    std::string error_msg = (boost::format(
            "Lattice::Lattice(): "
            "range [%1%] <= 0.0.\n")
          % range).str();
    throw std::runtime_error(error_msg);
  }

  // If the current range of the lattice exceeds the given range,
  // no operation is performed.
  range = std::ceil(range);
  if (this->range() >= range) return;

  // A queue of nodes to be explored.
  // The queue is started from the lattice exits.
  std::queue<boost::shared_ptr<Node>> nodes_queue;
  for (auto& exit : lattice_exits_) nodes_queue.push(exit.lock());

  while (!nodes_queue.empty()) {
    // Get the next node to explore and remove it from the queue.
    boost::shared_ptr<Node> node = nodes_queue.front();
    nodes_queue.pop();

    extendFront(node, range, nodes_queue);
    extendLeft(node, nodes_queue);
    extendRight(node, nodes_queue);
  }

  // Update lattice entries and exits.
  findLatticeEntriesAndExits();

  return;
}

template<typename Node>
void Lattice<Node>::shorten(double range) {

  if (range < 0.0) {
    std::string error_msg((boost::format(
            "Lattice::shorten(): "
            "required range [%1%] < 0.0.\n") % range).str());
    throw std::runtime_error(error_msg);
  }

  // If the current lattice range is already smaller than the given range,
  // no operation is performed.
  range = std::ceil(range);
  if (this->range() <= range) return;

  // The distance before which nodes should be removed.
  const double safe_distance = this->range() - range;

  // Save the ids for the waypoint to be removed.
  std::unordered_set<size_t> removed_waypoint_ids;
  // Save the nodes to be processed.
  std::queue<boost::shared_ptr<Node>> nodes_queue;

  for (auto& entry : lattice_entries_) {
    if (entry.lock()->distance() >= safe_distance) continue;
    removed_waypoint_ids.insert(entry.lock()->waypoint()->GetId());
    nodes_queue.push(entry.lock());
  }

  while (!nodes_queue.empty()) {
    // Get the next node to be processed.
    boost::shared_ptr<Node> node = nodes_queue.front();
    nodes_queue.pop();

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

  // Update the entries and exits of the lattic.
  findLatticeEntriesAndExits();

  // Update the distance of all remaining nodes.
  updateNodeDistance();

  return;
}

template<typename Node>
void Lattice<Node>::updateNodeDistance() {

  // Find the existing lattice entry with the smallest distance.
  // The distance of all nodes will be reduced by this much.
  double shift_distance = lattice_entries_[0].lock()->distance();
  for (auto& entry : lattice_entries_) {
    if (entry.lock()->distance() >= shift_distance) continue;
    shift_distance = entry.lock()->distance();
  }

  // Put all existing entries into the queue.
  std::unordered_set<size_t> updated_waypoint_ids;
  std::queue<boost::shared_ptr<Node>> nodes_queue;

  for (auto& entry : lattice_entries_) {
    entry.lock()->distance() -= shift_distance;
    nodes_queue.push(entry.lock());
    updated_waypoint_ids.insert(entry.lock()->waypoint()->GetId());
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

    // Update the back node.
    if (node->back().lock()) {
      boost::shared_ptr<Node> back_node = node->back().lock();
      if (updated_waypoint_ids.count(back_node->waypoint()->GetId()) == 0) {
        updated_waypoint_ids.insert(back_node->waypoint()->GetId());
        nodes_queue.push(back_node);
        back_node->distance() = node->distance() - longitudinal_resolution_;
      }
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
    }
  }

  return;
}

template<typename Node>
void Lattice<Node>::extendLeft(
    const boost::shared_ptr<Node>& node,
    std::queue<boost::shared_ptr<Node>>& nodes_queue) {
  // Find the left waypoint.
  boost::shared_ptr<CarlaWaypoint> left_waypoint =
    findLeftWaypoint(node->waypoint());

  // Return if there is no left waypoint.
  if (!left_waypoint) return;

  // Return if the left waypoint is not drivable.
  if(left_waypoint->GetType() != carla::road::Lane::LaneType::Driving) return;

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

  // The left node is set to the left of this node
  // if one can do a left lane change here.
  if ((node->waypoint()->GetLaneChange() ==
       carla::road::element::LaneMarking::LaneChange::Left) ||
      (node->waypoint()->GetLaneChange() ==
       carla::road::element::LaneMarking::LaneChange::Both)) {
    node->left() = left_node;
  } else {
    node->left().reset();
  }

  return;
}

template<typename Node>
void Lattice<Node>::extendRight(
    const boost::shared_ptr<Node>& node,
    std::queue<boost::shared_ptr<Node>>& nodes_queue) {

  // Find the right waypoint.
  boost::shared_ptr<CarlaWaypoint> right_waypoint =
    findRightWaypoint(node->waypoint());

  // Return if there is no right waypoint.
  if (!right_waypoint) return;

  // Return if the right waypoint is not drivable.
  if(right_waypoint->GetType() != carla::road::Lane::LaneType::Driving) return;

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

  // The right node is set to the right of this node
  // if one can do a right lane change here.
  if ((node->waypoint()->GetLaneChange() ==
       carla::road::element::LaneMarking::LaneChange::Right) ||
      (node->waypoint()->GetLaneChange() ==
       carla::road::element::LaneMarking::LaneChange::Both)) {
    node->right() = right_node;
  } else {
    node->right().reset();
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
void Lattice<Node>::findLatticeEntriesAndExits() {

  lattice_entries_.clear();
  lattice_exits_.clear();

  for (auto& item : waypoint_to_node_table_) {
    if (!(item.second->back().lock())) lattice_entries_.push_back(item.second);
    if (!(item.second->front().lock())) lattice_exits_.push_back(item.second);
  }

  return;
}

template<typename Node>
boost::shared_ptr<Node> Lattice<Node>::closestNode(
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
    //else return nullptr;
  }

  // Now, we really have to pull out the big gun, searching through all
  // nodes on the lattice in order to find the closest node.
  double closest_distance = std::numeric_limits<double>::max();
  boost::shared_ptr<Node> closest_node = nullptr;

  for (const auto& item : waypoint_to_node_table_) {

    const double distance = (
        item.second->waypoint()->GetTransform().location -
        waypoint->GetTransform().location).Length();

    if (distance < closest_distance) {
      closest_distance = distance;
      closest_node = item.second;
    }
  }

  //std::printf("closest distance:%f tolerance:%f\n", closest_distance, tolerance);

  if (closest_distance < tolerance) return closest_node;
  else return nullptr;
  //return nullptr;
}

template<typename Node>
std::string Lattice<Node>::string(const std::string& prefix) const {

  std::string lattice_msg = (boost::format(
        "lattice longitudinal resolution: %1%.\n"
        "lattice node #: %2%\n")
      % longitudinal_resolution_
      % waypoint_to_node_table_.size()).str();

  std::string lattice_entries_msg = (boost::format(
        "%1% lattice entries:\n") % lattice_entries_.size()).str();
  for (const auto& entry : lattice_entries_)
    lattice_entries_msg += entry.lock()->string();

  std::string lattice_exits_msg = (boost::format(
        "%1% lattice exits:\n") % lattice_exits_.size()).str();
  for (const auto& exit : lattice_exits_)
    lattice_exits_msg += exit.lock()->string();

  return prefix + lattice_msg + lattice_entries_msg + lattice_exits_msg;
}

} // End namespace planner.

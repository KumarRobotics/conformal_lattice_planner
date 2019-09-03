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
#include <stdexcept>
#include <unordered_set>
#include <algorithm>
#include <boost/format.hpp>

#include <conformal_lattice_planner/waypoint_lattice.h>

namespace planner {

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

  return;
}

template<typename Node>
void Lattice<Node>::extend(const double range) {

  // If the current range of the lattice exceeds the given range,
  // no operation is performed.
  if (lattice_exit_->distance()-lattice_entry_->distance() >= range)
    return;

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

  //std::printf("Total nodes # on lattice: %lu\n", waypoint_to_node_table_.size());
  return;
}

template<typename Node>
void Lattice<Node>::shorten(const double range) {
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

  removed_waypoint_ids.insert(lattice_entry_->waypoint()->GetId());
  nodes_queue.push(lattice_entry_);

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

template<typename Node>
void Lattice<Node>::updateNodeDistance() {

  std::unordered_set<size_t> updated_waypoint_ids;
  std::queue<boost::shared_ptr<Node>> nodes_queue;

  lattice_entry_->distance() = 0.0;
  nodes_queue.push(lattice_entry_);
  updated_waypoint_ids.insert(lattice_entry_->waypoint()->GetId());

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

} // End namespace planner.

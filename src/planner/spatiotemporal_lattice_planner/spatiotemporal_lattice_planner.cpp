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

#include <list>
#include <planner/spatiotemporal_lattice_planner/spatiotemporal_lattice_planner.h>

namespace planner {
namespace spatiotemporal_lattice_planner {

constexpr std::array<std::pair<double, double>, 3> Vertex::kVelocityIntervalsPerStation_;
constexpr std::array<double, 7> SpatiotemporalLatticePlanner::kAccelerationOptions_;

void Vertex::updateOptimalParent() {
  // Set the \c optimal_parent_ to an existing parent vertex.
  // It does not matter which parent is used for now.
  if (!optimal_parent_) {
    for (const auto& parent : left_parents_) {
      if (!parent) continue;
      optimal_parent_ = parent;
      break;
    }
  }

  if (!optimal_parent_) {
    for (const auto& parent : back_parents_) {
      if (!parent) continue;
      optimal_parent_ = parent;
      break;
    }
  }

  if (!optimal_parent_) {
    for (const auto& parent : right_parents_) {
      if (!parent) continue;
      optimal_parent_ = parent;
      break;
    }
  }

  if (!optimal_parent_) {
    throw std::runtime_error(
        "Vertex::updateOptimalParent(): "
        "cannot update optimal parent since there is no parent available.\n");
  }

  // Set the \c optimal_parent_ to the existing parent with the minimum cost-to-come.
  // With the same cost-to-come, the back parent is preferred.
  for (const auto& parent : left_parents_) {
    if (!parent) continue;
    if (std::get<1>(*parent) <= std::get<1>(*optimal_parent_))
      optimal_parent_ = parent;
  }

  for (const auto& parent : right_parents_) {
    if (!parent) continue;
    if (std::get<1>(*parent) <= std::get<1>(*optimal_parent_))
      optimal_parent_ = parent;
  }

  for (const auto& parent : back_parents_) {
    if (!parent) continue;
    if (std::get<1>(*parent) <= std::get<1>(*optimal_parent_))
      optimal_parent_ = parent;
  }

  return;
}

void Vertex::updateLeftParent(
    const Snapshot& snapshot,
    const double cost_to_come,
    const boost::shared_ptr<Vertex>& parent_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(parent_vertex->speed());
  if (!idx) return;

  left_parents_[*idx] = std::make_tuple(snapshot, cost_to_come, parent_vertex);
  updateOptimalParent();
  return;
}

void Vertex::updateBackParent(
    const Snapshot& snapshot,
    const double cost_to_come,
    const boost::shared_ptr<Vertex>& parent_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(parent_vertex->speed());
  if (!idx) return;

  back_parents_[*idx] = std::make_tuple(snapshot, cost_to_come, parent_vertex);
  updateOptimalParent();
  return;
}

void Vertex::updateRightParent(
    const Snapshot& snapshot,
    const double cost_to_come,
    const boost::shared_ptr<Vertex>& parent_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(parent_vertex->speed());
  if (!idx) return;

  right_parents_[*idx] = std::make_tuple(snapshot, cost_to_come, parent_vertex);
  updateOptimalParent();
  return;
}

void Vertex::updateLeftChild(
    const ContinuousPath& path,
    const double acceleration,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(child_vertex->speed());
  if (!idx) return;

  if (!(left_children_[*idx]) ||
      std::get<2>(*(left_children_[*idx])) > stage_cost)
    left_children_[*idx] = std::make_tuple(path, acceleration, stage_cost, child_vertex);

  return;
}

void Vertex::updateFrontChild(
    const ContinuousPath& path,
    const double acceleration,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(child_vertex->speed());
  if (!idx) return;

  if (!(front_children_[*idx]) ||
      std::get<2>(*(front_children_[*idx])) > stage_cost)
    front_children_[*idx] = std::make_tuple(path, acceleration, stage_cost, child_vertex);
  return;
}

void Vertex::updateRightChild(
    const ContinuousPath& path,
    const double acceleration,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  // Figure out which speed interval this vertex belongs to.
  boost::optional<size_t> idx = speedIntervalIdx(child_vertex->speed());
  if (!idx) return;

  if (!(right_children_[*idx]) ||
      std::get<2>(*(right_children_[*idx])) > stage_cost)
    right_children_[*idx] = std::make_tuple(path, acceleration, stage_cost, child_vertex);
  return;
}

std::string Vertex::string(const std::string& prefix) const {
  std::string output = prefix;
  output += "node id: " + std::to_string(node_.lock()->id()) + "\n";
  output += "snapshot: \n" + snapshot_.string();

  // Output the parents.
  boost::format parent_format("node id:%1% ego speed:%2% cost to come:%3%\n");

  output += std::string("left parents #: ") + std::to_string(leftParentsSize()) + "\n";
  for (const auto& parent : left_parents_) {
    if (!parent) continue;
    output += (parent_format % std::get<2>(*parent).lock()->node().lock()->id()
                             % std::get<2>(*parent).lock()->speed()
                             % std::get<1>(*parent)).str();
  }

  output += std::string("back parents #: ") + std::to_string(backParentsSize()) + "\n";
  for (const auto& parent : back_parents_) {
    if (!parent) continue;
    output += (parent_format % std::get<2>(*parent).lock()->node().lock()->id()
                             % std::get<2>(*parent).lock()->speed()
                             % std::get<1>(*parent)).str();
  }

  output += std::string("right parents #: ") + std::to_string(rightParentsSize()) + "\n";
  for (const auto& parent : right_parents_) {
    if (!parent) continue;
    output += (parent_format % std::get<2>(*parent).lock()->node().lock()->id()
                             % std::get<2>(*parent).lock()->speed()
                             % std::get<1>(*parent)).str();
  }

  output += "optimal parent: ";
  if (optimal_parent_)
    output += (parent_format % std::get<2>(*optimal_parent_).lock()->node().lock()->id()
                             % std::get<2>(*optimal_parent_).lock()->speed()
                             % std::get<1>(*optimal_parent_)).str();
  else output += "\n";

  // Output the children.
  boost::format child_format(
      "node id:%1% ego speed:%2% acceleration:%3% path length:%4% stage cost:%5%\n");

  output += std::string("left children #: ") + std::to_string(leftChildrenSize()) + "\n";
  for (const auto& child : left_children_) {
    if (!child) continue;
    output = (child_format % std::get<3>(*child).lock()->node().lock()->id()
                           % std::get<3>(*child).lock()->speed()
                           % std::get<1>(*child)
                           % std::get<0>(*child).range()
                           % std::get<2>(*child)).str();
  }

  output += std::string("front children #: ") + std::to_string(frontChildrenSize()) + "\n";
  for (const auto& child : front_children_) {
    if (!child) continue;
    output = (child_format % std::get<3>(*child).lock()->node().lock()->id()
                           % std::get<3>(*child).lock()->speed()
                           % std::get<1>(*child)
                           % std::get<0>(*child).range()
                           % std::get<2>(*child)).str();
  }

  output += std::string("right children #: ") + std::to_string(rightChildrenSize()) + "\n";
  for (const auto& child : right_children_) {
    if (!child) continue;
    output = (child_format % std::get<3>(*child).lock()->node().lock()->id()
                           % std::get<3>(*child).lock()->speed()
                           % std::get<1>(*child)
                           % std::get<0>(*child).range()
                           % std::get<2>(*child)).str();
  }

  return output;
}

bool SpatiotemporalLatticePlanner::immediateNextVertexReached(
    const Snapshot& snapshot) const {

  //std::printf("immediateNextStationReached(): \n");

  boost::shared_ptr<const WaypointLattice<router::LoopRouter>> waypoint_lattice =
    boost::const_pointer_cast<const WaypointLattice<router::LoopRouter>>(waypoint_lattice_);

  // Find out the current distance of the ego on the lattice.
  boost::shared_ptr<const WaypointNode> ego_node = waypoint_lattice->closestNode(
      fast_map_->waypoint(snapshot.ego().transform().location),
      waypoint_lattice->longitudinalResolution());
  double ego_distance = ego_node->distance();

  // Find out the distance the ego need to achieve.
  double target_distance = 0.0;
  if (root_.lock()->hasLeftChildren()) {
    boost::shared_ptr<Vertex> vertex = std::get<3>(root_.lock()->validLeftChildren().front()).lock();
    target_distance = vertex->node().lock()->distance();
  } else if (root_.lock()->hasFrontChildren()) {
    boost::shared_ptr<Vertex> vertex = std::get<3>(root_.lock()->validFrontChildren().front()).lock();
    target_distance = vertex->node().lock()->distance();
  } else if (root_.lock()->hasRightChildren()) {
    boost::shared_ptr<Vertex> vertex = std::get<3>(root_.lock()->validRightChildren().front()).lock();
    target_distance = vertex->node().lock()->distance();
  }

  // If the difference is less than 0.5, or the ego has travelled beyond the
  // the target distance, the immediate station is considered to be reached.
  if (target_distance-ego_distance < 0.5) return true;
  else return false;
}

void SpatiotemporalLatticePlanner::updateWaypointLattice(const Snapshot& snapshot) {
  //std::printf("updateWaypointLattice(): \n");

  // If the waypoint lattice has not been initialized, a new one is created with
  // the start waypoint as where the ego currently is. Meanwhile, the range of
  // the lattice is set to the spatial horizon. The resolution is hardcoded as 1.0m.
  if (!waypoint_lattice_) {
    //std::printf("Create new waypoint lattice.\n");
    boost::shared_ptr<CarlaWaypoint> ego_waypoint =
      fast_map_->waypoint(snapshot.ego().transform().location);
    waypoint_lattice_ = boost::make_shared<WaypointLattice<router::LoopRouter>>(
        ego_waypoint, spatial_horizon_+30.0, 1.0, router_);
    return;
  }

  // If the waypoint lattice has been created before, we will choose to either
  // update the lattice or leave it as it currently is based on whether the ego
  // has reached one of the child stations of the root.
  if (immediateNextVertexReached(snapshot)) {
    boost::shared_ptr<const WaypointLattice<router::LoopRouter>> waypoint_lattice =
      boost::const_pointer_cast<const WaypointLattice<router::LoopRouter>>(waypoint_lattice_);

    boost::shared_ptr<const WaypointNode> ego_node = waypoint_lattice->closestNode(
        fast_map_->waypoint(snapshot.ego().transform().location),
        waypoint_lattice->longitudinalResolution());
    //std::printf("Shift the lattice by %f\n", ego_node->distance()-5.0);
    const double shift_distance = ego_node->distance() - 5.0;
    waypoint_lattice_->shift(shift_distance);
  }

  return;
}

std::vector<boost::shared_ptr<Vertex>>
  SpatiotemporalLatticePlanner::connectVertexToFrontNode(
      const boost::shared_ptr<Vertex>& vertex,
      const boost::shared_ptr<const WaypointNode>& target_node) {

  std::printf("SpatiotemporalLatticePlanner::connectVertexToFrontNode()\n");

  // Return directly if the target node does not exist.
  if (!target_node) return std::vector<boost::shared_ptr<Vertex>>();

  // Plan a path between the node at the current vertex to the target node.
  boost::shared_ptr<ContinuousPath> path = nullptr;
  try {
    path = boost::make_shared<ContinuousPath>(
        std::make_pair(vertex->snapshot().ego().transform(),
                       vertex->snapshot().ego().curvature()),
        std::make_pair(target_node->waypoint()->GetTransform(),
                       target_node->curvature(map_)),
        ContinuousPath::LaneChangeType::KeepLane);
  } catch (std::exception& e) {
    // If for whatever reason, the path cannot be created, the front vertices
    // cannot be created either.
    std::printf("%s", e.what());
    return std::vector<boost::shared_ptr<Vertex>>();
  }

  // Simulate the traffic forward with the ego applying different constant
  // accelerations over the path created above.
  for (const double accel : kAccelerationOptions_) {
    // Prepare the start snapshot.
    // The acceleration of the ego is set accordingly.
    Snapshot snapshot = vertex->snapshot();
    snapshot.ego().acceleration() = accel;

    TrafficSimulator simulator(snapshot, map_, fast_map_);
    double simulation_time = 0.0; double stage_cost = 0.0;
    const bool no_collision = simulator.simulate(
        *path, sim_time_step_, 5.0, simulation_time, stage_cost);

    // Continue if this acceleration option leads to collision.
    if (!no_collision) continue;

    // Create a new vertex using the end snapshot of the simulation.
    boost::shared_ptr<Vertex> next_vertex = boost::make_shared<Vertex>(
        simulator.snapshot(), waypoint_lattice_, fast_map_);

    // Check if a similar vertex (close in ego velocity) has already been created.
    // If so, the \c next_vertex is replaced with the existing one in the table.
    boost::shared_ptr<Vertex> similar_vertex = findVertexInTable(next_vertex);
    if (similar_vertex) next_vertex = similar_vertex;

    // Update the child of the parent vertex.
    vertex->updateFrontChild(*path, accel, stage_cost, next_vertex);

    // Update the parent vertex of the child.
    if (vertex->hasParents()) {
      next_vertex->updateBackParent(
          simulator.snapshot(), vertex->costToCome()+stage_cost, vertex);
    } else {
      next_vertex->updateBackParent(
          simulator.snapshot(), stage_cost, vertex);
    }
  } // End for loop for different acceleration options.

  // Collect all the front child vertices of the input vertex.
  auto front_children = vertex->validFrontChildren();
  std::vector<boost::shared_ptr<Vertex>> front_vertices;
  for (const auto& child : front_children) {
    front_vertices.push_back(std::get<3>(child).lock());
  }

  return front_vertices;
}

boost::shared_ptr<Vertex> SpatiotemporalLatticePlanner::findVertexInTable(
    const boost::shared_ptr<Vertex>& vertex) {
  boost::optional<size_t> idx = Vertex::speedIntervalIdx(vertex->speed());
  if (!idx) {
    std::string error_msg(
        "SpatiotemporalLattice::findVertexInTable(): "
        "invalid ego speed in input vertex.\n");
    error_msg + vertex->string();
    throw std::runtime_error(error_msg);
  }

  auto iter = node_to_vertices_table_.find(vertex->node().lock()->id());
  if (iter == node_to_vertices_table_.end()) return nullptr;
  return (iter->second)[*idx];
}

} // End namespace spatiotemporal_lattice_planner.
} // End namespace planner.


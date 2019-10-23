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

void Vertex::updateOptimalParent() {
  // Set the \c optimal_parent_ to an existing parent vertex.
  // It does not matter which parent is used for now.
  if (!optimal_parent_) {
    for (const auto& parent : left_parents_) {
      if (!parent) continue;
      optimal_parent_ = parent_;
      break;
    }
  }

  if (!optimal_parent_) {
    for (const auto& parent : back_parents_) {
      if (!parent) continue;
      optimal_parent_ = parent_;
      break;
    }
  }

  if (!optimal_parent_) {
    for (const auto& parent : right_parents_) {
      if (!parent) continue;
      optimal_parent_ = parent_;
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

boost::optional<size_t> Vertex::speedIntervalIdx(const double speed) {

  // Return \c boost::none if the input speed is less than 0.
  if (speed < 0.0) return boost::none;

  for (size_t i = 0; i < kVelocityIntervalsPerStation_.size(); ++i) {
    if (speed < kVelocityIntervalsPerStation_[i].second) return i;
  }

  // Return \c boost::none if the speed is too large.
  return boost::none;
}

void Vertex:updateLeftParent(
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

  right_children_[*idx] = std::make_tuple(path, acceleration, stage_cost, child_vertex);
  return;
}

std::string Vertex::string(const std::string& prefix) const {
  return prefix;
}

} // End namespace spatiotemporal_lattice_planner.
} // End namespace planner.


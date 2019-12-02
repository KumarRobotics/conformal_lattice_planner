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
#include <planner/slc_lattice_planner/slc_lattice_planner.h>

namespace planner {
namespace slc_lattice_planner {

const double SLCTrafficSimulator::egoAcceleration() const {

  // The logic for computing the acceleration for the ego vehicle is simple.
  // Regardless whether the ego vehicle is in the process of lane changing
  // or not, we will only consider the front vehicle (if any) on the same
  // lane with the head of the ego vehicle.
  //
  // TODO: Should we consider the front vehicle on two lanes, both the
  //       old lane and the target lane?

  double accel = 0.0;
  boost::optional<std::pair<size_t, double>> lead =
    snapshot_.trafficLattice()->front(snapshot_.ego().id());

  if (lead) {
    const double lead_speed = snapshot_.vehicle(lead->first).speed();
    const double following_distance = lead->second;
    accel = idm_->idm(snapshot_.ego().speed(),
                      snapshot_.ego().policySpeed(),
                      lead_speed,
                      following_distance);
  } else {
    accel = idm_->idm(snapshot_.ego().speed(),
                      snapshot_.ego().policySpeed());
  }

  return accel;
}

const double SLCTrafficSimulator::agentAcceleration(const size_t agent) const {

  // We assume all agent vehicles are lane followers for now.
  double accel = 0.0;
  boost::optional<std::pair<size_t, double>> lead =
    snapshot_.trafficLattice()->front(agent);

  if (lead) {
    const double lead_speed = snapshot_.vehicle(lead->first).speed();
    const double following_distance = lead->second;
    accel = idm_->idm(snapshot_.vehicle(agent).speed(),
                      snapshot_.vehicle(agent).policySpeed(),
                      lead_speed,
                      following_distance);
  } else {
    accel = idm_->idm(snapshot_.vehicle(agent).speed(),
                      snapshot_.vehicle(agent).policySpeed());
  }

  return accel;
}

void Vertex::updateParent(
    const Snapshot& snapshot,
    const double cost_to_come,
    const boost::shared_ptr<Vertex>& parent_vertex) {
  parent_ = std::make_tuple(snapshot, cost_to_come, parent_vertex);
  snapshot_ = snapshot;
  return;
}

void Vertex::updateLeftChild(
    const ContinuousPath& path,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  left_child_ = std::make_tuple(path, stage_cost, child_vertex);
  return;
}

void Vertex::updateFrontChild(
    const ContinuousPath& path,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  front_child_ = std::make_tuple(path, stage_cost, child_vertex);
  return;
}

void Vertex::updateRightChild(
    const ContinuousPath& path,
    const double stage_cost,
    const boost::shared_ptr<Vertex>& child_vertex) {
  right_child_ = std::make_tuple(path, stage_cost, child_vertex);
  return;
}

const bool Vertex::sameLaneWithRoot(
    const boost::shared_ptr<const Vertex>& root) const {

  boost::shared_ptr<const WaypointNode> node = node_.lock();
  boost::shared_ptr<const WaypointNode> root_node = root->node();

  while (node) {
    if (node->id() == root_node->id()) return true;
    if (node->left()->id() == root_node->id()) return false;
    if (node->right()->id() == root_node->id()) return false;

    node = node->back();
  }

  // There must be something wrong if we cannnot find the root
  // node by looking backwards.
  std::string error_msg(
      "Vertex::sameLaneWithRoot(): "
      "Cannot find the root node by looking backwards.\n");
  throw std::runtime_error(error_msg + this->string());

  return false;
}

std::string Vertex::string(const std::string& prefix) const {
  std::string output = prefix;
  output += "node id: " + std::to_string(node_.lock()->id()) + "\n";
  output += "snapshot: \n" + snapshot_.string();

  boost::format parent_format("node id:%1% cost to come:%2%\n");
  output += "parent: ";
  if (parent_)
    output += (parent_format % std::get<2>(*parent_).lock()->node().lock()->id()
                             % std::get<1>(*parent_)).str();
  else output += "\n";

  boost::format child_format("node id:%1% path length:%2% stage cost:%3%\n");

  output += "front child: ";
  if (front_child_)
    output += (child_format % std::get<2>(*front_child_).lock()->node().lock()->id()
                            % std::get<0>(*front_child_).range()
                            % std::get<1>(*front_child_)).str();
  else output += "\n";

  output += "left child: ";
  if (left_child_)
    output += (child_format % std::get<2>(*left_child_).lock()->node().lock()->id()
                            % std::get<0>(*left_child_).range()
                            % std::get<1>(*left_child_)).str();
  else output += "\n";

  output += "right child: ";
  if (right_child_)
    output += (child_format % std::get<2>(*right_child_).lock()->node().lock()->id()
                            % std::get<0>(*right_child_).range()
                            % std::get<1>(*right_child_)).str();
  else output += "\n";

  return output;
}

} // End namespace slc_lattice_planner.
} // End namespace planner.

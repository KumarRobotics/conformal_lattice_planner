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

#include <conformal_lattice_planner/traffic_simulator.h>
#include <conformal_lattice_planner/conformal_lattice_planner.h>

namespace planner {

void Station::updateOptimalParent() {

  // Set the \c optimal_parent_ to a existing parent. It does not
  // matter which parent is used for now.
  if (!optimal_parent_) {
    if (left_parent_) optimal_parent_ = left_parent_;
    else if (back_parent_) optimal_parent_ = back_parent_;
    else if (right_parent_) optimal_parent_ = right_parent_;
    else throw std::runtime_error("Cannot update optimal parent since there is no parent available.");
  }

  // Set the \c optimal_parent_ to the existing parent with the minimum cost-to-come.
  if (back_parent_ && back_parent_->first <= optimal_parent_->first)
    optimal_parent_ = back_parent_;
  if (left_parent_ && left_parent_->first < optimal_parent_->first)
    optimal_parent_ = left_parent_;
  if (right_parent_ && right_parent_->first < optimal_parent_->first)
    optimal_parent_ = right_parent_;

  return;
}

void Station::updateLeftParent(
    const double cost_to_come, const boost::shared_ptr<Station>& parent_station) {
  left_parent_ = std::make_pair(cost_to_come, parent_station);
  updateOptimalParent();
  return;
}

void Station::updateBackParent(
    const double cost_to_come, const boost::shared_ptr<Station>& parent_station) {
  back_parent_ = std::make_pair(cost_to_come, parent_station);
  updateOptimalParent();
  return;
}

void Station::updateRightParent(
    const double cost_to_come, const boost::shared_ptr<Station>& parent_station) {
  right_parent_ = std::make_pair(cost_to_come, parent_station);
  updateOptimalParent();
  return;
}

void Station::updateLeftChild(
    const ContinuousPath& path,
    const double stage_cost,
    const boost::shared_ptr<Station>& child_station) {
  left_child_ = std::make_tuple(path, stage_cost, child_station);
  return;
}

void Station::updateFrontChild(
    const ContinuousPath& path,
    const double stage_cost,
    const boost::shared_ptr<Station>& child_station) {
  front_child_ = std::make_tuple(path, stage_cost, child_station);
  return;
}

void Station::updateRightChild(
    const ContinuousPath& path,
    const double stage_cost,
    const boost::shared_ptr<Station>& child_station) {
  right_child_ = std::make_tuple(path, stage_cost, child_station);
  return;
}


ConformalLatticePlanner::ConformalLatticePlanner(
    const double time_step,
    const size_t ego,
    const double plan_horizon,
    const boost::shared_ptr<router::LoopRouter>& router) :
  Base   (time_step),
  router_(router) {

  boost::shared_ptr<CarlaWaypoint> ego_waypoint = carlaVehicleWaypoint(ego);
  waypoint_lattice_ = boost::make_shared<WaypointLattice<router::LoopRouter>>(
      ego_waypoint, plan_horizon, 2.0, router_);

  return;
}


void ConformalLatticePlanner::plan(
    const std::pair<size_t, double> ego,
    const std::unordered_map<size_t, double>& agents) {
  initializeRootStation(ego, agents);
  return;
}

void ConformalLatticePlanner::initializeRootStation(
    const std::pair<size_t, double> ego,
    const std::unordered_map<size_t, double>& agents) {

  Vehicle ego_vehicle = Vehicle(carlaVehicle(ego.first), ego.second);
  std::unordered_map<size_t, Vehicle> agent_vehicles;
  for (const auto& agent : agents) {
    agent_vehicles.insert(std::make_pair(
          agent.first, Vehicle(carlaVehicle(agent.first), agent.second)));
  }

  // Get the start snapshot.
  Snapshot snapshot(ego_vehicle, agent_vehicles, router_, map_);

  // Create the starting node.
  boost::shared_ptr<Station> root = boost::make_shared<Station>(
      snapshot, waypoint_lattice_, map_);
  node_to_station_table_[root->id()] = root;
  root_ = root;

  return;
}

void ConformalLatticePlanner::constructStationGraph() {
  std::queue<boost::shared_ptr<Station>> station_queue;
  station_queue.push(root_.lock());

  while (!station_queue.empty()) {
    boost::shared_ptr<Station> station = station_queue.front();
    station_queue.pop();
    exploreFrontStation(station, station_queue);
    exploreLeftStation(station, station_queue);
    exploreRightStation(station, station_queue);
  }

  return;
}

void ConformalLatticePlanner::exploreFrontStation(
    const boost::shared_ptr<Station>& station,
    std::queue<boost::shared_ptr<Station>>& station_queue) {

  // Find the target node on the waypoint lattice.
  // If the target node is not the lattice, we have reached the planning horizon.
  boost::shared_ptr<const WaypointNode> target_node =
    waypoint_lattice_->front(station->node().lock()->waypoint(), 50.0);
  if (!target_node) return;

  // Plan a path between the node at the current station to
  // the target node.
  boost::shared_ptr<ContinuousPath> path = nullptr;
  try {
    path = boost::make_shared<ContinuousPath>(
        station->transform(),
        target_node->waypoint()->GetTransform(),
        ContinuousPath::LaneChangeType::KeepLane);
  } catch (...) {
    // If for whatever reason, the path cannot be created,
    // just ignore this option.
    return;
  }

  // Now, simulate the traffic forward.
  TrafficSimulator simulator(station->snapshot(), map_);
  double simulation_time = 0.0; double stage_cost = 0.0;
  const bool no_collision = simulator.simulate(
      *path, time_step_, 5.0, simulation_time, stage_cost);

  // There a collision is detected in the simulation, this option is ignored.
  if (!no_collision) return;

  // Create a new station with the end snapshot of the simulation.
  boost::shared_ptr<Station> next_station = boost::make_shared<Station>(
      simulator.snapshot(), waypoint_lattice_, map_);

  // Set the child station of the input station.
  station->updateFrontChild(*path, stage_cost, next_station);
  // Set the parent station of the newly created station.
  if (station->hasParent()) {
    next_station->updateBackParent(
        station->optimalParent()->first+stage_cost, station);
  } else {
    next_station->updateBackParent(stage_cost, station);
  }

  // Add the new station to the table and queue if necessary.
  if (node_to_station_table_.count(next_station->id()) == 0) {
    node_to_station_table_[next_station->id()] = next_station;
    // The newly created station will only be added to the queue
    // if the target node is reached. Otherwise (the ego vehicle
    // did not reach end target node in time), we will consider
    // the new station as a terminate.
    if (next_station->id() == target_node->id()) station_queue.push(next_station);
  }

  return;
}

void ConformalLatticePlanner::exploreLeftStation(
    const boost::shared_ptr<Station>& station,
    std::queue<boost::shared_ptr<Station>>& station_queue) {
  return;
}
void ConformalLatticePlanner::exploreRightStation(
    const boost::shared_ptr<Station>& station,
    std::queue<boost::shared_ptr<Station>>& station_queue) {
  return;
}

} // End namespace planner.

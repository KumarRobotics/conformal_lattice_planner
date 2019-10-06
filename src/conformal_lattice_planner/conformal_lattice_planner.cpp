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

std::string Station::string(const std::string& prefix) const {
  std::string output = prefix;
  output += "id: " + std::to_string(id()) + "\n";
  output += "snapshot: \n" + snapshot_.string();

  boost::format parent_format("id:%1% cost to come:%2%\n");

  output += "back parent: ";
  if (back_parent_)
    output += ((parent_format) % back_parent_->second.lock()->id() % back_parent_->first).str();
  else output += "\n";

  output += "left parent: ";
  if (left_parent_)
    output += ((parent_format) % left_parent_->second.lock()->id() % left_parent_->first).str();
  else output += "\n";

  output += "right parent: ";
  if (right_parent_)
    output += ((parent_format) % right_parent_->second.lock()->id() % right_parent_->first).str();
  else output += "\n";

  output += "optimal parent: ";
  if (optimal_parent_)
    output += ((parent_format) % optimal_parent_->second.lock()->id() % optimal_parent_->first).str();
  else output += "\n";

  boost::format child_format("id:%1% path length:%2% stage cost:%3%\n");

  output += "front child: ";
  if (front_child_)
    output += ((child_format) % std::get<2>(*front_child_).lock()->id()
                              % std::get<0>(*front_child_).range()
                              % std::get<1>(*front_child_)).str();
  else output += "\n";

  output += "left child: ";
  if (left_child_)
    output += ((child_format) % std::get<2>(*left_child_).lock()->id()
                              % std::get<0>(*left_child_).range()
                              % std::get<1>(*left_child_)).str();
  else output += "\n";

  output += "right child: ";
  if (right_child_)
    output += ((child_format) % std::get<2>(*right_child_).lock()->id()
                              % std::get<0>(*right_child_).range()
                              % std::get<1>(*right_child_)).str();
  else output += "\n";

  return output;
}

std::vector<boost::shared_ptr<const Station>>
  ConformalLatticePlanner::nodes() const {

  std::vector<boost::shared_ptr<const Station>> stations;
  for (const auto& item : node_to_station_table_)
    stations.push_back(item.second);

  return stations;
}

std::vector<ContinuousPath> ConformalLatticePlanner::edges() const {

  std::vector<ContinuousPath> paths;
  for (const auto& item : node_to_station_table_) {
    const boost::shared_ptr<const Station> station = item.second;

    if (station->hasFrontChild())
      paths.push_back(std::get<0>(*(station->frontChild())));

    if (station->hasLeftChild())
      paths.push_back(std::get<0>(*(station->leftChild())));

    if (station->hasRightChild())
      paths.push_back(std::get<0>(*(station->rightChild())));
  }

  return paths;
}

DiscretePath ConformalLatticePlanner::plan(
    const size_t ego, const Snapshot& snapshot) {

  if (ego != snapshot.ego().id())
    throw std::runtime_error("The conformal lattice planner should only plan for the ego.");

  // Construct the waypoint lattice.
  initializeWaypointLattice(snapshot.ego());

  // Initialize the root station with the current snapshot.
  initializeRootStation(snapshot);

  // Construct the station graph.
  constructStationGraph();

  // Select the optimal path sequence from the station graph.
  std::list<ContinuousPath> optimal_path_seq = selectOptimalPath();

  // Merge the path sequence into one discrete path.
  DiscretePath optimal_path = mergePaths(optimal_path_seq);

  return optimal_path;
}

void ConformalLatticePlanner::initializeWaypointLattice(const Vehicle& ego) {

  std::printf("initializeWaypointLattice(): \n");

  boost::shared_ptr<CarlaWaypoint> ego_waypoint =
    map_->GetWaypoint(ego.transform().location);
  waypoint_lattice_ = boost::make_shared<WaypointLattice<router::LoopRouter>>(
      ego_waypoint, spatial_horizon_, 2.0, router_);

  return;
}

void ConformalLatticePlanner::initializeRootStation(const Snapshot& snapshot) {

  std::printf("initializeRootStation(): \n");

  boost::shared_ptr<Station> root = boost::make_shared<Station>(
      snapshot, waypoint_lattice_, map_);
  node_to_station_table_[root->id()] = root;
  root_ = root;

  std::cout << root_.lock()->string() << std::endl;

  return;
}

void ConformalLatticePlanner::constructStationGraph() {
  std::printf("constructStationGraph(): \n");
  std::queue<boost::shared_ptr<Station>> station_queue;
  station_queue.push(root_.lock());

  while (!station_queue.empty()) {
    boost::shared_ptr<Station> station = station_queue.front();
    station_queue.pop();
    exploreFrontStation(station, station_queue);
    exploreLeftStation(station, station_queue);
    exploreRightStation(station, station_queue);
  }

  //for (const auto& item : node_to_station_table_) {
  //  if (!item.second) throw std::runtime_error("node is not available.");
  //  std::cout << item.second->string() << std::endl;;
  //}

  return;
}

void ConformalLatticePlanner::exploreFrontStation(
    const boost::shared_ptr<Station>& station,
    std::queue<boost::shared_ptr<Station>>& station_queue) {

  std::printf("exploreFrontStation(): \n");

  // Find the target node on the waypoint lattice.
  // If the target node is not the lattice, we have reached the planning horizon.
  //std::printf("Find target node.\n");
  boost::shared_ptr<const WaypointNode> target_node =
    waypoint_lattice_->front(station->node().lock()->waypoint(), 50.0);
  if (!target_node) return;

  // Plan a path between the node at the current station to the target node.
  //std::printf("Compute Kelly-Nagy path.\n");
  boost::shared_ptr<ContinuousPath> path = nullptr;
  path = boost::make_shared<ContinuousPath>(
      station->transform(),
      target_node->waypoint()->GetTransform(),
      ContinuousPath::LaneChangeType::KeepLane);
  //try {
  //  path = boost::make_shared<ContinuousPath>(
  //      station->transform(),
  //      target_node->waypoint()->GetTransform(),
  //      ContinuousPath::LaneChangeType::KeepLane);
  //} catch (...) {
  //  // If for whatever reason, the path cannot be created,
  //  // just ignore this option.
  //  return;
  //}

  // Now, simulate the traffic forward.
  //std::printf("Simulate the traffic.\n");
  TrafficSimulator simulator(station->snapshot(), map_);
  double simulation_time = 0.0; double stage_cost = 0.0;
  const bool no_collision = simulator.simulate(
      *path, sim_time_step_, 5.0, simulation_time, stage_cost);

  // There a collision is detected in the simulation, this option is ignored.
  if (!no_collision) return;

  // Create a new station with the end snapshot of the simulation.
  //std::printf("Create child station.\n");
  boost::shared_ptr<Station> next_station = boost::make_shared<Station>(
      simulator.snapshot(), waypoint_lattice_, map_);

  // Add the new station to the table and queue if necessary.
  if (node_to_station_table_.count(next_station->id()) == 0) {
    node_to_station_table_[next_station->id()] = next_station;
    // The newly created station will only be added to the queue
    // if the target node is reached. Otherwise (the ego vehicle
    // did not reach end target node in time), we will consider
    // the new station as a terminate.
    if (next_station->id() == target_node->id()) station_queue.push(next_station);
  } else {
    next_station = node_to_station_table_[next_station->id()];
  }

  // Set the child station of the input station.
  //std::printf("Update the child station of the input station.\n");
  station->updateFrontChild(*path, stage_cost, next_station);

  // Set the parent station of the newly created station.
  //std::printf("Update the parent station of the new station.\n");
  if (station->hasParent()) {
    next_station->updateBackParent(
        station->optimalParent()->first+stage_cost, station);
  } else {
    next_station->updateBackParent(stage_cost, station);
  }

  return;
}

void ConformalLatticePlanner::exploreLeftStation(
    const boost::shared_ptr<Station>& station,
    std::queue<boost::shared_ptr<Station>>& station_queue) {

  std::printf("exploreLeftStation(): \n");

  // Check the left front and left back vehicles.
  //
  // If there are vehicles at the left front or left back of the ego,
  // meanwhile those vehicles has non-positive distance to the ego, we will
  // ignore this path option.
  //
  // TODO: Should we increase the margin of the check?
  boost::optional<std::pair<size_t, double>> left_front =
    station->snapshot().trafficLattice()->leftFront(station->snapshot().ego().id());
  boost::optional<std::pair<size_t, double>> left_back =
    station->snapshot().trafficLattice()->leftBack(station->snapshot().ego().id());

  if (left_front && left_front->second <= 0.0) {
    std::printf("Close left front vehicle.\n");
    return;
  }
  if (left_back  && left_back->second  <= 0.0) {
    std::printf("Close left back vehicle.\n");
    return;
  }

  // Find the target node on the waypoint lattice.
  // If the target node is not the lattice, we have reached the planning horizon.
  // TODO: Maybe check both \c leftFront and \c frontLeft.
  boost::shared_ptr<const WaypointNode> target_node =
    waypoint_lattice_->leftFront(station->node().lock()->waypoint(), 50.0);
  if (!target_node) return;

  // Plan a path between the node at the current station to the target node.
  //std::printf("Compute Kelly-Nagy path.\n");
  boost::shared_ptr<ContinuousPath> path = nullptr;
  path = boost::make_shared<ContinuousPath>(
      station->transform(),
      target_node->waypoint()->GetTransform(),
      ContinuousPath::LaneChangeType::LeftLaneChange);
  //try {
  //  path = boost::make_shared<ContinuousPath>(
  //      station->transform(),
  //      target_node->waypoint()->GetTransform(),
  //      ContinuousPath::LaneChangeType::LeftLaneChange);
  //} catch (...) {
  //  // If for whatever reason, the path cannot be created,
  //  // just ignore this option.
  //  return;
  //}

  // Now, simulate the traffic forward.
  //std::printf("Simulate the traffic.\n");
  TrafficSimulator simulator(station->snapshot(), map_);
  double simulation_time = 0.0; double stage_cost = 0.0;
  const bool no_collision = simulator.simulate(
      *path, sim_time_step_, 5.0, simulation_time, stage_cost);

  // There a collision is detected in the simulation, this option is ignored.
  if (!no_collision) return;

  // Create a new station with the end snapshot of the simulation.
  //std::printf("Create child station.\n");
  boost::shared_ptr<Station> next_station = boost::make_shared<Station>(
      simulator.snapshot(), waypoint_lattice_, map_);

  // Add the new station to the table and queue if necessary.
  if (node_to_station_table_.count(next_station->id()) == 0) {
    node_to_station_table_[next_station->id()] = next_station;
    // The newly created station will only be added to the queue
    // if the target node is reached. Otherwise (the ego vehicle
    // did not reach end target node in time), we will consider
    // the new station as a terminate.
    if (next_station->id() == target_node->id()) station_queue.push(next_station);
  } else {
    next_station = node_to_station_table_[next_station->id()];
  }

  // Set the child station of the input station.
  station->updateLeftChild(*path, stage_cost, next_station);

  // Set the parent station of the newly created station.
  //std::printf("Update the parent station of the new station.\n");
  if (station->hasParent()) {
    next_station->updateRightParent(
        station->optimalParent()->first+stage_cost, station);
  } else {
    next_station->updateRightParent(stage_cost, station);
  }

  return;
}

void ConformalLatticePlanner::exploreRightStation(
    const boost::shared_ptr<Station>& station,
    std::queue<boost::shared_ptr<Station>>& station_queue) {

  std::printf("exploreRightStation(): \n");

  // Check the right front and right back vehicles.
  //
  // If there are vehicles at the right front or right back of the ego,
  // meanwhile those vehicles has non-positive distance to the ego, we will
  // ignore this path option.
  //
  // TODO: Should we increase the margin of the check?
  boost::optional<std::pair<size_t, double>> right_front =
    station->snapshot().trafficLattice()->rightFront(station->snapshot().ego().id());
  boost::optional<std::pair<size_t, double>> right_back =
    station->snapshot().trafficLattice()->rightBack(station->snapshot().ego().id());

  if (right_front && right_front->second <= 0.0) {
    std::printf("Close right front vehicle.\n");
    return;
  }
  if (right_back  && right_back->second  <= 0.0) {
    std::printf("Close right back vehicle. \n");
    return;
  }

  // Find the target node on the waypoint lattice.
  // If the target node is not the lattice, we have reached the planning horizon.
  // TODO: Maybe check both \c rightFront and \c frontRight.
  boost::shared_ptr<const WaypointNode> target_node =
    waypoint_lattice_->rightFront(station->node().lock()->waypoint(), 50.0);
  if (!target_node) return;

  // Plan a path between the node at the current station to the target node.
  //std::printf("Compute Kelly-Nagy path.\n");
  boost::shared_ptr<ContinuousPath> path = nullptr;
  path = boost::make_shared<ContinuousPath>(
      station->transform(),
      target_node->waypoint()->GetTransform(),
      ContinuousPath::LaneChangeType::RightLaneChange);
  //try {
  //  path = boost::make_shared<ContinuousPath>(
  //      station->transform(),
  //      target_node->waypoint()->GetTransform(),
  //      ContinuousPath::LaneChangeType::RightLaneChange);
  //} catch (...) {
  //  // If for whatever reason, the path cannot be created,
  //  // just ignore this option.
  //  return;
  //}

  // Now, simulate the traffic forward.
  //std::printf("Simulate the traffic.\n");
  TrafficSimulator simulator(station->snapshot(), map_);
  double simulation_time = 0.0; double stage_cost = 0.0;
  const bool no_collision = simulator.simulate(
      *path, sim_time_step_, 5.0, simulation_time, stage_cost);

  // There a collision is detected in the simulation, this option is ignored.
  if (!no_collision) return;

  // Create a new station with the end snapshot of the simulation.
  //std::printf("Create child station.\n");
  boost::shared_ptr<Station> next_station = boost::make_shared<Station>(
      simulator.snapshot(), waypoint_lattice_, map_);

  // Add the new station to the table and queue if necessary.
  if (node_to_station_table_.count(next_station->id()) == 0) {
    node_to_station_table_[next_station->id()] = next_station;
    // The newly created station will only be added to the queue
    // if the target node is reached. Otherwise (the ego vehicle
    // did not reach end target node in time), we will consider
    // the new station as a terminate.
    if (next_station->id() == target_node->id()) station_queue.push(next_station);
  } else {
    next_station = node_to_station_table_[next_station->id()];
  }

  // Set the child station of the input station.
  station->updateRightChild(*path, stage_cost, next_station);

  // Set the parent station of the newly created station.
  //std::printf("Update the parent station of the new station.\n");
  if (station->hasParent()) {
    next_station->updateLeftParent(
        station->optimalParent()->first+stage_cost, station);
  } else {
    next_station->updateLeftParent(stage_cost, station);
  }

  return;
}

const double ConformalLatticePlanner::terminalSpeedCost(
    const boost::shared_ptr<Station>& station) const {

  if (station->hasChild())
    throw std::runtime_error("The station is not a terminal station.");

  static std::unordered_map<int, double> cost_map {
    {0, 3.0}, {1, 3.0}, {2, 2.0}, {3, 2.0}, {4, 2.0},
    {5, 2.0}, {6, 1.0}, {7, 1.0}, {8, 0.0}, {9, 0.0},
  };

  const double ego_speed = station->snapshot().ego().speed();
  const double ego_policy_speed = station->snapshot().ego().policySpeed();
  if (ego_speed < 0.0 || ego_policy_speed < 0.0)
    throw std::runtime_error("invalid ego speed or ego policy speed.");

  // FIXME: What if the policy speed is 0.
  //        Should not worry about this too much here since,
  //        1) IDM does not support 0 policy speed.
  //        2) 0 policy speed may cause some invalidity in carla, such as
  //           looking for a waypoint 0m ahead.
  const double speed_ratio = ego_speed / ego_policy_speed;

  // There is no cost if the speed of the ego matches or exceeds the policy speed.
  if (speed_ratio >= 1.0) return 0.0;
  else return cost_map[static_cast<int>(speed_ratio*10.0)];
}

const double ConformalLatticePlanner::terminalDistanceCost(
    const boost::shared_ptr<Station>& station) const {

  if (station->hasChild())
    throw std::runtime_error("The station is not a terminal station.");

  static std::unordered_map<int, double> cost_map {
    {0, 3.0}, {1, 3.0}, {2, 3.0}, {3, 3.0}, {4, 2.0},
    {5, 2.0}, {6, 2.0}, {7, 1.0}, {8, 1.0}, {9, 1.0},
  };

  const double distance_ratio = station->node().lock()->distance() / spatial_horizon_;

  if (distance_ratio >= 1.0) return 0.0;
  else return cost_map[static_cast<int>(distance_ratio*10.0)];
}

const double ConformalLatticePlanner::costFromRootToTerminal(
    const boost::shared_ptr<Station>& terminal) const {

  if (terminal->hasChild())
    throw std::runtime_error("The station is not a terminal station.");

  const double path_cost = (*(terminal->optimalParent())).first;
  const double terminal_speed_cost = terminalSpeedCost(terminal);
  const double terminal_distance_cost = terminalDistanceCost(terminal);

  // TODO: Weight the cost properly.
  return path_cost + terminal_speed_cost + terminal_distance_cost;
}

std::list<ContinuousPath> ConformalLatticePlanner::selectOptimalPath() const {

  std::printf("selectOptimalPath():\n");

  std::printf("Find optimal terminal station.\n");
  boost::shared_ptr<Station> optimal_station = nullptr;
  // Set the initial cost to a large enough number.
  double optimal_cost = 1.0e10;

  for (const auto& item : node_to_station_table_) {

    boost::shared_ptr<Station> station = item.second;
    // Only terminal stations are considered, i.e. stations without children.
    if (station->hasChild()) continue;
    const double station_cost = costFromRootToTerminal(station);
    std::printf("=============================================\n");
    std::cout << station->string();
    std::printf("cost:%f\n", station_cost);
    std::printf("=============================================\n");

    // Update the optimal station if the candidate has small cost.
    // Here we assume terminal station always has at least one parent station.
    // Otherwise, there is just on root station in the graph.
    // FIXME: Add the terminal cost as well.
    if (station_cost < optimal_cost) {
      optimal_station = station;
      optimal_cost = station_cost;
    }
  }

  // There should be at least one parent node.
  if (!optimal_station)
    throw std::runtime_error("No terminal station in the graph.");

  // There should always be parent stations for a terminal station.
  if (!optimal_station->hasParent())
    throw std::runtime_error("Graph only consists of one root station.");

  // Lambda function to get the child station IDs given a parent station.
  auto frontChildId = [this](const boost::shared_ptr<Station>& station)->boost::optional<size_t>{
    if (!(station->frontChild())) return boost::none;
    else return std::get<2>(*(station->frontChild())).lock()->id();
  };
  auto leftChildId = [this](const boost::shared_ptr<Station>& station)->boost::optional<size_t>{
    if (!(station->leftChild())) return boost::none;
    else return std::get<2>(*(station->leftChild())).lock()->id();
  };
  auto rightChildId = [this](const boost::shared_ptr<Station>& station)->boost::optional<size_t>{
    if (!(station->rightChild())) return boost::none;
    else return std::get<2>(*(station->rightChild())).lock()->id();
  };

  // Track back from the terminal station to find all the paths.
  std::list<ContinuousPath> path_sequence;
  boost::shared_ptr<Station> station = optimal_station;

  int counter = 0;

  std::printf("Trace back parent stations.\n");
  while (station->hasParent()) {

    std::cout << station->string() << std::endl;

    boost::shared_ptr<Station> parent_station =
      (*(station->optimalParent())).second.lock();

    // The station is the front child station of the parent.
    if (frontChildId(parent_station) &&
        frontChildId(parent_station) == station->id()) {
      path_sequence.push_front(std::get<0>(*(parent_station->frontChild())));
      station = parent_station;
      continue;
    }

    // The station is the left child station of the parent.
    if (leftChildId(parent_station) &&
        leftChildId(parent_station) == station->id()) {
      path_sequence.push_front(std::get<0>(*(parent_station->leftChild())));
      station = parent_station;
      continue;
    }

    // The station is the right child station of the parent.
    if (rightChildId(parent_station) &&
        rightChildId(parent_station) == station->id()) {
      path_sequence.push_front(std::get<0>(*(parent_station->rightChild())));
      station = parent_station;
      continue;
    }
  }

  return path_sequence;
}

DiscretePath ConformalLatticePlanner::mergePaths(
    const std::list<ContinuousPath>& paths) const {

  //std::printf("mergePaths(): \n");
  //std::printf("path #: %lu\n", paths.size());

  DiscretePath path(paths.front());
  for (std::list<ContinuousPath>::const_iterator iter = ++(paths.begin());
       iter != paths.end(); ++iter) path.append(*iter);
  return path;
}

} // End namespace planner.

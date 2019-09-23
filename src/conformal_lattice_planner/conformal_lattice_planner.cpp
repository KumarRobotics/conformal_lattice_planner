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

#include <conformal_lattice_planner/vehicle_planner.h>
#include <conformal_lattice_planner/conformal_lattice_planner.h>

namespace planner {
namespace detail {

Snapshot::Snapshot(
    const size_t ego,
    const std::vector<size_t>& agents,
    const boost::shared_ptr<CarlaWorld>& world,
    const boost::shared_ptr<router::LoopRouter>& router) {

  // Get the ego vehicle.
  boost::shared_ptr<CarlaVehicle> ego_vehicle =
    boost::static_pointer_cast<CarlaVehicle>(world->GetActor(ego));
  if (!ego_vehicle) throw std::runtime_error("Ego vehicle does not exist in the simulator.");

  // Get the agent vehicles.
  std::vector<boost::shared_ptr<CarlaVehicle>> agent_vehicles;
  for (const size_t agent : agents) {
    boost::shared_ptr<CarlaVehicle> agent_vehicle =
      boost::static_pointer_cast<CarlaVehicle>(world->GetActor(agent)));
    if (!agent_vehicle) throw std::runtime_error("Agent vehicle does not exist in the simulator.");
    agent_vehicles.push_back(agent_vehicle);
  }

  // Collect all vehicles into a single vector.
  std::vector<boost::shared_ptr<const CarlaVehicle>> all_vehicles;
  all_vehicles.push_back(ego_vehicle);
  for (auto agent : agent_vehicles) all_vehicles.push_back(agent);

  // Construct the traffic lattice.
  traffic_lattice_ = boost::make_shared<TrafficLattice<router::LoopRouter>>(
      all_vehicles, world->GetMap(), router);

  // Set the vehicles in the snapshot object.
  std::unordered_set<size_t> all_vehicle_ids = traffic_lattice_->vehicles();
  if (all_vehicle_ids.count(ego_vehicle->GetId()) == 0) {
    throw std::runtime_error("The ego vehicle cannot be registered onto the traffice lattice.");
  }

  ego_ = Vehicle(ego_vehicle);
  for (const auto& agent : agent_vehicles) {
    // Ignore the agent vehicles that cannot be registered onto the traffic lattice.
    if (all_vehicle_ids.count(agent->GetId()) == 0) continue;
    agents_[agent->GetId()] = Vehicle(agent);
  }

  return;
}

Snapshot::Snapshot(
    const boost::shared_ptr<const CarlaVehicle>& ego,
    const std::vector<boost::shared_ptr<const CarlaVehicle>>& agents,
    const boost::shared_ptr<CarlaMap>& map,
    const boost::shared_ptr<router::LoopRouter>& router) {

  // Collect all vehicles into a single vector.
  std::vector<boost::shared_ptr<const CarlaVehicle>> all_vehicles;
  all_vehicles.push_back(ego);
  for (auto agent : agents) all_vehicles.push_back(agent);

  // Construct the traffic lattice.
  traffic_lattice_ = boost::make_shared<TrafficLattice<router::LoopRouter>>(
      all_vehicles, map, router);

  // Set the vehicles in the snapshot object.
  std::unordered_set<size_t> all_vehicle_ids = traffic_lattice_->vehicles();
  if (all_vehicle_ids.count(ego->GetId()) == 0) {
    throw std::runtime_error("The ego vehicle cannot be registered onto the traffice lattice.");
  }

  ego_ = Vehicle(ego);
  for (const auto& agent : agents) {
    // Ignore the agent vehicles that cannot be registered onto the traffic lattice.
    if (all_vehicle_ids.count(agent->GetId()) == 0) continue;
    agents_[agent->GetId()] = Vehicle(agent);
  }

  return;
}

Snapshot::Snapshot(const Snapshot& other) :
  ego_(other.ego_),
  agents(other.agents_),
  traffic_lattice_(boost::make_shared<TrafficLattice<router::LoopRouter>>(other.traffic_lattice_)) {}

Snapshot& operator=(const Snapshot& other) {
  ego_ = other.ego_;
  agents_ = other.agents_;
  traffic_lattice_ = boost::make_shared<TrafficLattice<router::LoopRouter>>(*(other.traffic_lattice_));
  return *this;
}

KellyNagyPath::KellyNagyPath(const CarlaTransform& start, const CarlaTransform& end) :
  start_transform_(start),
  end_transform_(end),
  progress_(0.0) {

  // Convert the start and end to right handed coordinate system.
  const NonHolonomicPath::State start_state = carlaTransformToPathState(start);
  const NonHolonomicPath::State end_state = carlaTransformToPathState(end);

  path_.optimizePathFast(start_state, end_state);
  return;
}

NonHolonomicPath::State KellyNagyPath::carlaTransformToPathState(
    const CarlaTransform& transform) const {
  const CarlaTransform transform_rh = utils::convertTransform(transform);
  return NonHolonomicPath::State(transform_rh.location.x,
                                 transform_rh.location.y,
                                 transform_rh.rotation.yaw,
                                 0.0);
}

carla::geom::Transform KellyNagyPath::pathStateToCarlaTransform(
    const NonHolonomicPath::State& state) const {

  CarlaTransform transform;
  transform.location.x = state.x;
  transform.location.y = state.y;
  transform.rotation.yaw = state.theta;
  // Convert to left handed coordinates.
  utils::convertTransformInPlace(transform);

  // The input state only tells x, y, and theta/yaw.
  // We will interplate start and end transforms to get z, roll, and ptich.
  const double progress_ratio = progress_ / path_.sf;

  transform.location.z =
    start_transform_.location.z*(1-progress_ratio) +
    end_transform_.location.z*progress_ratio;

  transform.rotation.roll =
    start_transform_.rotation.roll*(1-progress_ratio) +
    end_transform_.rotation.roll*progress_ratio;

  transform.rotation.pitch =
    start_transform_.rotation.pitch*(1-progress_ratio) +
    end_transform_.rotation.pitch*progress_ratio;

  return transform;
}


const CarlaTransform KellyNagyPath::followPath(
    const double speed, const double acceleration, const double time_step) {

  if (finished()) {
    throw std::runtime_error("The end of path has been reached.");
  }

  if (speed < 0.0) {
    throw std::runtime_error("The input speed cannot be negative.");
  }

  // Compute how much time we should simulate forward. The duration depends on
  // several factors:
  // 1) the input time step;
  // 2) when the speed will reach 0;
  // 3) when the vehicle will reach the end of path.
  // The actual duration should be the minimum of the above three.
  double dt = time_step;

  if (acceleration < 0.0)
    dt = -speed/acceleration < dt ? -speed/acceleration : dt;

  if (speed*dt+0.5*acceleration*dt*dt > path_.sf-progress_) {
    const double distance = path_.sf - progress_;
    const double final_speed = std::sqrt(speed*speed + 2.0*acceleration*distance);
    dt = (final_speed - speed) / acceleration;
  }

  progress_ += speed*dt + 0.5*acceleration*dt*dt;
  return dt;
}

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
  if (left_parent_ && left_parent_->first < optimal_parent_->first)
    optimal_parent_ = left_parent_;
  if (back_parent_ && back_parent_->first < optimal_parent_->first)
    optimal_parent_ = back_parent_;
  if (right_parent_ && right_parent_->first < optimal_parent_->first)
    optimal_parent_ = right_parent_;

  return;
}

void Station::updateLeftParent(
    const double cost_to_come, const boost::shared_ptr<Station>& parent_station) {
  left_parent_ = std::make_pair<double, boost::weak_ptr<Station>>(cost_to_come, parent_station);
  updateOptimalParent();
  return;
}

void Station::updateBackParent(
    const double cost_to_come, const boost::shared_ptr<Station>& parent_station) {
  back_parent_ = std::make_pair<double, boost::weak_ptr<Station>>(cost_to_come, parent_station);
  updateOptimalParent();
  return;
}

void Station::updateRightParent(
    const double cost_to_come, const boost::shared_ptr<Station>& parent_station) {
  right_parent_ = std::make_pair<double, boost::weak_ptr<Station>>(cost_to_come, parent_station);
  updateOptimalParent();
  return;
}

void Station::updateLeftChild(
    const KellyNagyPath& path,
    const double stage_cost,
    const boost::shared_ptr<Station>& child_station) {
  left_child_ = std::make_tuple<KellyNagyPath, double, boost::weak_ptr<Station>>(
      path, stage_cost, child_station);
  return;
}

void Station::updateFrontChild(
    const KellyNagyPath& path,
    const double stage_cost,
    const boost::shared_ptr<Station>& child_station) {
  front_child_ = std::make_tuple<KellyNagyPath, double, boost::weak_ptr<Station>>(
      path, stage_cost, child_station);
  return;
}

void Station::updateRightChild(
    const KellyNagyPath& path,
    const double stage_cost,
    const boost::shared_ptr<Station>& child_station) {
  right_child_ = std::make_tuple<KellyNagyPath, double, boost::weak_ptr<Station>>(
      path, stage_cost, child_station);
  return;
}

} // End namespace detail.
} // End namespace planner.

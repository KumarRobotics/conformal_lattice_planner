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

#include <cmath>
#include <limits>

#include <conformal_lattice_planner/utils.h>
#include <conformal_lattice_planner/traffic_simulator.h>

namespace planner {

const double TrafficSimulator::egoAcceleration() const {

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

const double TrafficSimulator::agentAcceleration(const size_t agent) const {

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

const std::tuple<size_t, typename TrafficSimulator::CarlaTransform, double, double, double>
  TrafficSimulator::updatedAgentTuple(
      const size_t id, const double accel, const double dt) const {

    const Vehicle& agent = snapshot_.vehicle(id);

    // The updated speed.
    const double updated_speed = agent.speed() + accel*dt;

    // The updated transform.
    // In case the router cannot find the next waypoint for the agent,
    // we will used the orgin as the updated transform to indicate
    // that the vehicle does not exist anymore.
    // FIXME: Is there a better way to handle this?

    boost::shared_ptr<CarlaWaypoint> waypoint =
      map_->GetWaypoint(agent.transform().location);
    const double movement = agent.speed()*dt + 0.5*accel*dt*dt;
    boost::shared_ptr<CarlaWaypoint> next_waypoint =
      router_->frontWaypoint(waypoint, movement);

    double curvature = 0.0;
    CarlaTransform updated_transform;
    updated_transform.location = carla::geom::Vector3D(0.0, 0.0, 0.0);
    updated_transform.rotation = carla::geom::Rotation(0.0, 0.0, 0.0);

    if (next_waypoint) {
      updated_transform = next_waypoint->GetTransform();
      curvature = utils::curvatureAtWaypoint(next_waypoint, map_);
    }

    return std::make_tuple(id, updated_transform, updated_speed, accel, curvature);
}

const double TrafficSimulator::remainingTime(
    const double speed, const double accel, const double distance) const {

  if (speed < 0.0)
    throw std::runtime_error("The input speed cannot be negative.");

  // Just a number large enough.
  double t = 100.0;

  if (accel < 0.0) t = -speed / accel;

  if (speed*t+0.5*accel*t*t > distance) {
    const double final_speed = std::sqrt(speed*speed + 2.0*accel*distance);
    t = (final_speed - speed) / accel;
  }

  return t;
}

const double TrafficSimulator::ttcCost(const double ttc) const {
  // The cost map for ttc.
  // [0, 1) -> 4
  // [1, 2) -> 2
  // [2, 3) -> 1
  // {3, +) -> 0
  static std::unordered_map<int, double> cost_map {
    {0, 4.0}, {1, 2.0}, {2, 1.0}
  };

  if (ttc < 0.0) throw std::runtime_error("ttc < 0");

  const int ttc_key = static_cast<int>(ttc);
  if (ttc_key <= 2) return cost_map[ttc_key];
  else return 0.0;
}

const double TrafficSimulator::ttcCost() const {
  boost::optional<std::pair<size_t, double>> ego_lead =
    snapshot_.trafficLattice()->front(snapshot_.ego().id());

  if (!ego_lead) return ttcCost(10.0);
  else return ttcCost(ego_lead->second/snapshot_.ego().speed());
}

const double TrafficSimulator::accelCost(const double accel) const {
  // The cost map for brake.
  // [0, 1) -> 0
  // [1, 2) -> 1
  // [2, 4) -> 2
  // [4, 6) -> 4
  // [6, 8) -> 8
  // [8, +) -> 16
  static std::unordered_map<int, double> cost_map {
    {0, 0.0},
    {1, 1.0},
    {2, 2.0}, {3, 2.0},
    {4, 4.0}, {5, 4.0},
    {6, 8.0}, {7, 8.0},
  };

  if (accel >= 0.0) return 0.0;

  const int brake_key = static_cast<int>(-accel);
  if (brake_key < 8) return cost_map[brake_key];
  else return 16.0;
}

const double TrafficSimulator::accelCost() const {
  // We consider four vehicles in computing the accel cost.
  // The ego and the followers of the ego vehicle.
  boost::optional<std::pair<size_t, double>> back =
    snapshot_.trafficLattice()->back(snapshot_.ego().id());
  boost::optional<std::pair<size_t, double>> left_back =
    snapshot_.trafficLattice()->leftBack(snapshot_.ego().id());
  boost::optional<std::pair<size_t, double>> right_back =
    snapshot_.trafficLattice()->rightBack(snapshot_.ego().id());

  double ego_brake_cost = accelCost(snapshot_.ego().acceleration());
  double agent_brake_cost = 0.0;
  if (back)       agent_brake_cost += accelCost(snapshot_.vehicle(back->first).acceleration());
  if (left_back)  agent_brake_cost += accelCost(snapshot_.vehicle(left_back->first).acceleration());
  if (right_back) agent_brake_cost += accelCost(snapshot_.vehicle(right_back->first).acceleration());

  return ego_brake_cost + 0.5*agent_brake_cost;
}

} // End namespace planner.

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
    const double lead_speed = snapshot_.agent(lead->first).speed();
    const double following_distance = lead->second;
    accel = idm_->idm(snapshot_.ego().speed(),
                     snapshot_.ego().policySpeed(),
                     lead_speed,
                     following_distance);
    std::printf("lead speed: %f distance: %f\n", lead_speed, following_distance);
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
    const double lead_speed = snapshot_.agent(lead->first).speed();
    const double following_distance = lead->second;
    accel = idm_->idm(snapshot_.agent(agent).speed(),
                     snapshot_.agent(agent).policySpeed(),
                     lead_speed,
                     following_distance);
  } else {
    accel = idm_->idm(snapshot_.agent(agent).speed(),
                     snapshot_.agent(agent).policySpeed());
  }

  return accel;
}

const std::tuple<size_t, typename TrafficSimulator::CarlaTransform, double, double>
  TrafficSimulator::updatedAgentTuple(
      const size_t id, const double accel, const double dt) const {

    const Vehicle& agent = snapshot_.agent(id);

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
    std::printf("movement: %f\n", movement);
    boost::shared_ptr<CarlaWaypoint> next_waypoint =
      router_->frontWaypoint(waypoint, movement);

    CarlaTransform updated_transform;
    if (next_waypoint) {
      updated_transform = next_waypoint->GetTransform();
    }

    return std::make_tuple(id, updated_transform, updated_speed, accel);
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

} // End namespace planner.

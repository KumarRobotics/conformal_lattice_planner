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

//#include <iostream>
#include <vector>
#include <stdexcept>
#include <boost/pointer_cast.hpp>
//#include <boost/timer/timer.hpp>

#include <carla/rpc/VehicleControl.h>
#include <conformal_lattice_planner/lane_follower.h>

namespace planner {

void LaneFollower::updateTrafficLattice(
    const std::unordered_set<size_t>& vehicles) {

  // Collect all carla vehicles.
  std::vector<boost::shared_ptr<const CarlaVehicle>> vehicle_actors;
  for (const size_t id : vehicles) {
    boost::shared_ptr<CarlaVehicle> vehicle_actor =
      boost::static_pointer_cast<CarlaVehicle>(world_->GetActor(id));
    vehicle_actors.push_back(vehicle_actor);
  }

  // Check if \c router_ is valid.
  if (!router_)
    throw std::runtime_error("The router has not been set.");

  // Recreate the traffic lattice.
  std::unordered_set<size_t> disappear_vehicles;
  traffic_lattice_ = boost::make_shared<TrafficLattice<router::LoopRouter>>(
      vehicle_actors, map_, router_, disappear_vehicles);

  // FIXME: how to handle the vehicles that cannot be added to the lattice.
  //        Just print them for now.
  if (disappear_vehicles.size() > 0) {
    std::printf("disappear vehicles in lane follower traffic lattice construction:\n");
    for (const size_t id : disappear_vehicles) std::printf("%lu ", id);
    std::printf("\n");
  }

  return;
}

void LaneFollower::plan(
    const size_t target,
    const double policy_speed,
    const std::unordered_set<size_t>& others) {

  if (!router_)
    throw std::runtime_error("The router has not been set.");
  if (!traffic_lattice_)
    throw std::runtime_error("The traffic lattice has not been set.");
  if (!idm_)
    throw std::runtime_error("The intelligent driver model has not been set.");

  // Get the target vehicle.
  boost::shared_ptr<CarlaVehicle> vehicle =
    boost::static_pointer_cast<CarlaVehicle>(world_->GetActor(target));

  // Get the (desired) speed of the target vehicle.
  // TODO: Maybe the target should look ahead a bit for desired speed
  //       in order to avoid aggressive brake.
  // FIXME: It seems that the unit of speed limit is km/h.
  const double speed = vehicle->GetVelocity().Length();
  const double speed_limit = vehicle->GetSpeedLimit() / 3.6;
  const double desired_speed = std::min(speed_limit, policy_speed);

  // Compute the acceleration of the target vehicle..
  double accel = 0.0;
  try {
    boost::optional<std::pair<size_t, double>> lead = traffic_lattice_->front(target);
    if (lead) {
      // There is a lead vehicle.
      const size_t lead_vehicle = lead->first;
      const double lead_distance = lead->second;
      std::printf("Has lead vehicle:%lu distance:%f\n", lead_vehicle, lead_distance);
      // Get the speed of the lead vehicle.
      boost::shared_ptr<CarlaVehicle> vehicle =
        boost::static_pointer_cast<CarlaVehicle>(world_->GetActor(lead_vehicle));
      const double lead_speed = vehicle->GetVelocity().Length();
      accel = idm_->idm(speed, desired_speed, lead_speed, lead_distance);
    } else {
      std::printf("No lead vehicle.\n");
      // There is no lead vehicle.
      accel = idm_->idm(speed, desired_speed);
    }
  } catch (std::exception& e) {
    std::printf("%s\n", e.what());
    // This is probably because we cannnot find the target vehicle on the lattice.
    // In this case, we just assume there is no lead vehicle.
    accel = idm_->idm(speed, desired_speed);
  }

  // Update the vehicle speed and transform.
  boost::shared_ptr<CarlaWaypoint> waypoint =
    map_->GetWaypoint(vehicle->GetTransform().location);
  boost::shared_ptr<CarlaWaypoint> reference_waypoint =
    router_->frontWaypoint(waypoint, speed*time_step_ + 0.5*accel*time_step_*time_step_);

  const CarlaTransform reference_transform = reference_waypoint->GetTransform();
  const double reference_speed = speed + accel*time_step_;

  std::printf("speed:%f reference speed:%f desired_speed:%f\n",
      speed, reference_speed, desired_speed);

  vehicle->SetTransform(reference_transform);
  vehicle->SetVelocity(reference_transform.GetForwardVector()*reference_speed);

  return;
}

} // End namespace planner.

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

#include <vector>
#include <boost/pointer_cast.hpp>
#include <conformal_lattice_planner/lane_follower.h>

namespace planner {

void LaneFollower::plan(const size_t target,
          const std::vector<size_t>& others) {

  // Get the target vehicle.
  SharedPtr<Vehicle> target_vehicle =
    boost::static_pointer_cast<Vehicle>(world_->GetActor(target));

  // Get the (desired) speed of the target vehicle.
  // TODO: Maybe the target should look ahead a bit for desired speed
  //       in order to avoid aggressive brake.
  const double target_desired_speed = target_vehicle->GetSpeedLimit();
  SharedPtr<Waypoint> target_waypoint =
    map_->GetWaypoint(target_vehicle->GetTransform().location);
  double target_speed = target_vehicle->GetVelocity().Length();

  // Get the leader vehicle.
  SharedPtr<Vehicle> lead_vehicle = nullptr;
  boost::optional<size_t> leader =  findLeader(target, others);
  if (leader) {
    lead_vehicle = boost::static_pointer_cast<Vehicle>(world_->GetActor(*leader));
  }

  // Compute the acceleration of the target vehicle
  // using the intelligent driver model.
  double target_accel = 0.0;
  if (lead_vehicle) {
    // Lead vehicle speed.
    const double lead_speed = lead_vehicle->GetVelocity().Length();
    // Following distance.
    SharedPtr<Waypoint> lead_waypoint =
      map_->GetWaypoint(lead_vehicle->GetTransform().location);
    const double following_distance = lead_waypoint->GetDistance() -
                                      target_waypoint->GetDistance();

    target_accel = idm_.idm(
        target_speed, target_desired_speed, lead_speed, following_distance);

  } else {
    target_accel = idm_.idm(target_speed, target_desired_speed);
  }

  // Snap the target to the next closest waypoint.
  double distance_travelled = target_speed*time_step_ + 0.5*target_accel*time_step_*time_step_;
  target_waypoint = findNextWaypoint(target_waypoint, distance_travelled);
  target_vehicle->SetTransform(target_waypoint->GetTransform());

  // Set the velocity of the target vehicle.
  // TODO: If the physics of the vehicles is disabled, is it still necessary to set the velocity?
  target_speed += target_accel * time_step_;
  target_vehicle->SetVelocity(target_vehicle->GetTransform().GetForwardVector()*target_speed);

  return;
}

LaneFollower::SharedPtr<LaneFollower::Waypoint>
  LaneFollower::findNextWaypoint(
    const SharedPtr<Waypoint>& waypoint, const double distance) {

  // Get candidate next waypoints at a certain distance from the given waypoint.
  std::vector<SharedPtr<Waypoint>> candidates = waypoint->GetNext(distance);
  SharedPtr<Waypoint> next_waypoint = nullptr;

  // Loop through all candidates to find the one on the same lane.
  // TODO: What if road id or section id changes?
  // TODO: What happens if no candidate satisfies the requirement.
  for (const auto& candidate : candidates) {
    if (candidate->GetLaneId() != waypoint->GetLaneId()) continue;
    else next_waypoint = candidate;
  }

  return next_waypoint;
}

} // End namespace planner.

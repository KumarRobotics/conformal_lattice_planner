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

#pragma once

#include <boost/optional.hpp>
#include <boost/pointer_cast.hpp>
#include <conformal_lattice_planner/vehicle_planner.h>
#include <conformal_lattice_planner/intelligent_driver_model.h>
#include <conformal_lattice_planner/EgoPlanAction.h>

namespace planner {

class LaneFollower final : public VehiclePlanner {

protected:

  IntelligentDriverModel idm_;

public:

  LaneFollower(const double time_step,
               const std::string& host = "localhost",
               const uint16_t port = 2000,
               const boost::optional<IntelligentDriverModel&> idm) :
    VehiclePlanner(time_step, host, port), idm_(idm){}

  void plan(const size_t target,
            const std::vector<size_t>& others) override {

    // Get the target vehicle.
    SharedPtr<Vehicle> target_vehicle =
      boost::static_pointer_cast<Vehicle>(world_->GetActor(target));

    // Get the (desired) speed of the target vehicle.
    // TODO: Maybe the target should look ahead a bit to avoid aggressive brake.
    const double target_desired_speed = target_vehicle->GetSpeedLimit();
    SharedPtr<Waypoint> target_waypoint = map_->GetWaypoint(target_vehicle->GetTransform());
    double target_speed = target_vehicle->GetVelocity().Length();

    // Get the leader vehicle.
    SharedPtr<Vehicle> lead_vehicle = nullptr;
    boost::optional<size_t> leader =  findLeader(target, others);
    if (leader) {
      lead_vehicle = boost::static_pointer_cast<Vehicle>(world_->GetActor(target));
    }

    // Compute the acceleration of the target vehicle.
    double target_accel = 0.0;
    if (lead_vehicle) {
      // Lead vehicle speed.
      const double lead_speed = lead_vehicle->GetVelocity().Length();
      // Following distance.
      SharedPtr<Waypoint> lead_waypoint =
        map_.GetWaypoint(lead_vehicle->GetTransform());
      const double following_distance =
        lead_waypoint->GetDistance() - target_waypoint->GetDistance;

      target_accel = idm_.idm(
          target_speed, target_desired_speed, lead_speed, following_distance);

    } else {
      target_accel = idm_.idm(target_speed, target_desired_speed);
    }

    // Update the state of the target vehicle.
    // Snap the target vehicle back to the center of the lane.

    return;
  }

};

} // End namespace planner.


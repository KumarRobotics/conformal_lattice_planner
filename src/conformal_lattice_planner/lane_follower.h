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

#include <boost/core/noncopyable.hpp>
#include <conformal_lattice_planner/vehicle_planner.h>
#include <conformal_lattice_planner/intelligent_driver_model.h>
#include <conformal_lattice_planner/loop_router.h>
#include <conformal_lattice_planner/traffic_lattice.h>

namespace planner {

class LaneFollower : public VehiclePlanner,
                     private boost::noncopyable {

private:

  using Base = VehiclePlanner;

protected:

  /// Intelligent driver model,
  /// used to generate acceleration for a target vehicle.
  boost::shared_ptr<IntelligentDriverModel> idm_ = nullptr;

  /// Used to organize the traffic.
  boost::shared_ptr<TrafficLattice<router::LoopRouter>> traffic_lattice_ = nullptr;

  /// Router used to determine which lanes to follow.
  boost::shared_ptr<router::LoopRouter> router_ = nullptr;

public:

  LaneFollower(const double time_step) :
    Base(time_step),
    idm_(boost::make_shared<IntelligentDriverModel>()),
    router_(boost::make_shared<router::LoopRouter>()) {}

  void updateIDM(const IntelligentDriverModel& idm) {
    idm_ = boost::make_shared<IntelligentDriverModel>(idm);
    return;
  }

  void updateIDM(const boost::shared_ptr<const IntelligentDriverModel>& idm) {
    updateIDM(*idm);
    return;
  }

  void updateRouter(const router::LoopRouter& router) {
    router_ = boost::make_shared<router::LoopRouter>(router);
    return;
  }

  void updateRouter(const boost::shared_ptr<const router::LoopRouter>& router) {
    updateRouter(*router);
    return;
  }

  boost::shared_ptr<const TrafficLattice<router::LoopRouter>> trafficLattice() const {
    return traffic_lattice_;
  }

  void updateTrafficLattice(const std::unordered_set<size_t>& vehicles);

  void plan(const size_t target, const double policy_speed);
};

} // End namespace planner.


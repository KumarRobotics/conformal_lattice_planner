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

/**
 * \brief LaneFollower is supposed to control the target vehicle to follow
 *        the route defined by a router.
 *
 * The LaneFollower planner will adjust the speed of the target vehicle,
 * ensuring it won't collide with the vehicle at its front.
 *
 * To invoid the plan() interface, one should call updateWorld() and then
 * updateTrafficLattice() to make sure the planner will function correctly.
 *
 * \note The objects of this class are noncopyable, since two objects should not
 * share the same TrafficLattice.
 */
class LaneFollower : public VehiclePlanner,
                     private boost::noncopyable {

private:

  using Base = VehiclePlanner;

protected:

  /// Used to generate acceleration for a target vehicle.
  boost::shared_ptr<IntelligentDriverModel> idm_ = nullptr;

  /// Used to organize the traffic.
  boost::shared_ptr<TrafficLattice<router::LoopRouter>> traffic_lattice_ = nullptr;

  /// Used to determine the roads and lanes to follow.
  boost::shared_ptr<router::LoopRouter> router_ = nullptr;

public:

  /**
   * \brief Class constructor.
   *
   * The constructor will use the default IntelligentDriverModel constructor.
   *
   * \param[in] time_step The fixed time step of the simulator.
   */
  LaneFollower(const double time_step) :
    Base(time_step),
    idm_(boost::make_shared<IntelligentDriverModel>()),
    router_(boost::make_shared<router::LoopRouter>()) {}

  /// Update the IDM.
  void updateIDM(const IntelligentDriverModel& idm) {
    idm_ = boost::make_shared<IntelligentDriverModel>(idm);
    return;
  }

  /// Update the IDM.
  void updateIDM(const boost::shared_ptr<const IntelligentDriverModel>& idm) {
    updateIDM(*idm);
    return;
  }

  /// Update the router.
  void updateRouter(const router::LoopRouter& router) {
    router_ = boost::make_shared<router::LoopRouter>(router);
    return;
  }

  /// Update the router.
  void updateRouter(const boost::shared_ptr<const router::LoopRouter>& router) {
    updateRouter(*router);
    return;
  }

  /// Get the const TrafficLattice.
  boost::shared_ptr<const TrafficLattice<router::LoopRouter>> trafficLattice() const {
    return traffic_lattice_;
  }

  /**
   * \brief Update the traffic lattice.
   * \param[in] vehicles IDs of all vehicles to be considered when planning for
   *                     the target vehicle.
   */
  void updateTrafficLattice(const std::unordered_set<size_t>& vehicles);

  void plan(const std::pair<size_t, double> target,
                    const std::unordered_map<size_t, double>& others) override;
};

} // End namespace planner.


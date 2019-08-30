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

#include <array>
#include <boost/optional.hpp>
#include <conformal_lattice_planner/vehicle_controller.h>
#include <conformal_lattice_planner/vehicle_planner.h>
#include <conformal_lattice_planner/intelligent_driver_model.h>

namespace planner {

class LaneFollower final : public VehiclePlanner {

private:

  template<typename T>
  using SharedPtr = VehiclePlanner::SharedPtr<T>;

protected:

  IntelligentDriverModel idm_;
  //controller::VehiclePIDController controller_;

public:

  LaneFollower(const double time_step,
               const IntelligentDriverModel& idm = IntelligentDriverModel()) :
    VehiclePlanner(time_step), idm_(idm) {}

  void plan(const size_t target,
            const std::vector<size_t>& others) override;

  //void setControllerGains(
  //    const boost::optional<std::array<double, 3>&> longitudinal_gains = boost::none,
  //    const boost::optional<std::array<double, 3>&> lateral_gains = boost::none) {

  //  if (longitudinal_gains) {
  //    controller_.longitudinalKp() = (*longitudinal_gains)[0];
  //    controller_.longitudinalKi() = (*longitudinal_gains)[1];
  //    controller_.longitudinalKd() = (*longitudinal_gains)[2];
  //  }

  //  if (lateral_gains) {
  //    controller_.lateralKp() = (*lateral_gains)[0];
  //    controller_.lateralKi() = (*lateral_gains)[1];
  //    controller_.lateralKd() = (*lateral_gains)[2];
  //  }

  //  return;
  //}

private:

  SharedPtr<Waypoint> findNextWaypoint(
      const SharedPtr<Waypoint>& waypoint, const double distance);

};

} // End namespace planner.


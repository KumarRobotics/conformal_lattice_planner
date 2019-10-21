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
#include <boost/smart_ptr.hpp>

#include <planner/common/intelligent_driver_model.h>
#include <planner/common/snapshot.h>

namespace planner {

class VehicleSpeedPlanner : private boost::noncopyable {

protected:

  boost::shared_ptr<IntelligentDriverModel> intelligent_driver_model_;

public:

  /// Default constructor.
  VehicleSpeedPlanner() :
    intelligent_driver_model_(boost::make_shared<IntelligentDriverModel>()) {}

  /**
   * \brief Class constructor.
   * \param[in] model The intelligent driver model to be used to compute acceleration.
   */
  VehicleSpeedPlanner(const boost::shared_ptr<IntelligentDriverModel>& model) :
    intelligent_driver_model_(model) {}

  virtual ~VehicleSpeedPlanner() {}

  /// Get the intelligent driver model used in the object.
  const boost::shared_ptr<const IntelligentDriverModel> intelligentDriverModel() const {
    return intelligent_driver_model_;
  }

  /// Get or set the intelligent driver model used in the object.
  boost::shared_ptr<IntelligentDriverModel>& intelligentDriverModel() {
    return intelligent_driver_model_;
  }

  /**
   * \brief The main interface of the speed planner.
   *
   * \param[in] target The ID of the target vehicle.
   * \param[in] snapshot Snapshot of the current traffic scenario.
   * \return The acceleration to be applied for the target vehicle.
   */
  virtual const double plan(const size_t target, const Snapshot& snapshot) {
    // Get the target vehicle.
    const Vehicle target_vehicle = snapshot.vehicle(target);

    // Get the lead vehicle of the target.
    boost::optional<std::pair<size_t, double>> lead =
      snapshot.trafficLattice()->front(target_vehicle.id());

    // Compute the acceleration to be applied by the target vehicle.
    if (lead) {
      return intelligent_driver_model_->idm(
          target_vehicle.speed(),
          target_vehicle.policySpeed(),
          snapshot.vehicle(lead->first).speed(),
          lead->second);
    } else {
      return intelligent_driver_model_->idm(
          target_vehicle.speed(),
          target_vehicle.policySpeed());
    }
  }

  /**
   * \brief The main interface of the speed planner.
   *
   * \param[in] target The ID of the target vehicle.
   * \param[in] snapshot Snapshot of the current traffic scenario.
   * \param[out] The acceleration to be applied for the target vehicle.
   */
  virtual void plan(const size_t target, const Snapshot& snapshot, double& accel) {
    accel = plan(target, snapshot);
    return;
  }

}; // End class VehicleSpeedPlanner.

} // End namespace planner.


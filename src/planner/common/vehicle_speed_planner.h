/*
 * Copyright 2020 Ke Sun
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
  virtual const double planSpeed(const size_t target, const Snapshot& snapshot) {
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
  virtual void planSpeed(const size_t target, const Snapshot& snapshot, double& accel) {
    accel = planSpeed(target, snapshot);
    return;
  }

}; // End class VehicleSpeedPlanner.

} // End namespace planner.


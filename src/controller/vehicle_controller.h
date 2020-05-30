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

#include <iostream>
#include <cmath>
#include <deque>
#include <algorithm>
#include <numeric>
#include <boost/core/noncopyable.hpp>
#include <carla/geom/Transform.h>

namespace controller {

template<typename ControlledData>
class PIDControllerBase : private boost::noncopyable {

protected:

  double kp_;
  double ki_;
  double kd_;

  double integrated_err_ = 0.0;
  double previous_err_ = 0.0;

public:

  PIDControllerBase(const double kp, const double ki, const double kd) :
    kp_(kp), ki_(ki), kd_(kd) {}

  double kp() const { return kp_; }
  double& kp() { return kp_; }

  double ki() const { return ki_; }
  double& ki() { return ki_; }

  double kd() const { return kd_; }
  double& kd() { return kd_; }

  double control(const ControlledData& current,
                 const ControlledData& reference,
                 const double dt) {
    // Compute the latest error.
    const double error = this->error(current, reference, dt);

    // Compute the control.
    double pe = error;
    double ie = integrated_err_ + error;
    double de = (error-previous_err_) / dt;
    previous_err_ = error;
    integrated_err_ += error;

    return kp_*pe + ki_*ie + kd_*de;
  }

protected:

  virtual double error(const ControlledData& current,
                       const ControlledData& reference,
                       const double dt) = 0;
};

class PIDLongitudinalController final : public PIDControllerBase<double> {

private:
  using Base = PIDControllerBase<double>;

public:

  PIDLongitudinalController(
      const double kp, const double ki, const double kd) : Base(kp, ki, kd) {}

private:

  double error(const double& current,
               const double& reference,
               const double dt) override {

    const double err = reference - current;
    return err;
  }

};

class PIDLateralController final : public PIDControllerBase<carla::geom::Transform> {

private:
  using Base = PIDControllerBase<carla::geom::Transform>;

public:

  PIDLateralController(
      const double kp, const double ki, const double kd) : Base(kp, ki, kd) {}

private:

  double error(const carla::geom::Transform& current,
               const carla::geom::Transform& reference,
               const double dt) override {

    // Project the current direction (heading of the vehicle).
    carla::geom::Vector3D current_direction = current.GetForwardVector();
    current_direction.z = 0.0;

    // The direction fromt he current location to the reference location.
    carla::geom::Vector3D reference_direction(0.0, 0.0, 0.0);
    reference_direction.x = reference.location.x - current.location.x;
    reference_direction.y = reference.location.y - current.location.y;

    // Compute the angle between these two vectors.
    const double dot_product = current_direction.x*reference_direction.x +
                               current_direction.y*reference_direction.y;
    //std::printf("cos value: %f\n", dot_product/(current_direction.Length()*reference_direction.Length()));
    double cos_value = dot_product/(current_direction.Length()*reference_direction.Length());
    if (cos_value > 1.0) cos_value = 0.999999999;
    if (cos_value < -1.0) cos_value = -0.99999999;
    double angle = std::acos(cos_value);
    double angle_sign_flag = reference_direction.x*current_direction.y -
                             reference_direction.y*current_direction.x;

    angle = angle_sign_flag>=0.0 ? angle : -angle;
    return angle;
  }
};

class VehiclePIDController {

private:

  PIDLongitudinalController longitudinal_controller_;
  PIDLateralController lateral_controller_;

public:

  VehiclePIDController() :
    longitudinal_controller_(8.0, 0.0, 0.0),
    lateral_controller_(14.0, 0.0, 0.0) {}

  template<typename Container>
  VehiclePIDController(const Container& long_gains, const Container& late_gains) :
    longitudinal_controller_(long_gains[0], long_gains[1], long_gains[2]),
    lateral_controller_(late_gains[0], late_gains[1], late_gains[2]) {}

  double longitudinalKp() const { return longitudinal_controller_.kp(); }
  double& longitudinalKp() { return longitudinal_controller_.kp(); }

  double longitudinalKi() const { return longitudinal_controller_.ki(); }
  double& longitudinalKi() { return longitudinal_controller_.ki(); }

  double longitudinalKd() const { return longitudinal_controller_.kd(); }
  double& longitudinalKd() { return longitudinal_controller_.kd(); }

  double lateralKp() const { return lateral_controller_.kp(); }
  double& lateralKp() { return lateral_controller_.kp(); }

  double lateralKi() const { return lateral_controller_.ki(); }
  double& lateralKi() { return lateral_controller_.ki(); }

  double lateralKd() const { return lateral_controller_.kd(); }
  double& lateralKd() { return lateral_controller_.kd(); }

  double throttle(const double current_speed,
                  const double reference_speed,
                  const double dt) {
    return longitudinal_controller_.control(current_speed, reference_speed, dt);
  }

  double throttle(const double current_speed,
                  const double reference_speed,
                  const double dt,
                  const double throttle_max,
                  const double throttle_min) {
    double throttle =  longitudinal_controller_.control(current_speed, reference_speed, dt);
    if (throttle > throttle_max) throttle = throttle_max;
    if (throttle < throttle_min) throttle = throttle_min;
    return throttle;
  }

  double steering(const carla::geom::Transform& current_transform,
                  const carla::geom::Transform& reference_transform,
                  const double dt) {
    return lateral_controller_.control(current_transform, reference_transform, dt);
  }

  double steering(const carla::geom::Transform& current_transform,
                  const carla::geom::Transform& reference_transform,
                  const double dt,
                  const double steering_max,
                  const double steering_min) {
    double steering = lateral_controller_.control(current_transform, reference_transform, dt);
    if (steering > steering_max) steering = steering_max;
    if (steering < steering_min) steering = steering_min;
    return steering;
  }

};

} // End namespace controller


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

#include <cmath>
#include <boost/optional.hpp>

namespace planner {

/**
 * \brief BasicIntelligentDriverModel implements the simplest IDM.
 *
 * The details of the model can be found at the following pdf:
 * <http://www.traffic-flow-dynamics.org/res/SampleChapter11.pdf>
 */
class BasicIntelligentDriverModel {

protected:

  // See the reference for the meaning of these variables.
  double time_gap_ = 1.0;
  double distance_gap_ = 6.0;
  double accel_exp_ = 4.0;
  double comfort_accel_ = 1.5;
  double comfort_decel_ = 2.5;

  double max_accel_ = 5.0;
  double max_decel_ = 8.0;

public:

  /**
   * \brief Class constructor.
   *
   * The class provides default values for each of the parameters. One
   * can provide \c boost::none for a variable in the constructor if
   * the default value is to be used.
   */
  BasicIntelligentDriverModel(
      const boost::optional<double> time_gap = boost::none,
      const boost::optional<double> distance_gap = boost::none,
      const boost::optional<double> accel_exp = boost::none,
      const boost::optional<double> comfort_accel = boost::none,
      const boost::optional<double> comfort_decel = boost::none,
      const boost::optional<double> max_accel = boost::none,
      const boost::optional<double> max_decel = boost::none) {

    if (time_gap) time_gap_ = *time_gap;
    if (distance_gap) distance_gap_ = *distance_gap;
    if (accel_exp) accel_exp_ = *accel_exp;
    if (comfort_accel) comfort_accel_ = *comfort_accel;
    if (comfort_decel) comfort_decel_ = *comfort_decel;
    if (max_accel) max_accel_ = *max_accel;
    if (max_decel) max_decel_ = *max_decel;

    return;
  }

  /**
   * @name Accessors of the variables
   */
  /// @{
  double timeGap() const { return time_gap_; }
  double &timeGap() { return time_gap_; }

  double distanceGap() const { return distance_gap_; }
  double &distanceGap() { return distance_gap_; }

  double accelExp() const { return accel_exp_; }
  double &accelExp() { return accel_exp_; }

  double comfortAccel() const { return comfort_accel_; }
  double &comfortAccel() { return comfort_accel_; }

  double comfortDecel() const { return comfort_decel_; }
  double &comfortDecel() { return comfort_decel_; }

  double maxAccel() const { return max_accel_; }
  double &maxAccel() { return max_accel_; }

  double maxDecel() const { return max_decel_; }
  double &maxDecel() { return max_decel_; }
  /// @}

  /**
   * \brief The major interface of the class, computing the acceleration
   *        to the applied for the ego vehicle.
   *
   * \param[in] ego_v Speed of the ego vehicle.
   * \param[in] ego_v0 The desired speed of the ego vehicle.
   * \param[in] lead_v The speed of the lead vehicle, left empty if there is no lead.
   * \param[in] s The (positive) distance between the ego and lead vehicle,
   *              left empty if there is no lead.
   */
  virtual double idm(
      const double ego_v,
      const double ego_v0,
      const boost::optional<double> lead_v = boost::none,
      const boost::optional<double> s = boost::none) const {

    double accel = 0.0;

    if (lead_v && s) {
      // A lead vehicle presents.
      const double v_ratio = ego_v / ego_v0;
      const double s_star = desiredDistance(ego_v, *lead_v);
      const double s_ratio = s_star / *s;
      accel = comfort_accel_ * (1.0 - std::pow(v_ratio, accel_exp_) - std::pow(s_ratio, 2.0));
    } else {
      // There is no lead vehicle.
      const double v_ratio = ego_v / ego_v0;
      accel = comfort_accel_ * (1.0 - std::pow(v_ratio, accel_exp_));
    }

    return saturateAccel(accel);
  }

protected:

  /**
   * \brief Compute the desired following distance between the ego and lead vehicle.
   * \param[in] ego_v Speed of the ego vehicle.
   * \param[in] lead_v Speed of the lead vehicle.
   */
  double desiredDistance(const double ego_v, const double lead_v) const {
    const double v_diff = ego_v - lead_v;
    const double braking_coeff = 2.0 * std::sqrt(comfort_accel_ * comfort_decel_);
    const double distance = std::max(0.0, ego_v * time_gap_ + (ego_v * v_diff) / braking_coeff);
    return distance_gap_ + distance;
  }

  double saturateAccel(double accel) const {
    if (accel > max_accel_) accel = max_accel_;
    if (accel < -max_decel_) accel = -max_decel_;
    return accel;
  }

}; // End class BasicIntelligentDriverModel.


/**
 * \brief ImprovedIntelligentDriverModel implements the IIDM.
 */
class ImprovedIntelligentDriverModel : public BasicIntelligentDriverModel {

public:

   /**
   * \brief Class constructor.
   *
   * The class provides default values for each of the parameters. One
   * can provide \c boost::none for a variable in the constructor if
   * the default value is to be used.
   */
  ImprovedIntelligentDriverModel(
      const boost::optional<double> time_gap = boost::none,
      const boost::optional<double> distance_gap = boost::none,
      const boost::optional<double> accel_exp = boost::none,
      const boost::optional<double> comfort_accel = boost::none,
      const boost::optional<double> comfort_decel = boost::none,
      const boost::optional<double> max_accel = boost::none,
      const boost::optional<double> max_decel = boost::none) :
    BasicIntelligentDriverModel(
        time_gap, distance_gap,
        accel_exp,
        comfort_accel, comfort_decel,
        max_accel, max_decel) {}

  virtual double idm(
      const double ego_v,
      const double ego_v0,
      const boost::optional<double> lead_v = boost::none,
      const boost::optional<double> s = boost::none) const override {

    double accel = 0.0;
    double accel_free = freeAccel(ego_v, ego_v0);

    if ((!lead_v) || (!s)) return saturateAccel(accel_free);

    const double v_ratio = ego_v / ego_v0;
    const double s_star = desiredDistance(ego_v, *lead_v);
    const double s_ratio = s_star / *s;  // This is the same as Z.

    if (ego_v < ego_v0 && s_ratio >= 1.0)
      accel = comfort_accel_ * (1 - std::pow(s_ratio, 2));
    else if (ego_v < ego_v0 && s_ratio < 1.0)
      accel = accel_free * (1 - std::pow(s_ratio, (2 * comfort_accel_) / accel_free));
    else if (ego_v >= ego_v0 && s_ratio >= 1.0)
      accel = accel_free + comfort_accel_ * (1 - std::pow(s_ratio, 2));
    else
      accel = accel_free;

    return saturateAccel(accel);
  }

protected:

  double freeAccel(const double ego_v, const double ego_v0) const {
    // There is no lead vehicle, thus we use the free acceleration model. Eq 11.22
    double accel{0.0};
    if (ego_v <= ego_v0) {
      accel = comfort_accel_ * (1.0 - std::pow(ego_v/ego_v0, accel_exp_));
    } else { // v > v0
      accel = -comfort_decel_ * (1.0 - std::pow(ego_v0/ego_v, comfort_accel_ * accel_exp_ / comfort_decel_));
    }
    return accel;
  }

}; // End Class ImprovedIntelligentDriverModel

/**
 * \brief AdaptiveCruiseControl implements the ACC model.
 */
class AdaptiveCruiseControl : public ImprovedIntelligentDriverModel {

protected:

  double coolness_factor_ = 0.9;

public:

  /**
    * \brief Class constructor.
    *
    * The class provides default values for each of the parameters. One
    * can provide \c boost::none for a variable in the constructor if
    * the default value is to be used.
    */
  AdaptiveCruiseControl(
      const boost::optional<double> time_gap = boost::none,
      const boost::optional<double> distance_gap = boost::none,
      const boost::optional<double> accel_exp = boost::none,
      const boost::optional<double> comfort_accel = boost::none,
      const boost::optional<double> comfort_decel = boost::none,
      const boost::optional<double> max_accel = boost::none,
      const boost::optional<double> max_decel = boost::none,
      const boost::optional<double> coolness_factor = boost::none) :
    ImprovedIntelligentDriverModel(
        time_gap, distance_gap,
        accel_exp,
        comfort_accel, comfort_decel,
        max_accel, max_decel) {

    if (coolness_factor) coolness_factor_ = *coolness_factor;
    return;
  }

  double coolnessFactor() const { return coolness_factor_; }
  double& coolnessFactor() { return coolness_factor_; }

  double idm(const double ego_v,
             const double ego_v0,
             const boost::optional<double> lead_v = boost::none,
             const boost::optional<double> s = boost::none) const {

    double lead_v_dot = 0.0;
    double accel {0.0};
    double a_iidm = ImprovedIntelligentDriverModel::idm(ego_v, ego_v0, lead_v, s);

    if ((!lead_v) || (!s)) return a_iidm;

    double acah = constAccelHeuristic(ego_v, ego_v0, *lead_v, lead_v_dot, *s);

    // Implement Equation 11.26
    if (a_iidm >= acah) {
      accel = a_iidm;
    } else {
      accel = (1-coolness_factor_) * a_iidm +
               coolness_factor_ * (acah + comfort_decel_ * std::tanh((a_iidm - acah)/comfort_decel_));
    }

    return saturateAccel(accel);
  }

protected:

  /**
   * Compute the Constant Acceleration Heuristic Acceleration (ACAH)
   * @return The ACAH value.
   */
  double constAccelHeuristic(
      const double ego_v,
      const double ego_v0,
      const double lead_v,
      const double lead_v_dot,
      const double s) const {

    double a_tilde = std::min(lead_v_dot, comfort_accel_);
    double acah {0.0};
    // Heaviside Lambda Function
    auto heaviside = [](double x)->double{ return x>=0 ? 1.0 : 0.0;};

    // Implement Equation 11.25
    if (lead_v * (ego_v - lead_v) < -2 * s * a_tilde) { // First Case
      acah = std::pow(ego_v, 2) * a_tilde / (std::pow(lead_v, 2) - 2 * s * a_tilde);
    } else {
      acah = a_tilde - std::pow(ego_v - lead_v, 2) * heaviside(ego_v - lead_v) / (2 * s);
    }

    return acah;
  }

};

//using IntelligentDriverModel = BasicIntelligentDriverModel;
using IntelligentDriverModel = AdaptiveCruiseControl;

} // End namespace planner


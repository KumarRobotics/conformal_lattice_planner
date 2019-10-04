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

#include <boost/smart_ptr.hpp>
#include <boost/core/noncopyable.hpp>
#include <boost/optional.hpp>

#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <conformal_lattice_planner/intelligent_driver_model.h>
#include <conformal_lattice_planner/loop_router.h>
#include <conformal_lattice_planner/snapshot.h>

namespace planner {

class TrafficSimulator : private boost::noncopyable {

protected:

  using CarlaMap       = carla::client::Map;
  using CarlaWaypoint  = carla::client::Waypoint;
  using CarlaTransform = carla::geom::Transform;

protected:

  /// The snapshot of the traffic scenario.
  Snapshot snapshot_;

  /// Intelligent driver model.
  boost::shared_ptr<IntelligentDriverModel> idm_ = nullptr;

  /// Router.
  boost::shared_ptr<router::LoopRouter> router_ = nullptr;

  /// Carla map.
  boost::shared_ptr<CarlaMap> map_ = nullptr;

public:

  TrafficSimulator(const Snapshot& snapshot,
                   const boost::shared_ptr<CarlaMap>& map) :
    snapshot_(snapshot),
    idm_(boost::make_shared<IntelligentDriverModel>()),
    router_(boost::make_shared<router::LoopRouter>()),
    map_(map) {}

  const Snapshot& snapshot() const { return snapshot_; }

  const boost::shared_ptr<const IntelligentDriverModel> idm() const { return idm_; }
  boost::shared_ptr<IntelligentDriverModel>& idm() { return idm_; }

  const boost::shared_ptr<const router::LoopRouter> router() const { return router_; }
  boost::shared_ptr<router::LoopRouter>& router() { return router_; }

  /**
   * \brief Simulate the traffic.
   *
   * The simulation stops if either of the following two conditions is met:
   * -1 The maximum duration \c max_time is reached.
   * -2 The ego vehicle reaches the end of the path.
   * The actual terminate cause can be determined by the returned \c time.
   *
   * In the case that the function returns false, i.e. collision is detected,
   * the output \c time and \c cost are invalid. Once the function returns false,
   * the object should not be used anymore.
   *
   * \param[in] path The path to be executed by the ego vehicle.
   * \param[in] default_dt The default simulation time step.
   * \param[in] max_time The maximum duration to simulate.
   * \param[out] time The actual simulation duration.
   * \param[out] cost The accumulated cost during the simulation.
   * \return false If collision detected during the simulation.
   */
  template<typename Path>
  const bool simulate(
      const Path& path, const double default_dt, const double max_time,
      double& time, double& cost);

protected:

  /// Compute the acceleration of the ego vehicle given the current traffic scenario.
  const double egoAcceleration() const;

  /// Compute the acceleration of the agent vehicle given the current traffic scenario.
  const double agentAcceleration(const size_t agent) const;

  const std::tuple<size_t, CarlaTransform, double, double>
    updatedAgentTuple(const size_t id, const double accel, const double dt) const;

  /**
   * \brief This function is used to determine how much longer a vehicle can travel.
   *
   * The remaining time depends on two factors:
   * - 1. When will the speed decelerate to 0.
   * - 2. When will the vehicle travel the given distance.
   *
   * The output of this function should be compared with the default simulation
   * time resolution. Whichever is smaller should be used as the actual simulation
   * time step.
   *
   * \param[in] speed The current speed.
   * \param[in] accel The current acceleration.
   * \param[in] distance The maximum distance to travel.
   * \return The remaining time the vehicle can still travel.
   */
  const double remainingTime(
      const double speed, const double accel, const double distance) const;


}; // End class TrafficSimulator.

template<typename Path>
const bool TrafficSimulator::simulate(
    const Path& path, const double default_dt, const double max_time,
    double& time, double& cost) {

  std::printf("simulate(): \n");

  // Reset the output to 0.
  time = 0.0;
  // TODO: Add the cost function later.
  cost = 0.0;

  // The actual simulation time step, which may vary for different iterations.
  double dt = default_dt;
  // The distance that ego has travelled on the input path.
  double ego_distance = 0.0;

  // FIXME: This is just a trial for defining the stage costs.
  // \c headway stores the time gap between the ego and its front vehicle.
  // In case that the ego does not have a front vehicle, the headway is
  // set to 100, which is assumed to be a large enough number.
  std::vector<double> headway;
  // \c brake stores the brakes, and brakes only, of all vehicles in the
  // current snapshot. At each instances, the stored number should be
  // the average brake of all vehicles that brakes. If no vehicle brakes,
  // 0 is added to the vector.
  std::vector<double> brake;

  while (time < max_time && dt >= default_dt) {

    // Used to store the updated status of all vehicles.
    std::vector<std::tuple<size_t, CarlaTransform, double, double>> updated_tuples;

    // The acceleration to be applied by the ego vehicle.
    const double ego_accel = egoAcceleration();

    // Compute the actual time step.
    const double remaining_time = remainingTime(
        snapshot_.ego().speed(), ego_accel, path.range()-ego_distance);
    dt = default_dt;
    dt = dt <= remaining_time ? dt : remaining_time;
    dt = dt <= max_time-time  ? dt : max_time-time;

    std::printf("============================================\n");
    std::cout << snapshot_.string("simulation snapshot:\n");
    std::printf("ego accel: %f\n", ego_accel);
    std::printf("time: %f dt: %f\n", time, dt);

    // Update the distance of the ego on the path.
    ego_distance += snapshot_.ego().speed()*dt + 0.5*ego_accel*dt*dt;
    if (ego_distance > path.range()) ego_distance = path.range();
    std::printf("ego distance: %f\n", ego_distance);

    // Store the updated status of the ego.
    updated_tuples.push_back(std::make_tuple(
          snapshot_.ego().id(),
          path.transformAt(ego_distance),
          snapshot_.ego().speed()+ego_accel*dt,
          ego_accel));

    // Take care of the agents.
    for (const auto& item : snapshot_.agents()) {
      const Vehicle& agent = item.second;
      const double agent_accel = agentAcceleration(agent.id());
      updated_tuples.push_back(updatedAgentTuple(agent.id(), agent_accel, dt));
    }

    // Update the snapshot.
    if (!snapshot_.updateTraffic(updated_tuples)) {
      std::printf("Collision detected in the simulation.\n");
      return false;
    }

    // TODO: Accumulate the cost.
    // Compute the headway.
    boost::optional<std::pair<size_t, double>> ego_lead =
      snapshot_.trafficLattice()->front(snapshot_.ego().id());
    if (!ego_lead) headway.push_back(100.0);
    else headway.push_back(ego_lead->second / snapshot_.ego().speed());

    // Compute the average brake.
    std::pair<double, size_t> brake_sum = std::make_pair(0.0, 0);
    if (snapshot_.ego().acceleration() < 0.0) {
      brake_sum.first += -snapshot_.ego().acceleration();
      brake_sum.second += 1.0;
    }

    for (const auto& item : snapshot_.agents()) {
      const Vehicle& agent = item.second;
      if (agent.acceleration() >= 0.0) continue;
      brake_sum.first += -agent.acceleration();
      brake_sum.second += 1;
    }

    if (brake_sum.second > 0) brake.push_back(brake_sum.first/brake_sum.second);
    else brake.push_back(0.0);

    // Tick the time.
    time += dt;
  }

  double headway_cost = 0.0, brake_cost = 0.0;
  for (const auto c : headway) headway_cost += 1.0 / c;
  headway_cost /= headway.size();
  for (const auto c : brake) brake_cost += c;
  brake_cost /= brake.size();

  std::printf("average headway: %f\n", headway_cost);
  std::printf("average brake: %f\n", brake_cost);

  return true;
}

} // End namespace planner.


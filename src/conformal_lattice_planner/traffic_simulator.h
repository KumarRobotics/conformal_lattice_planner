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
#include <conformal_lattice_planner/vehicle_path.h>
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

  /// Fast waypoint map.
  boost::shared_ptr<utils::FastWaypointMap> fast_map_ = nullptr;

public:

  TrafficSimulator(const Snapshot& snapshot,
                   const boost::shared_ptr<CarlaMap>& map,
                   const boost::shared_ptr<utils::FastWaypointMap>& fast_map) :
    snapshot_(snapshot),
    idm_(boost::make_shared<IntelligentDriverModel>()),
    router_(boost::make_shared<router::LoopRouter>()),
    map_(map),
    fast_map_(fast_map) {}

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

  const std::tuple<size_t, CarlaTransform, double, double, double>
    updatedAgentTuple(const size_t id, const double accel, const double dt) const;

  /// Compute the ttc cost based on the input ttc.
  const double ttcCost(const double ttc) const;

  /// Compute the ttc cost based on the ttc of the ego vehicle in the snapshot.
  const double ttcCost() const;

  /// Compute the accel cost based on the input accel.
  const double accelCost(const double accel) const;

  /// Compute the accel cost based on the accel of the vehicles in the snapshot.
  const double accelCost() const;

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

  //std::printf("simulate(): \n");

  // Reset the output to 0.
  time = 0.0;
  cost = 0.0;

  // The actual simulation time step, which may vary for different iterations.
  double dt = default_dt;
  // The distance that ego has travelled on the input path.
  double ego_distance = 0.0;

  // FIXME: This is just a trial for defining the stage costs.
  std::vector<double> ttc_cost;
  std::vector<double> brake_cost;

  while (time < max_time && dt >= default_dt) {

    // Used to store the updated status of all vehicles.
    std::vector<std::tuple<size_t, CarlaTransform, double, double, double>> updated_tuples;

    // The acceleration to be applied by the ego vehicle.
    const double ego_accel = egoAcceleration();

    // Compute the actual time step.
    const double remaining_time = remainingTime(
        snapshot_.ego().speed(), ego_accel, path.range()-ego_distance);
    dt = default_dt;
    dt = dt <= remaining_time ? dt : remaining_time;
    dt = dt <= max_time-time  ? dt : max_time-time;

    //std::printf("============================================\n");
    //std::cout << snapshot_.string("start snapshot:\n");
    //std::printf("ego accel:%f\n", ego_accel);
    //std::printf("default dt:%f max time:%f remaining time:%f\n",
    //    default_dt, max_time, remaining_time);
    //std::printf("time:%f dt:%f\n", time, dt);

    // Update the distance of the ego on the path.
    ego_distance += snapshot_.ego().speed()*dt + 0.5*ego_accel*dt*dt;
    if (ego_distance > path.range()) ego_distance = path.range();
    //std::printf("ego distance:%f path range:%f\n", ego_distance, path.range());

    // Store the updated status of the ego.
    std::pair<CarlaTransform, double> ego_transform = path.transformAt(ego_distance);
    updated_tuples.push_back(std::make_tuple(
          snapshot_.ego().id(),
          ego_transform.first,
          snapshot_.ego().speed()+ego_accel*dt,
          ego_accel,
          ego_transform.second));

    // Take care of the agents.
    for (const auto& item : snapshot_.agents()) {
      const Vehicle& agent = item.second;
      const double agent_accel = agentAcceleration(agent.id());
      updated_tuples.push_back(updatedAgentTuple(agent.id(), agent_accel, dt));
      //std::printf("agent %lu accel: %f\n", agent.id(), agent_accel);
    }

    // Update the snapshot.
    if (!snapshot_.updateTraffic(updated_tuples)) {
      std::printf("Collision detected in the simulation.\n");
      return false;
    }

    //std::cout << snapshot_.string("end simulation snapshot:\n");

    // TODO: Accumulate the cost.
    ttc_cost.push_back(ttcCost());
    brake_cost.push_back(accelCost());

    //std::printf("ttc cost: %f\n", ttcCost());
    //std::printf("brake cost: %f\n", accelCost());

    // Tick the time.
    time += dt;
  }

  // TODO: Should I use mean or max?
  double average_ttc_cost = 0.0;
  for (const auto c : ttc_cost) average_ttc_cost += c;
  average_ttc_cost /= ttc_cost.size();

  double average_brake_cost = 0.0;
  for (const auto c : brake_cost) average_brake_cost += c;
  average_brake_cost /= brake_cost.size();

  //std::printf("average ttc cost: %f\n", average_ttc_cost);
  //std::printf("average brake cost: %f\n", average_brake_cost);
  //std::printf("\n");

  cost = average_ttc_cost + average_brake_cost;
  if (path.laneChangeType() != VehiclePath::LaneChangeType::KeepLane)
    cost += 1.0;

  return true;
}

} // End namespace planner.


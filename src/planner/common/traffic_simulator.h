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

#include <boost/smart_ptr.hpp>
#include <boost/core/noncopyable.hpp>
#include <boost/optional.hpp>

#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>

#include <router/common/router.h>
#include <router/loop_router/loop_router.h>
#include <planner/common/vehicle_path.h>
#include <planner/common/snapshot.h>

namespace planner {

/**
 * \brief TrafficSimulator simulates the traffic forward by a given
 *        period of time.
 *
 * All vehicles are assumed to have constant acceleration during the
 * simulation period. The path of the ego vehicle should be given as
 * a paremeter, while the rest of the agents are assumed to be lane
 * followers.
 *
 */
class TrafficSimulator : private boost::noncopyable {

protected:

  using CarlaMap       = carla::client::Map;
  using CarlaWaypoint  = carla::client::Waypoint;
  using CarlaTransform = carla::geom::Transform;

protected:

  /// The snapshot of the traffic scenario.
  Snapshot snapshot_;

  /// Router.
  boost::shared_ptr<router::Router> router_ = nullptr;

  /// Carla map.
  boost::shared_ptr<CarlaMap> map_ = nullptr;

  /// Fast waypoint map.
  boost::shared_ptr<utils::FastWaypointMap> fast_map_ = nullptr;

public:

  TrafficSimulator(const Snapshot& snapshot,
                   const boost::shared_ptr<router::Router>& router,
                   const boost::shared_ptr<CarlaMap>& map,
                   const boost::shared_ptr<utils::FastWaypointMap>& fast_map) :
    snapshot_(snapshot),
    router_(router),
    map_(map),
    fast_map_(fast_map) {}

  TrafficSimulator(const Snapshot& snapshot,
                   const boost::shared_ptr<CarlaMap>& map,
                   const boost::shared_ptr<utils::FastWaypointMap>& fast_map) :
    snapshot_(snapshot),
    router_(boost::make_shared<router::LoopRouter>()),
    map_(map),
    fast_map_(fast_map) {}

  const Snapshot& snapshot() const { return snapshot_; }

  const boost::shared_ptr<const router::Router> router() const { return router_; }
  boost::shared_ptr<router::Router>& router() { return router_; }

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
  virtual const bool simulate(
      const ContinuousPath& path, const double default_dt, const double max_time,
      double& time, double& cost);

protected:

  /// Compute the acceleration of the ego vehicle given the current traffic scenario.
  virtual const double egoAcceleration() const = 0;

  /// Compute the acceleration of the agent vehicle given the current traffic scenario.
  virtual const double agentAcceleration(const size_t agent) const = 0;

  virtual const std::tuple<size_t, CarlaTransform, double, double, double>
    updatedAgentTuple(const size_t id, const double accel, const double dt) const;

  /// Compute the ttc cost based on the input ttc.
  virtual const double ttcCost(const double ttc) const;

  /// Compute the ttc cost based on the ttc of the ego vehicle in the snapshot.
  virtual const double ttcCost() const;

  /// Compute the accel cost based on the input accel.
  virtual const double accelCost(const double accel) const;

  /// Compute the accel cost based on the accel of the vehicles in the snapshot.
  virtual const double accelCost() const;

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
  virtual const double remainingTime(
      const double speed, const double accel, const double distance) const;


}; // End class TrafficSimulator.

} // End namespace planner.


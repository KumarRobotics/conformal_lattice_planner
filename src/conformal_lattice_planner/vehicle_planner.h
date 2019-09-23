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

#include <stdexcept>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <boost/smart_ptr.hpp>

#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/client/Actor.h>
#include <carla/client/Vehicle.h>
#include <carla/geom/Transform.h>

namespace planner {

/**
 * \brief VehiclePlanner is supposed to be the base class of all planner classes.
 *
 * Before each time the plan() function is called, one should update
 * the carla world of the object by calling updateWorld().
 */
class VehiclePlanner {

protected:

  using CarlaClient        = carla::client::Client;
  using CarlaWorld         = carla::client::World;
  using CarlaMap           = carla::client::Map;
  using CarlaWaypoint      = carla::client::Waypoint;
  using CarlaActor         = carla::client::Actor;
  using CarlaVehicle       = carla::client::Vehicle;
  using CarlaTransform     = carla::geom::Transform;

protected:

  /// Fixed time interval of the simulation.
  double time_step_;

  /// Carla world object.
  boost::shared_ptr<CarlaWorld> world_;

  /// Carla map object.
  boost::shared_ptr<CarlaMap> map_;

public:

  /**
   * \brief Class constructor.
   * \param[in] time_step The fixed time step of the simulation.
   */
  VehiclePlanner(const double time_step) : time_step_(time_step){}

  /// Class destructor.
  virtual ~VehiclePlanner() {}

  /// Get the \c time_step_.
  double timeStep() const { return time_step_; }

  /// Get or set the \c time_step_.
  double& timeStep() { return time_step_; }

  /// Update the carla world of this object.
  void updateWorld(const boost::shared_ptr<CarlaWorld>& world) {
    world_ = world;
    map_ = world_->GetMap();
  }

  /**
   * \brief The plan interface of the planner.
   *
   * Each vehicle consists of the pair vehicle id + policy speed.
   *
   * \param[in] target The policy of the target vehicle.
   * \param[in] others Policies of all vehicles other than the target vehicle.
   */
  virtual void plan(const std::pair<size_t, double> target,
                    const std::unordered_map<size_t, double>& others) = 0;

};

} // End namespace planner.

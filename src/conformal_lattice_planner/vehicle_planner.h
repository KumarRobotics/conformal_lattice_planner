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
#include <boost/smart_ptr.hpp>

#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/client/Actor.h>
#include <carla/client/Vehicle.h>
#include <carla/client/WorldSnapshot.h>
#include <carla/client/ActorSnapshot.h>
#include <carla/geom/Transform.h>

namespace planner {

class VehiclePlanner {

protected:

  /// Lift the carla classes into the class namespace.
  using CarlaClient        = carla::client::Client;
  using CarlaWorld         = carla::client::World;
  using CarlaMap           = carla::client::Map;
  using CarlaWaypoint      = carla::client::Waypoint;
  using CarlaActor         = carla::client::Actor;
  using CarlaVehicle       = carla::client::Vehicle;
  using CarlaWorldSnapshot = carla::client::WorldSnapshot;
  using CarlaActorSnapshot = carla::client::ActorSnapshot;
  using CarlaTransform     = carla::geom::Transform;

protected:

  double time_step_;

  boost::shared_ptr<CarlaWorld> world_;
  boost::shared_ptr<CarlaMap> map_;

public:

  VehiclePlanner(const double time_step) : time_step_(time_step){}

  virtual ~VehiclePlanner() {}

  double timeStep() const { return time_step_; }

  double& timeStep() { return time_step_; }

  void updateWorld(const boost::shared_ptr<CarlaWorld>& world) {
    world_ = world;
    map_ = world_->GetMap();
  }

  /// Derived class should implement either of the interface.
  /// TODO: Is there a way I can check the implementation at compile time?
  ///       Do not want to declare the functions as pure virtual. Then the
  ///       derived classes have to implement both of the interface.
  virtual void plan(const size_t target, const double policy_speed) {
    throw std::runtime_error("VehiclePlanner does not implement plan()");
  }

  virtual void plan(const size_t target,
                    const double policy_speed,
                    const std::unordered_set<size_t>& others) {
    throw std::runtime_error("VehiclePlanner does not implement plan()");
  }

};

} // End namespace planner.

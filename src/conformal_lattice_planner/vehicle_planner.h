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

#include <string>
#include <vector>
#include <cstdint>
#include <chrono>
#include <boost/optional.hpp>
#include <boost/core/noncopyable.hpp>

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

  template<typename T>
  using SharedPtr = carla::SharedPtr<T>;

protected:

  const double time_step_;

  SharedPtr<CarlaWorld> world_;
  SharedPtr<CarlaMap> map_;

public:

  VehiclePlanner(const double time_step) :
    time_step_(time_step){}

  void updateWorld(const SharedPtr<CarlaWorld>& world) {
    world_ = world;
    map_ = world_->GetMap();
  }

  // TODO: Should I use a template container instead of \c std::vector<size_t>?
  //       If I use template, I have to templating other member functions as well.
  //       Sounds like a messy option.
  virtual void plan(const size_t target,
                    const double policy_speed,
                    const std::vector<size_t>& others) = 0;

  /// Find the lead vehicle on the same lane of the target.
  boost::optional<size_t> findLeader(
      const size_t target, const std::vector<size_t>& others) const;

  /// Find the following vehicle on the same lane of the target.
  boost::optional<size_t> findFollower(
      const size_t target, const std::vector<size_t>& others) const;

  /// Find the lead vehicle on the given lane.
  boost::optional<size_t> findLeader(
      const size_t target,
      const std::vector<size_t>& others,
      const size_t lane) const;

  /// Find the following vehicle on the given lane.
  boost::optional<size_t> findFollower(
      const size_t target,
      const std::vector<size_t>& others,
      const size_t lane) const;

};

} // End namespace planner.

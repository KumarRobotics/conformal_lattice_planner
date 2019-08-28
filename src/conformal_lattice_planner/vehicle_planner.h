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
  using Client        = carla::client::Client;
  using World         = carla::client::World;
  using Map           = carla::client::Map;
  using Waypoint      = carla::client::Waypoint;
  using Actor         = carla::client::Actor;
  using Vehicle       = carla::client::Vehicle;
  using WorldSnapshot = carla::client::WorldSnapshot;
  using ActorSnapshot = carla::client::ActorSnapshot;
  using Transform     = carla::geom::Transform;

  template<typename T>
  using SharedPtr    = carla::SharedPtr<T>;

protected:

  const double time_step_;

  SharedPtr<World> world_;
  SharedPtr<Map> map_;

public:

  VehiclePlanner(
      const double time_step,
      const std::string& host = "localhost",
      const uint16_t port = 2000) : time_step_(time_step) {
    Client client = Client(host, port);
    client.SetTimeout(std::chrono::seconds(10));
    world_ = boost::make_shared<World>(client.GetWorld());
    map_ = world_->GetMap();
    return;
  }

  // TODO: Should I use a template container instead of \c std::vector?
  virtual void plan(const size_t target,
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

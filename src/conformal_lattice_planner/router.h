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
#include <boost/optional.hpp>
#include <carla/client/Waypoint.h>

namespace router {

class Router {

protected:

  using CarlaWaypoint = carla::client::Waypoint;

public:

  virtual ~Router() {}

  /// Get the next road of the given road ID.
  virtual boost::optional<size_t> nextRoad(const size_t road) const = 0;

  /// Get the previous road of the given road ID.
  virtual boost::optional<size_t> prevRoad(const size_t road) const = 0;

  /// Get the next road given a waypoint on the current road.
  virtual boost::optional<size_t> nextRoad(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const = 0;

  /// Get the previous road given a waypoint on the current road.
  virtual boost::optional<size_t> prevRoad(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const = 0;

  /// Get the next waypoint with the given distance.
  virtual boost::shared_ptr<CarlaWaypoint> frontWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint, const double distance) const = 0;

  /// Get the left waypoint.
  virtual boost::shared_ptr<CarlaWaypoint> leftWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const = 0;

  /// Get the right waypoint.
  virtual boost::shared_ptr<CarlaWaypoint> rightWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const = 0;
}; // End class Router.

} // End namespace router.

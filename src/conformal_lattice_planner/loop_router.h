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

#include <vector>
#include <conformal_lattice_planner/router.h>

namespace router {

class LoopRouter : public Router {

protected:

  std::vector<size_t> road_sequence_;

public:

  LoopRouter();

  ~LoopRouter() { return; }

  //LoopRouter(const std::vector<size_t>& road_sequence) :
  //  road_sequence_(road_sequence) {}

  /// Get the next road of the given road ID.
  boost::optional<size_t> nextRoad(const size_t road) const override;

  /// Get the previous road of the given road ID.
  boost::optional<size_t> prevRoad(const size_t road) const override;

  /// Get the next road given a waypoint on the current road.
  boost::optional<size_t> nextRoad(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const override;

  /// Get the previous road given a waypoint on the current road.
  boost::optional<size_t> prevRoad(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const override;

  /// Get the next waypoint with the given distance.
  boost::shared_ptr<CarlaWaypoint> frontWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double distance) const override;

  /// Get the left waypoint.
  boost::shared_ptr<CarlaWaypoint> leftWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const override;

  /// Get the right waypoint.
  boost::shared_ptr<CarlaWaypoint> rightWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const override;

  /// Get the whole road sequence.
  const std::vector<size_t>& roadSequence() const {
    return road_sequence_;
  }

}; // End class LoopRouter.

} // End namespace router.

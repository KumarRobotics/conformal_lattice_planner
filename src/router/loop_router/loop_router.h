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
#include <router/common/router.h>

namespace router {

/**
 * \brief LoopRouter implements a predefined loop router which never ends.
 *
 * For now, the predefined route is the highway loop in the carla map Town04.
 */
class LoopRouter : public Router {

protected:

  std::vector<size_t> road_sequence_;

public:

  /**
   * \brief Default constructor.
   *
   * The constructor has a predefined road sequence, which forms a
   * a loop in the map.
   */
  LoopRouter();

  /// Destructor of the class.
  ~LoopRouter() { return; }

  bool hasRoad(const size_t road) const override {
    std::vector<size_t>::const_iterator iter = std::find(
        road_sequence_.begin(), road_sequence_.end(), road);
    return iter != road_sequence_.end();
  }

  boost::shared_ptr<CarlaWaypoint> waypointOnRoute(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const;

  /**
   * \brief In LoopRouter, there is always a next road for any route
   *        on the route. In the case that the query road is the last
   *        road in \c road_sequence_, the first road in the
   *        \c road_sequence_ is returned.
   */
  boost::optional<size_t> nextRoad(const size_t road) const override;

  /**
   * \brief In LoopRouter, there is always a previous road for any route
   *        on the route. In the case that the query road is the first
   *        road in \c road_sequence_, the last road in the
   *        \c road_sequence_ is returned.
   */
  boost::optional<size_t> prevRoad(const size_t road) const override;

  boost::optional<size_t> nextRoad(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const override;

  boost::optional<size_t> prevRoad(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const override;

  boost::shared_ptr<CarlaWaypoint> frontWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double distance) const override;

  /**
   * \brief Get the road sequence in the LoopRouter.
   *
   * Keep in mind that the next road of the last element in the returned
   * vector is the first element.
   */
  const std::vector<size_t>& roadSequence() const {
    return road_sequence_;
  }

}; // End class LoopRouter.

} // End namespace router.

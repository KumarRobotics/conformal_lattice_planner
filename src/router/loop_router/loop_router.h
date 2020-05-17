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
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const override;

  boost::optional<size_t> nextRoad(const size_t road) const override;

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
  const std::vector<size_t>& roadSequence() const override {
    return road_sequence_;
  }

}; // End class LoopRouter.

} // End namespace router.

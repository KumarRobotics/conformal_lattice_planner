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
#include <boost/optional.hpp>
#include <carla/client/Waypoint.h>

namespace router {

/**
 * \brief Router is supposed to be the base class for all router classes.
 *
 * The Router class should contain the road sequence and a vehicle is supposed
 * to follow. This helps the vehicle to define the forward direction.
 */
class Router {

protected:

  using CarlaWaypoint = carla::client::Waypoint;

public:

  virtual ~Router() {}

  /**
   * \brief Tells whether the query road is in the road sequence.
   * \param[in] road The ID of the query road.
   * \return True if the query road is in the sequence.
   */
  virtual bool hasRoad(const size_t road) const = 0;

  /**
   * \brief This function is supposed to find a carla waypoint at the
   *        same location of the query waypoint but have a different
   *        road ID that is on the route.
   *
   * It could happen in the carla simulator that two waypoints shares
   * the same location but have different road IDs (e.g. the off-ramp
   * overlaps with the highway road).
   *
   * FIXME: It is hard to implement such a function because of the
   *        limited APIs in the carla client.
   *
   * \param[in] waypoint The query waypoint.
   * \return The waypoint on the route that shares the same location with
   *         the query waypoint. If such a waypoint cannot be found,
   *         \c nullptr is returned.
   */
  virtual boost::shared_ptr<CarlaWaypoint> waypointOnRoute(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const = 0;

  /**
   * \brief Get the next road ID on the route of the query road.
   * \param[in] road The query road ID.
   * \return The next road ID if there is one. Otherwise \c boost::none
   *         is returned, which can happen the given road is the last
   *         one in the route.
   */
  virtual boost::optional<size_t> nextRoad(const size_t road) const = 0;

  /**
   * \brief Get the previous road ID on the route of the query road.
   * \param[in] road The query road ID.
   * \return The previous road ID if there is one. Otherwise \c boost::none
   *         is returned, which can happen the given road is the first
   *         one in the route.
   */
  virtual boost::optional<size_t> prevRoad(const size_t road) const = 0;

  /**
   * \brief Get the next road ID on the route of the road which has the
   *        query waypoint.
   * \param[in] waypoint The query waypoint.
   * \return The next road ID if there is one. Otherwise \c boost::none
   *         is returned, which can happen the road contains the query
   *         waypoint is the last road on the route.
   */
  virtual boost::optional<size_t> nextRoad(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const = 0;

  /**
   * \brief Get the previous road ID on the route of the road which has the
   *        query waypoint.
   * \param[in] waypoint The query waypoint.
   * \return The previous road ID if there is one. Otherwise \c boost::none
   *         is returned, which can happen the road contains the query
   *         waypoint is the first road on the route.
   */
  virtual boost::optional<size_t> prevRoad(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint) const = 0;

  /**
   * \brief Get the front waypoint of the query one with a certain distance.
   *
   * In the case such a front waypoint is available on the same road, the front
   * waypoint is returned. Otherwise, the function will search for a waypoint on
   * the next road on the route.
   *
   * \param[in] waypoint The query waypoint.
   * \param[in] distance The distance to look for the front waypoint.
   * \return If there is no such front waypoint, \c nullptr is returned.
   */
  virtual boost::shared_ptr<CarlaWaypoint> frontWaypoint(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint, const double distance) const = 0;

  /**
   * \brief Get the road sequence in the router
   */
  virtual const std::vector<size_t>& roadSequence() const = 0;

}; // End class Router.

} // End namespace router.

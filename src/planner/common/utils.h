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

#include <boost/functional/hash.hpp>
#include <boost/smart_ptr.hpp>

#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/geom/Transform.h>

namespace utils {

/**
 * \defgroup Hashing Functions
 *
 * Hash various hashable values into a single hash value.
 *
 * @{
 */
template<typename T>
void hashCombine(size_t& seed, T val) {
  boost::hash_combine(seed, val);
  return;
}

template<typename T, typename... Args>
void hashCombine(size_t& seed, T val, Args... args) {
  boost::hash_combine(seed, val);
  hashCombine(seed, args...);
  return;
}
/**@}*/

/**
 * \defgroup Left and Right Hand Cooordinate System Conversion
 *
 * Carla uses left hand coordinate systems. However, right hand coordinate
 * system is preferred in robotics (e.g. ROS). The incompatiblity can cause
 * great confusion leading to software bugs that are hard to trace. Therefore,
 * consider using the following functions before using a coordinate.
 *
 * Other than \c carla::geom::Transform uses left hand coordinate systems,
 * some are harder to recognize, like \c carla::client::Waypoint::GetLeft(),
 * which actually returns a right waypoint in the right hand coordinate systems.
 *
 * The conversion is done by flipping the positive y-axis. Therefore the y
 * coordinate is inverted in translation. The roll and yaw are inverted in
 * euler angles.
 *
 * @{
 */
void convertLocationInPlace(carla::geom::Location& in);

carla::geom::Location convertLocation(const carla::geom::Location& in);

void convertRotationInPlace(carla::geom::Rotation& in);

carla::geom::Rotation convertRotation(const carla::geom::Rotation& in);

void convertTransformInPlace(carla::geom::Transform& in);

carla::geom::Transform convertTransform(const carla::geom::Transform& in);
/**@}*/

const double curvatureAtWaypoint(
    const boost::shared_ptr<const carla::client::Waypoint>& waypoint,
    const boost::shared_ptr<const carla::client::Map>& map);

const double unrollAngle(double angle);

const double shortestAngle(double angle1, double angle2);

/**
 * \brief Compute the distance from the given location to the lane center.
 * \param[in] location The query location.
 * \param[in] waypoint The waypoint obtained by projecting the given location
 *                     to the closest lane.
 * \return The distance of the query location to the lane center. If the returned
 *         value is less than 0.0, the location is at the left of the lane. If
 *         the returned value is greater than 0.0, the location is on the right.
 */
const double distanceToLaneCenter(
    const carla::geom::Location& location,
    const boost::shared_ptr<const carla::client::Waypoint>& waypoint);

} // End namespace utils.

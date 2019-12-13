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

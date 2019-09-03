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
void convertLocationInPlace(carla::geom::Location& in) {
  in.y = -in.y;
  return;
}

carla::geom::Location convertLocation(const carla::geom::Location& in) {
  carla::geom::Location out = in;
  convertLocationInPlace(out);
  return out;
}

void convertRotationInPlace(carla::geom::Rotation& in) {
  in.roll = -in.roll;
  in.yaw = -in.yaw;
  return;
}

carla::geom::Rotation convertRotation(const carla::geom::Rotation& in) {
  carla::geom::Rotation out = in;
  convertRotationInPlace(out);
  return out;
}

void convertTransformInPlace(carla::geom::Transform& in) {
  convertRotationInPlace(in.rotation);
  convertLocationInPlace(in.location);
  return;
}

carla::geom::Transform convertTransform(const carla::geom::Transform& in) {
  carla::geom::Transform out = in;
  convertTransformInPlace(out);
  return out;
}
/**@}*/

} // End namespace utils.

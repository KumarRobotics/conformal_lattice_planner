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

#include <conformal_lattice_planner/utils.h>

namespace utils {

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

} // End namespace utils.

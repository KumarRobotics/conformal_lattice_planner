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

#include <stdexcept>
#include <string>
#include <boost/format.hpp>
#include <carla/road/Road.h>
#include <carla/road/element/RoadInfoGeometry.h>
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

const double curvatureAtWaypoint(
    const boost::shared_ptr<const carla::client::Waypoint>& waypoint,
    const boost::shared_ptr<const carla::client::Map>& map) {
  // Get the road.
  const carla::road::Road& road =
    map->GetMap().GetMap().GetRoad(waypoint->GetRoadId());

  // Get the road geometry info.
  const carla::road::element::RoadInfoGeometry* road_info =
    road.GetInfo<carla::road::element::RoadInfoGeometry>(waypoint->GetDistance());

  // Get the actual geometry of the road.
  const carla::road::element::Geometry& geometry = road_info->GetGeometry();

  // Get the curvature.
  double curvature = 0.0;

  if (geometry.GetType() == carla::road::element::GeometryType::LINE) {
    curvature = 0.0;

  } else if (geometry.GetType() == carla::road::element::GeometryType::ARC) {
    const carla::road::element::GeometryArc& geometry_arc =
      dynamic_cast<const carla::road::element::GeometryArc&>(geometry);
    curvature = geometry_arc.GetCurvature();

  } else if (geometry.GetType() == carla::road::element::GeometryType::SPIRAL) {
    //FIXME: Not sure how to deal with this.
    //       But there is no example for this road type from Town01 to Town07.
    std::string error_msg(
        "curvatureAtWaypoint(): "
        "cannot get curvature for waypointa on spiral roads.\n");
    boost::format format(
        "waypoint %1% x:%2% y:%3% z:%4% r:%5% p:%6% y:%7% road:%8% lane:%9%\n");
    format % waypoint->GetId()
           % waypoint->GetTransform().location.x
           % waypoint->GetTransform().location.y
           % waypoint->GetTransform().location.z
           % waypoint->GetTransform().rotation.roll
           % waypoint->GetTransform().rotation.pitch
           % waypoint->GetTransform().rotation.yaw
           % waypoint->GetRoadId()
           % waypoint->GetLaneId();

    throw std::runtime_error(error_msg+format.str());
  } else {
  }

  // Fix the curvature based on the sign of the lane ID.
  if (waypoint->GetLaneId() >= 0) return curvature;
  else return -curvature;
}

const double unrollAngle(double angle) {
  angle = std::remainder(angle, 360.0);
  if (angle < 0.0) angle += 360.0;
  return angle;
}

const double shortestAngle(double angle1, double angle2) {
  angle1 = unrollAngle(angle1);
  angle2 = unrollAngle(angle2);

  double diff = angle1 - angle2;
  if (std::abs(diff+360.0) < std::abs(diff)) diff += 360.0;
  if (std::abs(diff-360.0) < std::abs(diff)) diff -= 360.0;

  return diff;
}

} // End namespace utils.

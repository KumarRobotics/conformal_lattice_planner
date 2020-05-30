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

#include <cmath>
#include <stdexcept>
#include <string>
#include <boost/format.hpp>
#include <carla/road/Road.h>
#include <carla/road/element/RoadInfoGeometry.h>

#include <planner/common/utils.h>

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

const double distanceToLaneCenter(
    const carla::geom::Location& location,
    const boost::shared_ptr<const carla::client::Waypoint>& waypoint) {

  // Compute the location difference.
  const carla::geom::Transform waypoint_transform = waypoint->GetTransform();
  const carla::geom::Vector3D diff = location - waypoint_transform.location;

  // Compute the unit vector in the lateral direction.
  const double angle = (waypoint_transform.rotation.yaw+90.0)/180.0*M_PI;
  const carla::geom::Vector3D lateral_unit(std::cos(angle), std::sin(angle), 0.0);

  return diff.x*lateral_unit.x + diff.y*lateral_unit.y;
}

} // End namespace utils.

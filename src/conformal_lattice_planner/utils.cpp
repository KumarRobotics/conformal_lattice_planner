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

  if (geometry.GetType() == carla::road::element::GeometryType::LINE)
    curvature = 0.0;

  if (geometry.GetType() == carla::road::element::GeometryType::ARC) {
    const carla::road::element::GeometryArc& geometry_arc =
      dynamic_cast<const carla::road::element::GeometryArc&>(geometry);
    curvature = geometry_arc.GetCurvature();
  }

  if (geometry.GetType() == carla::road::element::GeometryType::SPIRAL) {
    //FIXME: Not sure how to deal with this. But there is no example for this road type
    //       from Town01 to Town07.
    throw std::runtime_error("Curvature for spiral road is not defined.\n");
    //const carla::road::element::GeometrySpiral& geometry_spiral =
    //  dynamic_cast<const carla::road::element::GeometrySpiral&>(geometry);
    //return geometry_spiral.GetCurvatureEnd();
  }

  //std::printf("lane id:%d curvature:%f\n", waypoint->GetLaneId(), curvature);
  if (waypoint->GetLaneId() >= 0) return curvature;
  else return -curvature;
}

} // End namespace utils.

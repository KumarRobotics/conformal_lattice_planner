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

#include <string>
#include <vector>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/core/noncopyable.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>

#include <planner/common/utils.h>

namespace utils {

class FastWaypointMap : private boost::noncopyable {

protected:

  using CarlaMap       = carla::client::Map;
  using CarlaWaypoint  = carla::client::Waypoint;
  using CarlaTransform = carla::geom::Transform;
  using CarlaLocation  = carla::geom::Location;

  /// Hash function of \c pcl::PointXYZ.
  struct PointXYZHash {
    size_t operator()(const pcl::PointXYZ& point) const {
      size_t seed = 0;
      hashCombine(seed, point.x, point.y, point.z);
      return seed;
    }
  };

  /// Compare function of \c pcl::PointXYZ.
  struct PointXYZEqual {
    bool operator()(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) const {
      return (p1.x==p2.x) & (p1.y==p2.y) & (p1.z==p2.z);
    }
  };

protected:

  /// Resolution of the waypoints (minimum distance).
  double resolution_;

  /// The map is stored as location-to-waypoint pair.
  std::unordered_map<pcl::PointXYZ,
                     boost::shared_ptr<CarlaWaypoint>,
                     PointXYZHash,
                     PointXYZEqual> xyz_to_waypoint_table_;

  /// KDtree built with all waypoint locations.
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;

  /// The point cloud built with all waypoint locations.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ = nullptr;

public:

  FastWaypointMap(const boost::shared_ptr<const CarlaMap>& map,
                  const double resolution = 0.05) : resolution_(resolution) {

    // Generate waypoints on the map with the given resolution.
    const std::vector<boost::shared_ptr<CarlaWaypoint>>
      waypoints = map->GenerateWaypoints(resolution_);

    // Add all waypoints to the table.
    // At the same time, collect all locations into a point cloud.
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud_->width = waypoints.size();
    cloud_->height = 1;

    for (const auto& waypoint : waypoints) {
      const pcl::PointXYZ point = carlaLocationToPointXYZ(waypoint->GetTransform().location);
      xyz_to_waypoint_table_[point] = waypoint;
      cloud_->push_back(point);
    }

    // Build a KDtree with the point cloud.
    kdtree_.setEpsilon(resolution_);
    kdtree_.setInputCloud(cloud_);

    return;
  }

  /// Get the resolution of the map.
  const double resolution() const { return resolution_; }

  /// Get the number of waypoints stored in the map.
  const size_t size() const { return xyz_to_waypoint_table_.size(); }

  boost::shared_ptr<CarlaWaypoint> waypoint(
      const CarlaLocation& location) const {

    // Create the query point based on the location.
    const pcl::PointXYZ query_point = carlaLocationToPointXYZ(location);

    // Search for the closest point.
    std::vector<int> indices(1);
    std::vector<float> sqr_distance(1);
    int num = kdtree_.nearestKSearch(query_point, 1, indices, sqr_distance);

    if (num <= 0) {
      std::string error_msg("Cannot find a waypoint close to the query location.\n");
      std::string location_msg = (
          boost::format("Query location: x:%1% y:%2% z:%3%\n")
          % location.x % location.y % location.z).str();
      std::string fast_map_msg = (
          boost::format("fast map size:%lu resolution:%f\n")
          % size() % resolution()).str();

      throw std::runtime_error(error_msg + location_msg);
    }

    const pcl::PointXYZ target_point = cloud_->at(indices[0]);
    const auto iter = xyz_to_waypoint_table_.find(target_point);

    const CarlaLocation target_location(
        target_point.x, target_point.y, target_point.z);
    if ((target_location-location).Length() > resolution_) {
      std::string error_msg = (boost::format(
          "FastWaypointMap::waypoint(): "
          "the distance between query location and closest location > %1%") % resolution_).str();
      std::string location_msg = (
          boost::format("query location x:%1% y:%2% z:%3%\n")
          % location.x % location.y % location.z).str();
      std::string target_location_msg = (
          boost::format("closest location x:%1% y:%2% z:%3%\n")
          % target_location.x % target_location.y % target_location.z).str();
    }

    return iter->second;
  }

  boost::shared_ptr<CarlaWaypoint> waypoint(
      const CarlaTransform& transform) const {
    return waypoint(transform.location);
  }

protected:

  pcl::PointXYZ carlaLocationToPointXYZ(const CarlaLocation& location) const {
    pcl::PointXYZ point;
    point.x = location.x;
    point.y = location.y;
    point.z = location.z;
    return point;
  }

}; // End class FastWaypointMap.

} // End namespace utils.

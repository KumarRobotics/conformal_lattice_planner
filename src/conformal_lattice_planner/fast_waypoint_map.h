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

#include <vector>
#include <unordered_map>
#include <boost/smart_ptr.hpp>
#include <boost/core/noncopyable.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>

#include <conformal_lattice_planner/utils.h>

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
    kdtree_.nearestKSearch(query_point, 1, indices, sqr_distance);

    const pcl::PointXYZ target_point = cloud_->at(indices[0]);
    const auto iter = xyz_to_waypoint_table_.find(target_point);

    if (iter == xyz_to_waypoint_table_.end())
      throw std::out_of_range("No waypoint close to the given location.");
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

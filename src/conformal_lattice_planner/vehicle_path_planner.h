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

#include <boost/smart_ptr.hpp>
#include <carla/client/Map.h>
#include <conformal_lattice_planner/snapshot.h>
#include <conformal_lattice_planner/fast_waypoint_map.h>
#include <conformal_lattice_planner/vehicle_path.h>

namespace planner {

/**
 * \brief VehiclePathPlanner is supposed to be the base class of all path planner classes.
 */
class VehiclePathPlanner {

protected:

  using CarlaMap = carla::client::Map;

protected:

  /// Carla map object.
  boost::shared_ptr<CarlaMap> map_ = nullptr;

  /// Fast waypoint map.
  boost::shared_ptr<utils::FastWaypointMap> fast_map_ = nullptr;

public:

  /**
   * \brief Class constructor.
   * \param[in] map The carla map pointer.
   * \param[in] fast_map The fast map used to retrieve waypoints based on locations.
   */
  VehiclePathPlanner(const boost::shared_ptr<CarlaMap>& map,
                     const boost::shared_ptr<utils::FastWaypointMap>& fast_map) :
    map_(map), fast_map_(fast_map) {}

  /// Class destructor.
  virtual ~VehiclePathPlanner() {}

  /// Get the carla map pointer.
  const boost::shared_ptr<const CarlaMap> map() const { return map_; }

  /// Get or set the carla map pointer.
  boost::shared_ptr<CarlaMap>& map() { return map_; }

  /// Get the fast waypoint map.
  const boost::shared_ptr<const utils::FastWaypointMap>
    fastWaypointMap() const { return fast_map_; }

  /// Get or set the fast waypoint map.
  boost::shared_ptr<utils::FastWaypointMap>&
    fastWaypointMap() { return fast_map_; }

  /**
   * \brief The main interface of the path planner.
   *
   * \param[in] target The ID of the target vehicle.
   * \param[in] snapshot Snapshot of the current traffic scenario.
   * \return The planned path.
   */
  virtual DiscretePath plan(const size_t target, const Snapshot& snapshot) = 0;

  /**
   * \brief The main interface of the path planner.
   *
   * \param[in] target The ID of the target vehicle.
   * \param[in] snapshot Snapshot of the current traffic scenario.
   * \param[out] path The planned path.
   */
  virtual void plan(const size_t target, const Snapshot& snapshot, DiscretePath& path) {
    path = plan(target, snapshot);
    return;
  }
};

} // End namespace planner.

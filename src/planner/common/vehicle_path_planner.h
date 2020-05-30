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

#include <boost/smart_ptr.hpp>
#include <carla/client/Map.h>

#include <planner/common/snapshot.h>
#include <planner/common/fast_waypoint_map.h>
#include <planner/common/vehicle_path.h>

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
  virtual DiscretePath planPath(const size_t target, const Snapshot& snapshot) = 0;

  /**
   * \brief The main interface of the path planner.
   *
   * \param[in] target The ID of the target vehicle.
   * \param[in] snapshot Snapshot of the current traffic scenario.
   * \param[out] path The planned path.
   */
  virtual void planPath(const size_t target, const Snapshot& snapshot, DiscretePath& path) {
    path = planPath(target, snapshot);
    return;
  }
};

} // End namespace planner.

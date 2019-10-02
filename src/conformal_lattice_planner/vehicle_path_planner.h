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

namespace planner {

/**
 * \brief VehiclePathPlanner is supposed to be the base class of all path planner classes.
 */
class VehiclePathPlanner {

protected:

  using CarlaMap = carla::client::Map;

protected:

  /// Carla map object.
  boost::shared_ptr<CarlaMap> map_;

public:

  /**
   * \brief Class constructor.
   * \param[in] map The carla map pointer.
   */
  VehiclePathPlanner(const boost::shared_ptr<CarlaMap>& map) : map_(map) {}

  /// Class destructor.
  virtual ~VehiclePathPlanner() {}

  /// Get the carla map pointer.
  const boost::shared_ptr<const CarlaMap> map() const { return map_; }

  /// Get or set the carla map pointer.
  boost::shared_ptr<CarlaMap>& map() { return map_; }
};

} // End namespace planner.

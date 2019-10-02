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

#include <boost/core/noncopyable.hpp>
#include <conformal_lattice_planner/waypoint_lattice.h>
#include <conformal_lattice_planner/vehicle_path.h>
#include <conformal_lattice_planner/vehicle_path_planner.h>

namespace planner {

/**
 * \brief LaneFollower is supposed to control the target vehicle to follow a predefine route.
 */
class LaneFollower : public VehiclePathPlanner,
                     private boost::noncopyable {

private:

  using Base = VehiclePathPlanner;
  using This = LaneFollower;

protected:

  using CarlaWaypoint = carla::client::Waypoint;

protected:

  // Used to find the waypoint on the same lane.
  boost::shared_ptr<WaypointLattice<router::LoopRouter>> waypoint_lattice_ = nullptr;

public:

  /**
   * \brief Class constructor.
   *
   * \param[in] map The carla map pointer.
   * \param[in] lattice_start The start waypoint of the lattice.
   * \param[in] lattice_range The range of the lattice.
   * \param[in] router The router to be used in creating the waypoint lattice.
   */
  LaneFollower(const boost::shared_ptr<CarlaMap>& map,
               const boost::shared_ptr<const CarlaWaypoint>& lattice_start,
               const double lattice_range,
               const boost::shared_ptr<router::LoopRouter>& router) :
    Base(map),
    waypoint_lattice_(boost::make_shared<WaypointLattice<router::LoopRouter>>(
          lattice_start, lattice_range, 5.0, router)) {}

  /// Get the waypoint lattice maintained in the object.
  boost::shared_ptr<const WaypointLattice<router::LoopRouter>> waypointLattice() const {
    return waypoint_lattice_;
  }

  /// Get or set the waypoint lattice maintained in the object.
  boost::shared_ptr<WaypointLattice<router::LoopRouter>>& waypointLattice() {
    return waypoint_lattice_;
  }

  /**
   * \brief The main interface of the path planner.
   *
   * \param[in] target The ID of the target vehicle.
   * \param[in] snapshot Snapshot of the current traffic scenario.
   * \return The planned path.
   */
  template<typename Path>
  Path plan(const size_t target, const Snapshot& snapshot);

  /**
   * \brief The main interface of the path planner.
   *
   * \param[in] target The ID of the target vehicle.
   * \param[in] snapshot Snapshot of the current traffic scenario.
   * \param[out] path The planned path.
   */
  template<typename Path>
  void plan(const size_t target, const Snapshot& snapshot, Path& path) {
    path = plan<Path>(target, snapshot);
    return;
  }
};

template<typename Path>
Path LaneFollower::plan(const size_t target, const Snapshot& snapshot) {

  // Get the target vehicle and its waypoint.
  const Vehicle target_vehicle = snapshot.vehicle(target);

  const boost::shared_ptr<CarlaWaypoint> target_waypoint =
    map_->GetWaypoint(target_vehicle.transform().location);

  // Waypoint lattice is created if not available.
  if (!waypoint_lattice_)
    throw std::runtime_error("The waypoint lattice is not available.\n");

  // Find the waypoint 50m ahead of the current postion of the target vehicle.
  const boost::shared_ptr<const WaypointNode> front_node =
    waypoint_lattice_->front(target_waypoint, 50.0);
  boost::shared_ptr<const CarlaWaypoint> front_waypoint = nullptr;

  if (!front_node) {
    if (target_vehicle.id() == snapshot.ego().id()) {
      // If there is no front node for the ego, there must be something else wrong.
      // The ego should be always follow the route.
      throw std::runtime_error("Ego vehicle can no longer keep the current lane.");
    } else {
      // If there is no front node for an agent vehicle. We may just find its next
      // accessible waypoint with some distance.
      std::vector<boost::shared_ptr<CarlaWaypoint>> front_waypoints =
        target_waypoint->GetNext(10.0);
      if (front_waypoints.size() <= 0)
        throw std::runtime_error("The agent vehicle cannot proceed further.");
      front_waypoint = front_waypoints[0];
    }
  } else {
    front_waypoint = front_node->waypoint();
  }

  // Generate the path.
  return Path(target_waypoint->GetTransform(),
              front_waypoint->GetTransform(),
              VehiclePath::LaneChangeType::KeepLane);
}

} // End namespace planner.


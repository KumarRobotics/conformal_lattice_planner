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
#include <planner/common/waypoint_lattice.h>
#include <planner/common/vehicle_path.h>
#include <planner/common/vehicle_path_planner.h>
#include <planner/common/utils.h>

namespace planner {
namespace lane_follower {

/**
 * \brief LaneFollower is supposed to control the target vehicle to follow a predefine route.
 */
class LaneFollower : public VehiclePathPlanner,
                     private boost::noncopyable {

private:

  using Base = VehiclePathPlanner;
  using This = LaneFollower;

protected:

  using CarlaWaypoint  = carla::client::Waypoint;
  using CarlaTransform = carla::geom::Transform;

protected:

  // Used to find the waypoint on the same lane.
  boost::shared_ptr<WaypointLattice<router::LoopRouter>> waypoint_lattice_ = nullptr;

  boost::shared_ptr<router::LoopRouter> router_ = nullptr;

public:

  /**
   * \brief Class constructor.
   *
   * \param[in] map The carla map pointer.
   * \param[in] fast_map The fast map used to retrieve waypoints based on locations.
   * \param[in] lattice_start The start waypoint of the lattice.
   * \param[in] lattice_range The range of the lattice.
   * \param[in] router The router to be used in creating the waypoint lattice.
   */
  LaneFollower(const boost::shared_ptr<CarlaMap>& map,
               const boost::shared_ptr<utils::FastWaypointMap>& fast_map,
               const boost::shared_ptr<const CarlaWaypoint>& lattice_start,
               const double lattice_range,
               const boost::shared_ptr<router::LoopRouter>& router) :
    Base(map, fast_map),
    waypoint_lattice_(boost::make_shared<WaypointLattice<router::LoopRouter>>(
          lattice_start, lattice_range, 5.0, router)),
    router_(router) {}

  /// Get the waypoint lattice maintained in the object.
  boost::shared_ptr<const WaypointLattice<router::LoopRouter>> waypointLattice() const {
    return waypoint_lattice_;
  }

  /// Get or set the waypoint lattice maintained in the object.
  boost::shared_ptr<WaypointLattice<router::LoopRouter>>& waypointLattice() {
    return waypoint_lattice_;
  }

  virtual DiscretePath planPath(const size_t target, const Snapshot& snapshot) override {

    // Get the target vehicle and its waypoint.
    const Vehicle target_vehicle = snapshot.vehicle(target);
    const boost::shared_ptr<CarlaWaypoint> target_waypoint =
      fast_map_->waypoint(target_vehicle.transform().location);

    // Waypoint lattice is created if not available.
    if (!waypoint_lattice_) {
      throw std::runtime_error(
          "LaneFollower::plan(): the waypoint lattice is not set yet.\n");
    }

    // Find the waypoint 50m ahead of the current postion of the target vehicle.
    const boost::shared_ptr<const WaypointNode> front_node =
      waypoint_lattice_->front(target_waypoint, 50.0);
    boost::shared_ptr<const CarlaWaypoint> front_waypoint = nullptr;

    if (!front_node) {
      if (target_vehicle.id() == snapshot.ego().id()) {
        // If there is no front node 50m ahead of the ego,
        // the waypoint lattice must be set incorrectly.
        std::string error_msg("LaneFollower::plan(): there is no node 50m ahead of ego.\n");
        std::string ego_msg = snapshot.ego().string();
        throw std::runtime_error(error_msg + ego_msg);
      } else if (front_waypoint = router_->frontWaypoint(target_waypoint, 50.0)) {
      } else {
        // If there is no front node for an agent vehicle. We may just find its next
        // accessible waypoint with some distance.
        std::vector<boost::shared_ptr<CarlaWaypoint>> front_waypoints =
          target_waypoint->GetNext(10.0);

        if (front_waypoints.size() <= 0) {
          std::string error_msg("LaneFollower::plan(): cannot find next waypoints for an agent.\n");
          std::string agent_msg = target_vehicle.string();
          std::string waypoint_msg =
            (boost::format("waypoint %1% x:%2% y:%3% z:%4% r:%5% p:%6% y:%7% road:%8% lane:%9%.\n")
             % target_waypoint->GetId()
             % target_waypoint->GetTransform().location.x
             % target_waypoint->GetTransform().location.y
             % target_waypoint->GetTransform().location.z
             % target_waypoint->GetTransform().rotation.roll
             % target_waypoint->GetTransform().rotation.pitch
             % target_waypoint->GetTransform().rotation.yaw
             % target_waypoint->GetRoadId()
             % target_waypoint->GetLaneId()).str();
          throw std::runtime_error(error_msg + agent_msg + waypoint_msg);
        }

        // Select the one with the least angle difference.
        front_waypoint = front_waypoints[0];
        for (size_t i = 1; i < front_waypoints.size(); ++i) {
          const double diff1 = utils::shortestAngle(
              target_waypoint->GetTransform().rotation.yaw,
              front_waypoint->GetTransform().rotation.yaw);
          const double diff2 = utils::shortestAngle(
              target_waypoint->GetTransform().rotation.yaw,
              front_waypoints[i]->GetTransform().rotation.yaw);

          if (std::fabs(diff2) < std::fabs(diff1))
            front_waypoint = front_waypoints[i];
        }

      }
    } else {
      front_waypoint = front_node->waypoint();
    }

    // FIXME: Maybe I should use the transform and curvature from the target vehicle.
    //        However this can cause drifting of the vehile. Planning from the closest
    //        waypoint seems to be the easiest fix.
    const CarlaTransform current_transform = target_waypoint->GetTransform();
    const double current_curvature = utils::curvatureAtWaypoint(target_waypoint, map_);

    const CarlaTransform reference_transform = front_waypoint->GetTransform();
    const double reference_curvature = utils::curvatureAtWaypoint(front_waypoint, map_);

    // Generate the path.
    return DiscretePath(
        std::make_pair(current_transform, current_curvature),
        std::make_pair(reference_transform, reference_curvature),
        VehiclePath::LaneChangeType::KeepLane);
  }
};

} // End namespace lane_follower.
} // End namespace planner.


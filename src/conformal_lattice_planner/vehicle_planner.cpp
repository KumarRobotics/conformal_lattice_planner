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

#include <limits>
#include <boost/pointer_cast.hpp>
#include <conformal_lattice_planner/vehicle_planner.h>

using namespace std;
namespace bst = boost;

namespace planner {

bst::optional<size_t> VehiclePlanner::findLeader(
    const size_t target, const std::vector<size_t>& others) const {

  // Get the target vehicle.
  SharedPtr<Actor> target_actor = world_->GetActor(target);
  SharedPtr<Waypoint> target_waypoint =
    map_->GetWaypoint(target_actor->GetTransform().location);

  SharedPtr<Actor> leader = nullptr;
  double min_distance_diff = numeric_limits<double>::max();

  // Loop through other actors to find the leader.
  for (const auto& other : others) {
    SharedPtr<Actor> actor = world_->GetActor(other);
    SharedPtr<Waypoint> waypoint =
      map_->GetWaypoint(actor->GetTransform().location);

    // Continue if the vehicle is not on the same road or
    // the same lane with the target.
    if (waypoint->GetRoadId() != target_waypoint->GetRoadId() ||
        waypoint->GetLaneId() != target_waypoint->GetLaneId()) continue;

    // Continue if the vehicle is behind the target.
    // TODO: Is GetDistance() really doing what I hope for?
    if (waypoint->GetDistance() < target_waypoint->GetDistance()) continue;

    // Check if this leader is closer.
    const double diff = waypoint->GetDistance() - target_waypoint->GetDistance();
    if (diff < min_distance_diff) {
      min_distance_diff = diff;
      leader = actor;
    }
  }

  if (leader) return leader->GetId();
  else return bst::none;
}

boost::optional<size_t> VehiclePlanner::findFollower(
    const size_t target, const std::vector<size_t>& others) const {

  // Get the target vehicle.
  SharedPtr<Actor> target_actor = world_->GetActor(target);
  SharedPtr<Waypoint> target_waypoint =
    map_->GetWaypoint(target_actor->GetTransform().location);

  SharedPtr<Actor> follower = nullptr;
  double min_distance_diff = numeric_limits<double>::max();

  // Loop through other actors to find the leader.
  for (const auto& other : others) {
    SharedPtr<Actor> actor = world_->GetActor(other);
    SharedPtr<Waypoint> waypoint =
      map_->GetWaypoint(actor->GetTransform().location);

    // Continue if the vehicle is not on the same road or
    // the same lane with the target.
    if (waypoint->GetRoadId() != target_waypoint->GetRoadId() ||
        waypoint->GetLaneId() != target_waypoint->GetLaneId()) continue;

    // Continue if the vehicle is behind the target.
    // TODO: Is GetDistance() really doing what I hope for?
    if (waypoint->GetDistance() > target_waypoint->GetDistance()) continue;

    // Check if this leader is closer.
    const double diff = target_waypoint->GetDistance() - waypoint->GetDistance();
    if (diff < min_distance_diff) {
      min_distance_diff = diff;
      follower = actor;
    }
  }

  if (follower) return follower->GetId();
  else return bst::none;
}

} // End namespace planner.

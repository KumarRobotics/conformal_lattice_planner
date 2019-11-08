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

#include <boost/format.hpp>

#include <carla/road/element/RoadInfoGeometry.h>

#include <planner/common/utils.h>
#include <planner/common/lattice.h>

namespace planner {

/**
 * \brief WaypointNode Keeps track of the waypoints on a lattice.
 *
 * WaypointNode is used together with Lattice class template to
 * form the WaypointLattice class template.
 *
 * \note The copy constructor of this class performs a shallow copy,
 *       i.e. only the shared and weak pointers are copied. This makes
 *       sense since the class copy constructor won't know which piece
 *       of memory the pointers should point to. In case one would like
 *       to redirect the pointers in the class to other objects, use
 *       the accessor interfaces.
 */
class WaypointNode : public LatticeNode<WaypointNode> {

protected:

  using CarlaMap      = carla::client::Map;
  using CarlaWaypoint = carla::client::Waypoint;

  using Base = LatticeNode<WaypointNode>;
  using This = WaypointNode;

public:

  /// Default constructor.
  WaypointNode() = default;

  /**
   * \brief Construct a node with a carla waypoint.
   * \param[in] waypoint A carla waypoint at which a node should be created.
   */
  WaypointNode(const boost::shared_ptr<const CarlaWaypoint>& waypoint) :
    Base(waypoint) {}

  // Get the string describing the node.
  std::string string(const std::string& prefix="") const {
    boost::format waypoint_format(
        "waypoint %1% x:%2% y:%3% z:%4% r:%5% p:%6% y:%7% road:%8% lane:%9%.\n");
    std::string waypoint_msg = (waypoint_format
        % this->waypoint_->GetId()
        % this->waypoint_->GetTransform().location.x
        % this->waypoint_->GetTransform().location.y
        % this->waypoint_->GetTransform().location.z
        % this->waypoint_->GetTransform().rotation.roll
        % this->waypoint_->GetTransform().rotation.pitch
        % this->waypoint_->GetTransform().rotation.yaw
        % this->waypoint_->GetRoadId()
        % this->waypoint_->GetLaneId()).str();
    std::string distance_msg = (boost::format("node distance: %1%\n") % this->distance_).str();
    return prefix + waypoint_msg + distance_msg;
    // TODO: Add the info for neighbor waypoints as well.
  }
}; // End class WaypointNode.

/**
 * \brief WaypointLattice is a helper class used to query
 *        the relative waypoints of the given waypoint.
 */
using WaypointLattice = Lattice<WaypointNode>;
} // End namespace planner.

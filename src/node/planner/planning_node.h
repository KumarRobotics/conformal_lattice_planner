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

#include <utility>
#include <unordered_map>

#include <boost/core/noncopyable.hpp>
#include <boost/smart_ptr.hpp>

#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Map.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Waypoint.h>
#include <carla/geom/Transform.h>
#include <carla/road/Road.h>
#include <carla/road/element/RoadInfoGeometry.h>

#include <ros/ros.h>
#include <router/loop_router/loop_router.h>
#include <planner/common/snapshot.h>
#include <planner/common/utils.h>
#include <planner/common/fast_waypoint_map.h>
#include <conformal_lattice_planner/TrafficSnapshot.h>

namespace node {

class PlanningNode : private boost::noncopyable {

protected:

  using CarlaClient    = carla::client::Client;
  using CarlaWorld     = carla::client::World;
  using CarlaMap       = carla::client::Map;
  using CarlaWaypoint  = carla::client::Waypoint;
  using CarlaVehicle   = carla::client::Vehicle;
  using CarlaTransform = carla::geom::Transform;

protected:

  boost::shared_ptr<router::LoopRouter> router_       = nullptr;
  boost::shared_ptr<utils::FastWaypointMap> fast_map_ = nullptr;

  boost::shared_ptr<CarlaClient> client_ = nullptr;
  boost::shared_ptr<CarlaWorld> world_   = nullptr;
  boost::shared_ptr<CarlaMap> map_       = nullptr;

  mutable ros::NodeHandle nh_;

public:

  PlanningNode(ros::NodeHandle& nh) :
    router_(boost::make_shared<router::LoopRouter>()), nh_(nh) {}

  virtual ~PlanningNode() {}

  virtual bool initialize() = 0;

protected:

  virtual boost::shared_ptr<planner::Snapshot> createSnapshot(
      const conformal_lattice_planner::TrafficSnapshot& snapshot_msg);

  /// Populate the vehicle msg through object.
  virtual void populateVehicleMsg(
      const planner::Vehicle& vehicle_obj,
      conformal_lattice_planner::Vehicle& vehicle_msg);

  /// Populate the vehicle object through msg.
  virtual void populateVehicleObj(
      const conformal_lattice_planner::Vehicle& vehicle_msg,
      planner::Vehicle& vehicle_obj);

  /// Get the carla vehicle by ID.
  boost::shared_ptr<CarlaVehicle> carlaVehicle(const size_t id) const {
    boost::shared_ptr<CarlaVehicle> vehicle =
      boost::dynamic_pointer_cast<CarlaVehicle>(world_->GetActor(id));
    if (!vehicle) {
      throw std::runtime_error(
          "PlanningNode::carlaVehicle(): "
          "Cannot get the required vehicle in the carla server.");
    }
    return vehicle;
  }

  /// Get the carla vehicle transform by the vehicle ID.
  CarlaTransform carlaVehicleTransform(const size_t id) const {
    return carlaVehicle(id)->GetTransform();
  }

  /// Get the carla waypoint a vehicle is at by its ID.
  boost::shared_ptr<CarlaWaypoint> carlaVehicleWaypoint(const size_t id) const {
    return fast_map_->waypoint(carlaVehicleTransform(id).location);
  }

}; // End class PlanningNode.

} // End namespace node.


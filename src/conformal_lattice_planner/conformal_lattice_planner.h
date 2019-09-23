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

#include <tuple>
#include <boost/optional.hpp>

#include <conformal_lattice_planner/vehicle_planner.h>
#include <conformal_lattice_planner/traffic_lattice.h>
#include <conformal_lattice_planner/loop_router.h>
#include <conformal_lattice_planner/utils.h>
#include <conformal_lattice_planner/kn_path_gen.h>

namespace planner {
namespace detail {

/**
 * \brief Vehicle keeps tracks the information of a vehicle during the planning.
 */
class Vehicle {

protected:

  using CarlaBoundingBox = carla::geom::BoundingBox;
  using CarlaTransform   = carla::geom::Transform;
  using CarlaVehicle     = carla::client::Vehicle;

protected:

  /// ID of the vehicle in the carla simulator.
  size_t id_;

  /// Bounding box of the vehicle.
  CarlaBoundingBox bounding_box_;

  /// Transform of the vehicle, left handed to be compatible with the carla simualtor.
  CarlaTransform transform_;

  /// Speed of the vehicle.
  double speed_ = 0.0;

  /// Acceleration of the vehicle, brake should be negative.
  double acceleration_ = 0.0;

public:

  /**
   * The \c acceleration_ of the object won't be filled in using \c CarlaVehicle's
   * \c GetAcceleration() API, since the vehicles are assumed to be teleported
   * instead of controlled through physical dynamics.
   */
  Vehicle(const boost::shared_ptr<const CarlaVehicle>& actor) :
    id_          (actor->GetId()),
    bounding_box_(actor->GetBoundingBox()),
    transform_   (actor->GetTransform()),
    speed_       (actor->GetVelocity().Length()),
    acceleration_(0.0) {}

  Vehicle(const size_t id,
          const CarlaBoundingBox& bounding_box,
          const CarlaTransform& transform,
          const double speed,
          const double acceleration) :
    id_          (id),
    bounding_box_(bounding_box),
    transform_   (transform),
    speed_       (speed),
    acceleration_(acceleration) {}

  const size_t id() const { return id_; }
  size_t& id() { return id_; }

  const CarlaBoundingBox boundingBox() const { return bounding_box_; }
  CarlaBoundingBox& boundingBox() { return bounding_box_; }

  const CarlaTransform transform() const { return transform_; }
  CarlaTransform& transform() { return transform_; }

  const double speed() const { return speed_; }
  double& speed() { return speed_; }

  const double acceleration() const { return acceleration_; }
  double& acceleration() { return acceleration_; }

  /**
   * \brief Update the vehicle in the simulator server.
   *
   * The function throws runtime error if the ID of input \c actor does not
   * match the ID of the object.
   *
   * The acceleration of the \c actor won't be set.
   *
   * \param[in] actor The vehicle to be updated.
   */
  void updateCarlaVehicle(const boost::shared_ptr<CarlaVehicle>& actor) const {
    if (actor->GetId() != id_)
      throw std::runtime_error("The vehicle ID does not match.");
    actor->SetTransform(transform_);
    actor->SetVelocity(transform_.GetForwardVector()*speed_);
    // No need to set the acceleration of the vehicle.
    return;
  }

  /**
   * \brief Get the vehicle ID, transform, and bounding box as a tuple.
   */
  std::tuple<size_t, CarlaTransform, CarlaBoundingBox> tuple() const {
    return std::make_tuple(id_, transform_, bounding_box_);
  }

  // TODO: Add a function to update the vehicle's transform and speed
  //       using the new acceleration and path.

}; // End class Vehicle.

/**
 * \brief Snapshot records the status of all vehicles within a micro traffic at
 *        one time instance.
 *
 * The snapshot objects uses TrafficLattice to bookkeep the relative locations
 * of the vehicles.
 */
class Snapshot {

protected:

  using CarlaWorld   = carla::client::World;
  using CarlaMap     = carla::client::Map;
  using CarlaVehicle = carla::client::Vehicle;

protected:

  /// The ego vehicle.
  Vehicle ego_;

  /// Agents in the micro traffic, i.e. all vechiles other than the ego.
  //std::vector<Vehicle> agents_;
  std::unordered_map<size_t, Vehicle> agents_;

  /// Traffic lattice which is used to keep track of the relative
  /// location among the vehicles.
  boost::shared_ptr<TrafficLattice<router::LoopRouter>> traffic_lattice_;

public:

  Snapshot(const size_t ego,
           const std::vector<size_t>& agents,
           const boost::shared_ptr<CarlaWorld>& world,
           const boost::shared_ptr<router::LoopRouter>& router);

  Snapshot(const boost::shared_ptr<const CarlaVehicle>& ego,
           const std::vector<boost::shared_ptr<const CarlaVehicle>>& agents,
           const boost::shared_ptr<CarlaMap>& map,
           const boost::shared_ptr<router::LoopRouter>& router);

  Snapshot(const Snapshot& other);

  Snapshot& operator=(const Snapshot& other);

  const Vehicle& ego() const { return ego_; }
  Vehicle& ego() { return ego_; }

  const std::unordered_map<size_t, Vehicle>& agents() const { return agents_; }
  std::unordered_map<size_t, Vehicle>& agents() { return agents_; }

  const boost::shared_ptr<const TrafficLattice<router::LoopRouter>>
    trafficLattice() const { return traffic_lattice_; }
  const boost::shared_ptr<TrafficLattice<router::LoopRouter>>
    trafficLattice() { return traffic_lattice_; }

  // TODO: Add a function to simulate the micro traffic forward. The input
  //       parameters should include the router and the path of the ego vehicle.
};

class KellyNagyPath {

protected:

  using CarlaTransform = carla::geom::Transform;
  using CarlaLocation  = carla::geom::Location;
  using CarlaRotation  = carla::geom::Rotation;

protected:

  /// The start and end transform of the path.
  /// The transform is left handed to be compatible with carla.
  CarlaTransform start_transform_;
  CarlaTransform end_transform_;

  /// The distance of the path that has already been executed.
  double progress_ = 0.0;

  /// The path object.
  NonHolonomicPath path_;

public:

  KellyNagyPath(const CarlaTransform& start, const CarlaTransform& end);

  const CarlaTransform& startTransform() const { return start_transform_; }
  CarlaTransform& startTransform() { return start_transform_; }

  const CarlaTransform& endTransform() const { return end_transform_; }
  CarlaTransform& endTransform() { return end_transform_; }

  const double progress() const { return progress_; }
  double& progress() const { return progress_; }

  const NonHolonomicPath& path() const { return path_; }
  NonHolonomicPath& path() { return path_; }

  /// Get the transform on the path at the place of \c progress_.
  /// The returned transform is left handed.
  const CarlaTransform transform() const {
    const NonHolonomicPath::State start_state = carlaTransformToPathState(start_transform_);
    const NonHolonomicPath::State progress_state = path_.evaluate(start_state, progress_);
    return pathStateToCarlaTransform(progress_state);
  }

  /// Returns true if the path has finished execution.
  const bool finished() const { return progress_ >= path_.sf; }

  const double followPath(
      const Vehicle& vehicle, const double time_step) {
    return followPath(vehicle.speed, vehicle.acceleration, time_step);
  }

  const double followPath(
      const double speed, const double acceleration, const double time_step);

protected:

  NonHolonomicPath::State carlaTransformToPathState(
      const CarlaTransform& transform) const;

  CarlaTransform pathStateToCarlaTransform(
      const NonHolonomicPath::State& state) const;

}; // End class KellyNagyPath.

/**
 * \brief Station stores the information of the end points on a path/trajectory.
 */
class Station {

protected:

  /**
   * \brief Stores a parent station of this station.
   *
   * The pair stores the cost-to-come at this station and the parent station.
   */
  using Parent = std::pair<double, boost::weak_ptr<Station>>;

  /**
   * \brief Stores a child station of this station.
   *
   * The tuple stores path to the child station, the cost of the path, and
   * the child station.
   */
  using Child = std::tuple<KellyNagyPath, double, boost::weak_ptr<Station>>;

protected:

  /// The node that the station is at on the waypoint lattice.
  boost::weak_ptr<const WaypointNode> node_;

  /// The snapshot of the traffic when the ego vehicle reaches this station.
  Snapshot snapshot_;

  /**
   * \name Parent stations of this station.
   *
   * There are at most three parent stations, from the left, same, and right lanes
   * at the back of this station.
   *
   * The \c optimal_parent is a copy of one of the three parents which has the
   * minimum cost-to-come. This is mainly used to backtrace the optimal
   * path/trajectory.
   */
  /// @{
  boost::optional<Parent> left_parent_;
  boost::optional<Parent> back_parent_;
  boost::optional<Parent> right_parent_;
  boost::optional<Parent> optimal_parent_;
  /// @}

  /**
   * \name Child stations if this station.
   *
   * There are at most three child stations, to the left, same, and right lanes
   * at the front of this station.
   */
  /// @{
  boost::optional<Child> left_child_;
  boost::optional<Child> front_child_;
  boost::optional<Child> right_child_;
  /// @}

public:

  Station(const boost::shared_ptr<const WaypointNode>& node, const Snapshot& snapshot) :
    node_(node), snapshot_(snapshot) {}

  boost::shared_ptr<const WaypointNode> node() const { return node_.lock(); }
  boost::weak_ptr<const WaypointNode> node() { return node_; }

  const size_t id() const {
    if (!node_.lock())
      throw std::runtime_error("The station is not associated with any node.");
    return node_.lock().id();
  }

  const Snapshot& snapshot() const { return snapshot_; }
  Snapshot& snapshot() { return snapshot_; }

  /// Accessors for the parent stations.
  const boost::optional<Parent>& leftParent() const { return left_parent_; }
  const boost::optional<Parent>& backParent() const { return back_parent_; }
  const boost::optional<Parent>& rightParent() const { return right_parent_; }
  const boost::optional<Parent>& optimalParent() const { return optimal_parent_; }

  /// Check whether a parent exists or not.
  const bool hasLeftParent() const { return static_cast<bool>(left_parent_); }
  const bool hasRightParent() const { return static_cast<bool>(right_parent_); }
  const bool hasBackParent() const { return static_cast<bool>(back_parent_); }
  const bool hasParent() const {
    return hasLeftParent() | hasRightParent() | hasBackParent();
  }

  /// Accessors for the child stations.
  const boost::optional<Child>& leftChild() const { return left_child_; }
  const boost::optional<Child>& rightChild() const { return right_child_; }
  const boost::optional<Child>& frontChild() const { return front_child_; }

  /// Check whether a child exists or not.
  const bool hasLeftChild() const { return static_cast<bool>(left_child_); }
  const bool hasRightChild() const { return static_cast<bool>(right_child_); }
  const bool hasFrontChild() const { return static_cast<bool>(front_child_); }
  const bool hasChild() const {
    return hasLeftChild() | hasRightChild() | hasFrontChild();
  }

  /// Update a parent station.
  /// The \c optimal_parent_ station will be updated if necessary.
  void updateLeftParent(const double cost_to_come,
                        const boost::shared_ptr<Station>& parent_station);
  void updateBackParent(const double cost_to_come,
                        const boost::shared_ptr<Station>& parent_station);
  void updateRightParent(const double cost_to_come,
                         const boost::shared_ptr<Station>& parent_station);

  /// Update a child station.
  void updateLeftChild(const KellyNagyPath& path,
                       const double stage_cost,
                       const boost::shared_ptr<Station>& child_station);
  void updateFrontChild(const KellyNagyPath& path,
                        const double stage_cost,
                        const boost::shared_ptr<Station>& child_station);
  void updateRightChild(const KellyNagyPath& path,
                        const double stage_cost,
                        const boost::shared_ptr<Station>& child_station);

protected:

  /// Update the optimal parent station, which has the minimum cost-to-come.
  void updateOptimalParent();

}; // End class Station.

} // End namespace detail.
} // End namespace planner.

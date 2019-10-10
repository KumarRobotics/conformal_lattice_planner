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
#include <utility>

#include <boost/smart_ptr.hpp>
#include <boost/core/noncopyable.hpp>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <image_transport/image_transport.h>

#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/client/Map.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/ActorList.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/Image.h>

#include <conformal_lattice_planner/EgoPlanAction.h>
#include <conformal_lattice_planner/AgentPlanAction.h>

namespace carla {

class SimulatorNode : private boost::noncopyable {

protected:

  using CarlaClient           = carla::client::Client;
  using CarlaWorld            = carla::client::World;
  using CarlaMap              = carla::client::Map;
  using CarlaBlueprintLibrary = carla::client::BlueprintLibrary;
  using CarlaActor            = carla::client::Actor;
  using CarlaActorList        = carla::client::ActorList;
  using CarlaVehicle          = carla::client::Vehicle;
  using CarlaWaypoint         = carla::client::Waypoint;
  using CarlaSensor           = carla::client::Sensor;
  using CarlaSensorData       = carla::sensor::SensorData;
  using CarlaBGRAImage        = carla::sensor::data::Image;
  using CarlaTransform        = carla::geom::Transform;

protected:

  /// FIXME: It seems redundant to store the policy speed and actual
  //         speed of a vehicle separately. It can easily lead to bugs
  //         as well since the IDs between corresponding maps may not
  //         match.

  /// Ego ID and ego policy speed pair.
  std::pair<size_t, double> ego_policy_;

  /// Ego ID and ego speed pair.
  std::pair<size_t, double> ego_speed_;

  /// Agent ID and policy speed pair.
  std::unordered_map<size_t, double> agent_policies_;

  /// Agent ID and speed pair.
  std::unordered_map<size_t, double> agent_speed_;


  /// Indicates if the ego planner action server has returned success.
  bool ego_ready_ = true;

  /// Indicates if the agents' planner action server has returned success.
  bool agents_ready_ = true;

  /// Carla client object.
  boost::shared_ptr<CarlaClient> client_ = nullptr;

  /// Carla world object.
  boost::shared_ptr<CarlaWorld> world_ = nullptr;

  // A camera following the ego vehicle to generate the third person view.
  boost::shared_ptr<CarlaSensor> following_cam_ = nullptr;

  /**
   * @name ROS interface
   *
   * For convenience, the ros interface member variable are mutable. We don't really
   * care whether these variable will be modified or not in const member functions.
   */
  /// @{
  /// ROS node handle.
  mutable ros::NodeHandle nh_;

  /// For publishing tf.
  mutable tf2_ros::TransformBroadcaster tf_broadcaster_;

  /// Publish the map of the town.
  mutable ros::Publisher map_pub_;

  /// Publish traffice relatated stuff.
  mutable ros::Publisher traffic_pub_;

  /// ROS image transport.
  mutable image_transport::ImageTransport img_transport_;

  /// Publishing images for the following camera.
  mutable image_transport::Publisher following_img_pub_;

  /// The actionlib client for the ego vehicle planner.
  mutable actionlib::SimpleActionClient<
    conformal_lattice_planner::EgoPlanAction> ego_client_;

  /// The actionlib client for the planner controlling all agent vehicles.
  mutable actionlib::SimpleActionClient<
    conformal_lattice_planner::AgentPlanAction> agents_client_;
  /// @}

public:

  SimulatorNode(ros::NodeHandle& nh) :
    nh_(nh),
    img_transport_(nh),
    ego_client_(nh_, "ego_plan", false),
    agents_client_(nh_, "agents_plan", false) {}

  virtual ~SimulatorNode() {}

  /// Initialize the simulator ROS node.
  virtual bool initialize();

protected:

  /// Spawn the vehicles.
  virtual void spawnVehicles() = 0;

  virtual boost::optional<size_t> spawnEgoVehicle(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double policy_speed,
      const bool noisy_speed = true);

  virtual boost::optional<size_t> spawnAgentVehicle(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double policy_speed,
      const bool noisy_speed = true);

  virtual void spawnCamera();

  /// Simulate the world forward by one time step.
  virtual void tickWorld() {
    world_->Tick();
    publishTraffic();
    sendEgoGoal();
    sendAgentsGoal();
  }

  /// Publish the following image.
  void publishImage(const boost::shared_ptr<CarlaSensorData>& data) const;

  /**
   * @name Accessors to carla vehicles.
   */
  /// @{
  /// Get the ego vehicle actor.
  boost::shared_ptr<CarlaVehicle> egoVehicle();
  /// Get the ego vehicle actor.
  boost::shared_ptr<const CarlaVehicle> egoVehicle() const;

  /// Get the required agent vehicle.
  boost::shared_ptr<CarlaVehicle> agentVehicle(const size_t agent);
  /// Get the required agent vehicle.
  boost::shared_ptr<const CarlaVehicle> agentVehicle(const size_t agent) const;

  /// Get all agent vehicle actors.
  std::vector<boost::shared_ptr<CarlaVehicle>> agentVehicles();
  /// Get all agent vehicle actors.
  std::vector<boost::shared_ptr<const CarlaVehicle>> agentVehicles() const;

  /// Get all vehicle actors.
  /// The order of the vehicles in the sequenceis not guaranteed.
  std::vector<boost::shared_ptr<CarlaVehicle>> vehicles();
  /// Get all vehicle actors.
  /// The order of the vehicles in the sequenceis not guaranteed.
  std::vector<boost::shared_ptr<const CarlaVehicle>> vehicles() const;
  /// @}

  /**
   * @name Publishing functions
   */
  /// @{
  /// Publish the map visualization markers.
  virtual void publishMap() const;

  /// Publish the vehicle visualization markers.
  virtual void publishTraffic() const;
  /// @}

  /**
   * @name Ego actionlib client callbacks
   */
  /// @{
  /// Send the goal for the ego.
  virtual void sendEgoGoal();

  /// Done callback for \c ego_client_.
  virtual void egoPlanDoneCallback(
      const actionlib::SimpleClientGoalState& state,
      const conformal_lattice_planner::EgoPlanResultConstPtr& result);

  /// Action callback for \c ego_client_.
  virtual void egoPlanActiveCallback() {}

  /// Feedback callback for \c ego_client_.
  virtual void egoPlanFeedbackCallback(
      const conformal_lattice_planner::EgoPlanFeedbackConstPtr& feedback) {}
  /// @}

  /**
   * @name Agent actionlib client callbacks
   */
  /// @{
  /// Send the goal for the agents.
  virtual void sendAgentsGoal();

  /// Done callback for agents_client.
  virtual void agentsPlanDoneCallback(
      const actionlib::SimpleClientGoalState& state,
      const conformal_lattice_planner::AgentPlanResultConstPtr& result);

  /// Action callback for agents_client.
  virtual void agentsPlanActiveCallback() {}

  /// Feedback callback for agents_client.
  virtual void agentsPlanFeedbackCallback(
      const conformal_lattice_planner::AgentPlanFeedbackConstPtr& feedback) {}
  /// @}

}; // End class SimulatorNode.

} // End namespace carla.

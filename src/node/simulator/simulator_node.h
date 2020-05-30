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

#include <vector>
#include <utility>
#include <unordered_map>

#include <boost/smart_ptr.hpp>
#include <boost/core/noncopyable.hpp>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
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

#include <router/loop_router/loop_router.h>
#include <planner/common/fast_waypoint_map.h>
#include <planner/common/vehicle.h>

#include <conformal_lattice_planner/EgoPlanAction.h>
#include <conformal_lattice_planner/AgentPlanAction.h>

namespace node {

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

  /// The actual simulation time starting from 0.
  double simulation_time_ = 0.0;

  /// The ego vehicle.
  planner::Vehicle ego_;

  /// Agent vehicles.
  std::unordered_map<size_t, planner::Vehicle> agents_;

  /// Indicates if the ego planner action server has returned success.
  bool ego_ready_ = true;

  /// Indicates if the agents' planner action server has returned success.
  bool agents_ready_ = true;

  /// Loop router, the router is predefined on Town04.
  boost::shared_ptr<router::LoopRouter> loop_router_ = nullptr;

  /// Carla client object.
  boost::shared_ptr<CarlaClient> client_ = nullptr;

  /// Carla world object.
  boost::shared_ptr<CarlaWorld> world_ = nullptr;

  /// Carla map.
  boost::shared_ptr<CarlaMap> map_ = nullptr;

  /// Fast waypoint map.
  boost::shared_ptr<utils::FastWaypointMap> fast_map_ = nullptr;

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

  /// The service server to tell the simulation time.
  mutable ros::ServiceServer sim_time_server_;
  /// @}

public:

  SimulatorNode(ros::NodeHandle& nh) :
    loop_router_(new router::LoopRouter),
    nh_(nh),
    img_transport_(nh),
    ego_client_(nh_, "ego_plan", false),
    agents_client_(nh_, "agents_plan", false),
    sim_time_server_(nh_.advertiseService("simulation_time", &SimulatorNode::simTimeCallback, this)){}

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
    updateSimTime();
    publishTraffic();
    sendEgoGoal();
    sendAgentsGoal();
    return;
  }

  /// Update the simulation time based on the settings for the carla server.
  virtual void updateSimTime() {
    double fixed_delta_seconds = 0.05;
    nh_.param<double>("fixed_delta_seconds", fixed_delta_seconds, 0.05);
    simulation_time_ += fixed_delta_seconds;
    return;
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

  /// Populate the vehicle msg through object.
  virtual void populateVehicleMsg(
      const planner::Vehicle& vehicle_obj,
      conformal_lattice_planner::Vehicle& vehicle_msg);

  /// Populate the vehicle object through msg.
  virtual void populateVehicleObj(
      const conformal_lattice_planner::Vehicle& vehicle_msg,
      planner::Vehicle& vehicle_obj);

  /// Populate the vehicle object through actor.
  virtual void populateVehicleObj(
      const boost::shared_ptr<const CarlaVehicle>& vehicle_actor,
      planner::Vehicle& vehicle_obj);

  /// Callback for the simulation time server.
  virtual bool simTimeCallback(
      std_srvs::Trigger::Request& req,
      std_srvs::Trigger::Response& res) {

    res.success = true;
    res.message = std::to_string(simulation_time_);
    return true;
  }

}; // End class SimulatorNode.

} // End namespace node.

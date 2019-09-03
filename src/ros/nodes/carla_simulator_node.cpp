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

#include <chrono>
#include <limits>
#include <vector>
#include <array>
#include <unordered_set>
#include <boost/core/noncopyable.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <image_transport/image_transport.h>

#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/client/Sensor.h>
//#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
//#include <carla/image/ImageIO.h>
//#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

#include <conformal_lattice_planner/EgoPlanAction.h>
#include <conformal_lattice_planner/AgentPlanAction.h>
#include <conformal_lattice_planner/Policy.h>
#include <conformal_lattice_planner/waypoint_lattice.h>

using namespace std;
namespace bst = boost;
namespace cc = carla::client;
namespace cg = carla::geom;
namespace crpc = carla::rpc;
namespace cs = carla::sensor;
namespace csd = carla::sensor::data;
namespace clp = conformal_lattice_planner;

namespace carla {

/// Prototypes for creating visualization msgs.
visualization_msgs::MarkerPtr createWaypointMsg(
    const vector<carla::SharedPtr<cc::Waypoint>>&);
visualization_msgs::MarkerPtr createJunctionMsg(
    const cc::Map::TopologyList&);
visualization_msgs::MarkerPtr createVehicleMarkerMsg(
    const carla::SharedPtr<cc::Actor>&);
geometry_msgs::TransformStampedPtr createVehicleTransformMsg(
    const carla::SharedPtr<cc::Actor>&, const std::string&);
sensor_msgs::ImagePtr createImageMsg(
    const carla::SharedPtr<csd::Image>&);
visualization_msgs::MarkerArrayPtr createTrafficLatticeMsg(
    const bst::shared_ptr<const planner::WaypointNode>&);

class CarlaSimulatorNode : private bst::noncopyable {

public:

  using Ptr = bst::shared_ptr<CarlaSimulatorNode>;
  using ConstPtr = bst::shared_ptr<const CarlaSimulatorNode>;

private:


  /// Policy of the ego vehicle (ego_id->ego_desired_speed).
  std::pair<size_t, double> ego_policy_;

  /// Policies of agents.
  std::unordered_map<size_t, double> agent_policies_;

  /// Used to keep track of the traffic around the ego vehicle,
  /// so that we can add new agents and remove agents that are far
  /// from the ego vehicle efficiently.
  bst::shared_ptr<planner::WaypointLattice> traffic_lattice_;

  /// Indicates if the ego planner action server has returned success.
  bool ego_ready_ = true;

  /// Indicates if the agents' planner action server has returned success.
  bool agents_ready_ = true;

  /// Carla interface.
  SharedPtr<cc::World> world_ = nullptr;
  SharedPtr<cc::Client> client_ = nullptr;
  SharedPtr<cc::Sensor> following_cam_ = nullptr;

  /// ROS interface.
  mutable ros::NodeHandle nh_;
  mutable tf2_ros::TransformBroadcaster tf_broadcaster_;
  mutable ros::Publisher map_pub_;
  mutable ros::Publisher ego_marker_pub_;
  mutable ros::Publisher agents_marker_pub_;
  mutable ros::Publisher traffic_lattice_pub_;

  mutable image_transport::ImageTransport img_transport_;
  mutable image_transport::Publisher following_img_pub_;

  mutable actionlib::SimpleActionClient<clp::EgoPlanAction> ego_client_;
  mutable actionlib::SimpleActionClient<clp::AgentPlanAction> agents_client_;

public:

  CarlaSimulatorNode(ros::NodeHandle nh) :
    nh_(nh),
    img_transport_(nh_),
    ego_client_(nh_, "ego_plan", false),
    agents_client_(nh_, "agents_plan", false) {}

  /// Initialize the simulator ros node.
  bool initialize();

private:

  /// Spawn the vehicles.
  //cg::Transform spawnEgo(const bool no_rendering_mode = true);
  void spawnVehicles(const bool no_rendering_mode = true);
  void startVehicles();

  /// Manage agents around the ego vehicle.
  /// Add agents if there is empty space around the ego.
  /// Delete agents that are too far from the ego.
  void manageAgents();

  /// Publish the map visualization markers.
  void publishMap() const;

  /// Publish the vehicle visualization markers.
  void publishTraffic() const;

  /// Publish the following image.
  void publishImage(const SharedPtr<cs::SensorData>& data) const;

  /**
   * @name Ego action callbacks
   */
  /// @{
  /// Send the goal for the ego.
  void sendEgoGoal();

  /// Done callback for ego_client.
  void egoPlanDoneCallback(
      const actionlib::SimpleClientGoalState& state,
      const clp::EgoPlanResultConstPtr& result);

  /// Action callback for ego_client.
  void egoPlanActiveCallback() {}

  /// Feedback callback for ego_client.
  void egoPlanFeedbackCallback(
      const clp::EgoPlanFeedbackConstPtr& feedback) {}
  /// @}

  /**
   * @name Agent plan callbacks
   */
  /// @{
  /// Send the goal for the agents.
  void sendAgentsGoal();

  /// Done callback for agents_client.
  void agentsPlanDoneCallback(
      const actionlib::SimpleClientGoalState& state,
      const clp::AgentPlanResultConstPtr& result);

  /// Action callback for agents_client.
  void agentsPlanActiveCallback() {}

  /// Feedback callback for agents_client.
  void agentsPlanFeedbackCallback(
      const clp::AgentPlanFeedbackConstPtr& feedback) {}
  /// @}

}; // End class CarlaSimulatorNode.

bool CarlaSimulatorNode::initialize() {

  bool all_param_exist = true;

  // Create publishers.
  map_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("town_map", 1, true);
  ego_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("ego_object", 1, true);
  agents_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("agent_objects", 1, true);
  traffic_lattice_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("traffic_lattice", 1, true);

  // Get the world.
  string host = "localhost";
  int port = 2000;
  all_param_exist &= nh_.param<std::string>("host", host, "localhost");
  all_param_exist &= nh_.param<int>("port", port, 2000);

  ROS_INFO_NAMED("carla_simulator", "connect to the server.");
  client_ = bst::make_shared<cc::Client>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  world_ = bst::make_shared<cc::World>(client_->GetWorld());
  ros::Duration(2.0).sleep();

  // Applying the world settings.
  double fixed_delta_seconds = 0.05;
  bool no_rendering_mode = true;
  bool synchronous_mode = true;
  all_param_exist &= nh_.param<double>("fixed_delta_seconds", fixed_delta_seconds, 0.05);
  all_param_exist &= nh_.param<bool>("no_rendering_mode", no_rendering_mode, true);
  all_param_exist &= nh_.param<bool>("synchronous_mode", synchronous_mode, true);

  ROS_INFO_NAMED("carla_simulator", "apply world settings.");
  crpc::EpisodeSettings settings = world_->GetSettings();
  if (settings.fixed_delta_seconds) {
    ROS_DEBUG_NAMED("carla_simulator",
        "old settings: fixed_delta_seconds:N/A no_rendering_mode:%d synchronous_mode:%d",
        settings.no_rendering_mode, settings.synchronous_mode);
  } else {
    ROS_DEBUG_NAMED("carla_simulator",
        "old settings: fixed_delta_seconds:%f no_rendering_mode:%d synchronous_mode:%d",
        *(settings.fixed_delta_seconds), settings.no_rendering_mode, settings.synchronous_mode);
  }
  settings.fixed_delta_seconds = fixed_delta_seconds;
  settings.no_rendering_mode = no_rendering_mode;
  settings.synchronous_mode = synchronous_mode;
  world_->ApplySettings(settings);
  ros::Duration(2.0).sleep();

  settings = world_->GetSettings();
  ROS_INFO_NAMED("carla_simulator",
      "new settings: fixed_delta_seconds:%f no_rendering_mode:%d synchronous_mode:%d",
      *(settings.fixed_delta_seconds), settings.no_rendering_mode, settings.synchronous_mode);

  // Publish the map.
  ROS_INFO_NAMED("carla_simulator", "publish global map.");
  publishMap();

  // Initialize the ego vehicle.
  ROS_INFO_NAMED("carla_simulator", "spawn the vehicles.");
  spawnVehicles(no_rendering_mode);
  world_->Tick();

  // Start the vehicles.
  ROS_INFO_NAMED("carla_simulator", "start the vehicles.");
  startVehicles();
  world_->Tick();

  // Publish the ego vehicle marker.
  ROS_INFO_NAMED("carla_simulator", "publish ego and agents.");
  publishTraffic();

  // Wait for the planner servers.
  ROS_INFO_NAMED("carla_simulator", "waiting for action servers.");
  ego_client_.waitForServer(ros::Duration(5.0));

  // Send out the first goal of ego.
  ROS_INFO_NAMED("carla_simulator", "send the first goals to action servers");
  sendEgoGoal();

  ROS_INFO_NAMED("carla_simulator", "initialization finishes.");
  return all_param_exist;
}

void CarlaSimulatorNode::spawnVehicles(const bool no_rendering_mode) {

  bool all_param_exist = true;

  // \c blueprint_library has all actor models we can create.
  SharedPtr<cc::BlueprintLibrary> blueprint_library =
    world_->GetBlueprintLibrary();

  // Load blueprint library.
  std::string vehicle_name = "vehicle.audi.tt";
  all_param_exist &= nh_.param<std::string>(
      "vehicle_name", vehicle_name, "vehicle.audi.tt");
  auto ego_blueprint = blueprint_library->at(vehicle_name);

  // Load ego vehicle initial position.
  array<double, 3> ego_pt{0, 0, 0};
  all_param_exist &= nh_.param<double>("ego_initial_x", ego_pt[0], 0.0);
  all_param_exist &= nh_.param<double>("ego_initial_y", ego_pt[1], 0.0);
  all_param_exist &= nh_.param<double>("ego_initial_z", ego_pt[2], 0.0);

  vector<cg::Transform> spawn_points = world_->GetMap()->GetRecommendedSpawnPoints();
  cg::Transform ego_transform;
  float min_distance_sq = numeric_limits<float>::max();
  // Find the available spawn point cloest to the given ego initial state.
  for (const auto pt : spawn_points) {
    const float x_diff = pt.location.x - ego_pt[0];
    const float y_diff = pt.location.y - ego_pt[1];
    const float z_diff = pt.location.z - ego_pt[2];
    const float distance_sq = x_diff*x_diff + y_diff*y_diff + z_diff*z_diff;
    if (distance_sq < min_distance_sq) {
      ego_transform = pt;
      min_distance_sq = distance_sq;
    }
  }
  ROS_INFO_NAMED("carla_simulator", "Initial Ego x:%f y:%f z:%f r:%f p:%f y:%f",
      ego_transform.location.x, ego_transform.location.y, ego_transform.location.z,
      ego_transform.rotation.roll, ego_transform.rotation.pitch, ego_transform.rotation.yaw);

  // Spawn the ego vehicle.
  SharedPtr<cc::Actor> ego_actor = world_->SpawnActor(ego_blueprint, ego_transform);

  ego_policy_.first = ego_actor->GetId();
  ego_policy_.second = 29.0576; // 65mph

  // Spawn a camera following the ego vehicle.
  if (!no_rendering_mode) {
    auto camera_blueprint = blueprint_library->at("sensor.camera.rgb");
    camera_blueprint.SetAttribute("sensor_tick", "0.2");
    camera_blueprint.SetAttribute("image_size_x", "320");
    camera_blueprint.SetAttribute("image_size_y", "240");
    camera_blueprint.SetAttribute("fov", "120");
    cg::Transform camera_transform = cg::Transform{
      cg::Location{-5.5f, 0.0f, 2.8f},   // x, y, z.
      cg::Rotation{-15.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    SharedPtr<cc::Actor> cam_actor = world_->SpawnActor(
        camera_blueprint, camera_transform, ego_actor.get());
    following_cam_ = boost::static_pointer_cast<cc::Sensor>(cam_actor);
    following_cam_->Listen(bst::bind(&CarlaSimulatorNode::publishImage, this, _1));

    // Create the image publisher for the following camera.
    following_img_pub_ = img_transport_.advertise("third_person_view", 5, true);
  }

  // Initialize the traffic lattice.
  SharedPtr<cc::Waypoint> ego_waypoint =
    world_->GetMap()->GetWaypoint(ego_transform.location);
  traffic_lattice_ = bst::make_shared<planner::WaypointLattice>(ego_waypoint, 200.0, 4.0);

  // TODO: Spawn target vehicles.

  return;
}

void CarlaSimulatorNode::startVehicles() {

  SharedPtr<cc::Actor> ego_actor = world_->GetActor(ego_policy_.first);
  ego_actor->SetVelocity(ego_actor->GetTransform().GetForwardVector()*15.0);

  // TODO: Set the velocity for the agents.
  return;
}

void CarlaSimulatorNode::publishMap() const {

  visualization_msgs::MarkerPtr waypoints_msg =
    createWaypointMsg(world_->GetMap()->GenerateWaypoints(5.0));
  visualization_msgs::MarkerPtr junctions_msg =
    createJunctionMsg(world_->GetMap()->GetTopology());

  visualization_msgs::MarkerArrayPtr map_msg(
      new visualization_msgs::MarkerArray);
  map_msg->markers.push_back(*waypoints_msg);
  map_msg->markers.push_back(*junctions_msg);

  map_pub_.publish(map_msg);
  return;
}

void CarlaSimulatorNode::publishTraffic() const {

  // Publish the traffic lattice.
  traffic_lattice_pub_.publish(createTrafficLatticeMsg(traffic_lattice_->latticeEntry()));

  // Publish the ego marker and tf.
  ego_marker_pub_.publish(createVehicleMarkerMsg(world_->GetActor(ego_policy_.first)));
  tf_broadcaster_.sendTransform(*(createVehicleTransformMsg(world_->GetActor(ego_policy_.first), "ego")));

  // TODO: Publish the agents' markers.

  return;
}

void CarlaSimulatorNode::publishImage(const SharedPtr<cs::SensorData>& data) const {

  const SharedPtr<csd::Image> img = bst::static_pointer_cast<csd::Image>(data);
  following_img_pub_.publish(createImageMsg(img));

  return;
}

void CarlaSimulatorNode::sendEgoGoal() {

  clp::EgoPlanGoal goal;
  goal.ego_policy.id = ego_policy_.first;
  goal.ego_policy.desired_speed = ego_policy_.second;

  for (const auto agent_policy : agent_policies_) {
    goal.agent_policies.push_back(clp::Policy());
    goal.agent_policies.back().id = agent_policy.first;
    goal.agent_policies.back().desired_speed = agent_policy.second;
  }

  ego_client_.sendGoal(
      goal,
      bst::bind(&CarlaSimulatorNode::egoPlanDoneCallback, this, _1, _2),
      bst::bind(&CarlaSimulatorNode::egoPlanActiveCallback, this),
      bst::bind(&CarlaSimulatorNode::egoPlanFeedbackCallback, this, _1));

  ego_ready_ = false;

  return;
}

void CarlaSimulatorNode::egoPlanDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const clp::EgoPlanResultConstPtr& result) {

  ROS_INFO_NAMED("carla_simulator", "egoPlanDoneCallback().");
  ROS_INFO_NAMED("carla_simulator", "tick world.");
  world_->Tick();
  publishTraffic();
  sendEgoGoal();

  ego_ready_ = true;
  return;
}

using CarlaSimulatorNodePtr = CarlaSimulatorNode::Ptr;
using CarlaSimulatorNodeConstPtr = CarlaSimulatorNode::ConstPtr;
} // End namespace carla.


int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  if(ros::console::set_logger_level(
        ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  carla::CarlaSimulatorNodePtr carla_sim =
    bst::make_shared<carla::CarlaSimulatorNode>(nh);
  if (!carla_sim->initialize()) {
    ROS_ERROR("Cannot initialize the CARLA simulator.");
  }

  ros::spin();
  return 0;
}


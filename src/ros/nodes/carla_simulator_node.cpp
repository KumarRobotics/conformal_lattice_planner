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

#include <cmath>
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
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/sensor/data/Image.h>

#include <conformal_lattice_planner/EgoPlanAction.h>
#include <conformal_lattice_planner/AgentPlanAction.h>
#include <conformal_lattice_planner/Policy.h>
#include <conformal_lattice_planner/loop_router.h>
#include <conformal_lattice_planner/traffic_manager.h>
#include <ros/convert_to_visualization_msgs.h>

using namespace std;
using namespace planner;
using namespace router;

namespace bst = boost;
namespace cc = carla::client;
namespace cg = carla::geom;
namespace crpc = carla::rpc;
namespace cs = carla::sensor;
namespace cr = carla::road;
namespace csd = carla::sensor::data;
namespace clp = conformal_lattice_planner;

namespace carla {

class CarlaSimulatorNode : private bst::noncopyable {

public:

  using Ptr = bst::shared_ptr<CarlaSimulatorNode>;
  using ConstPtr = bst::shared_ptr<const CarlaSimulatorNode>;

private:

  /// Policy of the ego vehicle (ego_id->ego_desired_speed).
  std::pair<size_t, double> ego_policy_;

  /// Policies of agents.
  std::unordered_map<size_t, double> agent_policies_;

  /// Indicates if the ego planner action server has returned success.
  bool ego_ready_ = true;

  /// Indicates if the agents' planner action server has returned success.
  bool agents_ready_ = true;

  /// Loop router, the router is predefined on Town04.
  boost::shared_ptr<LoopRouter> loop_router_;

  /// Traffic lattice.
  boost::shared_ptr<TrafficManager<LoopRouter>> traffic_manager_;

  /// Carla interface.
  SharedPtr<cc::World> world_ = nullptr;
  SharedPtr<cc::Client> client_ = nullptr;
  SharedPtr<cc::Sensor> following_cam_ = nullptr;

  /// ROS interface.
  mutable ros::NodeHandle nh_;
  mutable tf2_ros::TransformBroadcaster tf_broadcaster_;
  mutable ros::Publisher map_pub_;
  //mutable ros::Publisher ego_marker_pub_;
  //mutable ros::Publisher agents_marker_pub_;
  mutable ros::Publisher traffic_pub_;
  //mutable ros::Publisher road_ids_pub_;
  //mutable ros::Publisher vehicle_ids_pub_;

  mutable image_transport::ImageTransport img_transport_;
  mutable image_transport::Publisher following_img_pub_;

  mutable actionlib::SimpleActionClient<clp::EgoPlanAction> ego_client_;
  mutable actionlib::SimpleActionClient<clp::AgentPlanAction> agents_client_;

public:

  CarlaSimulatorNode(ros::NodeHandle nh) :
    loop_router_(new LoopRouter),
    nh_(nh),
    img_transport_(nh_),
    ego_client_(nh_, "ego_plan", false),
    agents_client_(nh_, "agents_plan", false) {}

  /// Initialize the simulator ros node.
  bool initialize();

private:

  /// Spawn the vehicles.
  void spawnVehicles(const bool no_rendering_mode = true);
  void startVehicles();

  /// Manager (add/delete) the vehicles in the simulation.
  void manageTraffic();

  /**
   * @name Get Carla vehicle actors.
   */
  /// @{
  /// Get the ego vehicle actor.
  SharedPtr<cc::Vehicle> egoVehicle();
  SharedPtr<const cc::Vehicle> egoVehicle() const;

  /// Get the required agent vehicle.
  SharedPtr<cc::Vehicle> agentVehicle(const size_t agent);
  SharedPtr<const cc::Vehicle> agentVehicle(const size_t agent) const;

  /// Get all agent vehicle actors.
  vector<SharedPtr<cc::Vehicle>> agentVehicles();
  vector<SharedPtr<const cc::Vehicle>> agentVehicles() const;

  /// Get all vehicle actors.
  /// The order of the vehicles in the sequenceis not guaranteed.
  vector<SharedPtr<cc::Vehicle>> vehicles();
  vector<SharedPtr<const cc::Vehicle>> vehicles() const;
  /// @}

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
  traffic_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("traffic", 1, true);

  // Get the world.
  string host = "localhost";
  int port = 2000;
  all_param_exist &= nh_.param<std::string>("host", host, "localhost");
  all_param_exist &= nh_.param<int>("port", port, 2000);

  ROS_INFO_NAMED("carla_simulator", "connect to the server.");
  client_ = bst::make_shared<cc::Client>(host, port);
  client_->SetTimeout(std::chrono::seconds(10));
  world_ = bst::make_shared<cc::World>(client_->GetWorld());
  ros::Duration(1.0).sleep();

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
  ros::Duration(1.0).sleep();

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

  //// Wait for the planner servers.
  //ROS_INFO_NAMED("carla_simulator", "waiting for action servers.");
  //ego_client_.waitForServer(ros::Duration(5.0));
  //agents_client_.waitForServer(ros::Duration(5.0));

  //// Send out the first goal of ego.
  //ROS_INFO_NAMED("carla_simulator", "send the first goals to action servers");
  //sendEgoGoal();
  //sendAgentsGoal();

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

  // The start position and waypoint.
  array<double, 3> start_pt{0, 0, 0};
  //all_param_exist &= nh_.param<double>("start_initial_x", start_pt[0], 0.0);
  //all_param_exist &= nh_.param<double>("start_initial_y", start_pt[1], 0.0);
  //all_param_exist &= nh_.param<double>("start_initial_z", start_pt[2], 0.0);

  vector<cg::Transform> spawn_points = world_->GetMap()->GetRecommendedSpawnPoints();
  cg::Transform start_transform;
  double min_distance_sq = numeric_limits<double>::max();
  // Find the available spawn point cloest to the start point.
  for (const auto& pt : spawn_points) {
    const double x_diff = pt.location.x - start_pt[0];
    const double y_diff = pt.location.y - start_pt[1];
    const double z_diff = pt.location.z - start_pt[2];
    const double distance_sq = x_diff*x_diff + y_diff*y_diff + z_diff*z_diff;
    if (distance_sq < min_distance_sq) {
      start_transform = pt;
      min_distance_sq = distance_sq;
    }
  }
  ROS_INFO_NAMED("carla_simulator", "Start waypoint transform\nx:%f y:%f z:%f",
      start_transform.location.x, start_transform.location.y, start_transform.location.z);

  SharedPtr<cc::Waypoint> start_waypoint =
    world_->GetMap()->GetWaypoint(start_transform.location);

  // Initialize the traffic manager.
  traffic_manager_ = boost::make_shared<TrafficManager<LoopRouter>>(
      start_waypoint, 150.0, loop_router_);

  // Spawn the ego vehicle.
  // The ego vehicle is at 50m on the lattice, and there is an 100m buffer
  // in the front of the ego vehicle.
  SharedPtr<const cc::Waypoint> ego_waypoint =
    traffic_manager_->front(start_waypoint, 50.0)->waypoint();
  if (!ego_waypoint) throw std::runtime_error("Cannot create ego vehicle on the lattice.");

  ROS_INFO_NAMED("carla_simulator", "Ego vehicle initial transform\nx:%f y:%f z:%f",
      ego_waypoint->GetTransform().location.x,
      ego_waypoint->GetTransform().location.y,
      ego_waypoint->GetTransform().location.z);

  SharedPtr<cc::Actor> ego_actor =
    world_->SpawnActor(ego_blueprint, ego_waypoint->GetTransform());
  ego_policy_.first = ego_actor->GetId();
  ego_policy_.second = 25.0;

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

  // TODO: Spawn agent vehicles.
  {
    SharedPtr<const cc::Waypoint> waypoint0 = ego_waypoint;
    SharedPtr<const cc::Waypoint> waypoint1 = ego_waypoint->GetRight();
    SharedPtr<const cc::Waypoint> waypoint2 = waypoint1->GetRight();
    SharedPtr<const cc::Waypoint> waypoint3 = waypoint2->GetRight();

    SharedPtr<const cc::Waypoint> agent_waypoint = nullptr;
    cg::Transform agent_transform;
    SharedPtr<cc::Actor> agent_actor = nullptr;

    // First lane, 80m, 20m/s.
    agent_waypoint = traffic_manager_->front(waypoint0, 80.0)->waypoint();
    agent_transform = agent_waypoint->GetTransform();
    agent_transform.location.z += 1.2;
    agent_actor = world_->SpawnActor(ego_blueprint, agent_transform);
    agent_policies_[agent_actor->GetId()] = 20.0;

    // Second lane, 60m, 20m/s
    agent_waypoint = traffic_manager_->front(waypoint1, 60.0)->waypoint();
    agent_transform = agent_waypoint->GetTransform();
    agent_transform.location.z += 1.2;
    agent_actor = world_->SpawnActor(ego_blueprint, agent_transform);
    agent_policies_[agent_actor->GetId()] = 20.0;
  }

  return;
}

void CarlaSimulatorNode::startVehicles() {

  SharedPtr<cc::Actor> ego_actor = world_->GetActor(ego_policy_.first);
  ego_actor->SetVelocity(ego_actor->GetTransform().GetForwardVector()*15.0);

  // TODO: Set the velocity for the agents.
  for (const auto& agent : agent_policies_) {
    SharedPtr<cc::Actor> actor = world_->GetActor(agent.first);
    actor->SetVelocity(actor->GetTransform().GetForwardVector()*15.0);
  }

  return;
}

void CarlaSimulatorNode::manageTraffic() {

  // Cache the last ego transform.
  // This is used to compute how much the ego vehicle has moved forward.
  static cg::Transform last_ego_transform = egoVehicle()->GetTransform();

  // Check how much the ego vehicle has moved.
  // FIXME: Is there a better way other than using the Euclidean distance?
  const cg::Transform ego_transform = egoVehicle()->GetTransform();
  const double shift_distance = ego_transform.location.Distance(last_ego_transform.location);
  last_ego_transform = ego_transform;

  // Get all vehicles for the server.
  vector<SharedPtr<const cc::Vehicle>> vehicles =
    const_cast<const CarlaSimulatorNode*>(this)->vehicles();

  // Update the traffic on the lattice.
  unordered_set<size_t> disappear_vehicles;
  traffic_manager_->moveTrafficForward(vehicles, shift_distance, disappear_vehicles);

  // Remove the vehicles that disappear.
  for (const size_t id : disappear_vehicles) {
    if (id == ego_policy_.first)
      throw std::runtime_error("The ego vehicle is removed in the simulation.");

    // Remove the vehicle from the carla server.
    SharedPtr<cc::Vehicle> vehicle = agentVehicle(id);
    if (!vehicle->Destroy())
      throw std::runtime_error("Cannot destroy an agent.");

    // Remove the vehicle from the class.
    agent_policies_.erase(id);
  }

  // Spawn more vehicles if the number of agents around the ego vehicle
  // does not meet the requirement.
  // At most one vehicle is spawned every time this function is called.
  if (agent_policies_.size() < 8) {

    std::string vehicle_name = "vehicle.audi.tt";
    SharedPtr<cc::BlueprintLibrary> blueprint_library =
      world_->GetBlueprintLibrary();
    auto blueprint = blueprint_library->at(vehicle_name);

    const double min_distance = 20.0;
    boost::optional<pair<double, SharedPtr<const cc::Waypoint>>> front =
      traffic_manager_->frontSpawnWaypoint(min_distance);
    boost::optional<pair<double, SharedPtr<const cc::Waypoint>>> back =
      traffic_manager_->backSpawnWaypoint(min_distance);

    const double front_distance = front ? front->first : 0.0;
    const double back_distance = back ? back->first : 0.0;

    // Waypoint to spawn the new vehicle.
    SharedPtr<const cc::Waypoint> spawn_waypoint = nullptr;
    double start_speed = 0.0;

    if (front_distance>=back_distance && front_distance>=min_distance) {
      // Spawn a new vehicle at the front of the lattice.
      SharedPtr<const cc::Waypoint> waypoint = front->second;
      spawn_waypoint = traffic_manager_->back(waypoint, 4.0)->waypoint();
      start_speed = 20.0;
    }

    if (front_distance<=back_distance && back_distance>=min_distance) {
      // Spawn a new vehicle at the back of the lattice.
      SharedPtr<const cc::Waypoint> waypoint = back->second;
      spawn_waypoint = traffic_manager_->front(waypoint, 4.0)->waypoint();
      start_speed = 22.0;
    }

    SharedPtr<cc::Actor> agent_actor =
      world_->SpawnActor(blueprint, spawn_waypoint->GetTransform());
    agent_policies_[agent_actor->GetId()] = start_speed;
    agent_actor->SetVelocity(
        spawn_waypoint->GetTransform().GetForwardVector()*start_speed);
  }

  return;
}

SharedPtr<cc::Vehicle> CarlaSimulatorNode::egoVehicle() {
  return boost::static_pointer_cast<cc::Vehicle>(
      world_->GetActor(ego_policy_.first));
}

SharedPtr<const cc::Vehicle> CarlaSimulatorNode::egoVehicle() const {
  return boost::static_pointer_cast<cc::Vehicle>(
      world_->GetActor(ego_policy_.first));
}

SharedPtr<cc::Vehicle> CarlaSimulatorNode::agentVehicle(const size_t agent) {
  if (agent_policies_.count(agent) == 0) return nullptr;
  return boost::static_pointer_cast<cc::Vehicle>(world_->GetActor(agent));
}

SharedPtr<const cc::Vehicle> CarlaSimulatorNode::agentVehicle(const size_t agent) const {
  if (agent_policies_.count(agent) == 0) return nullptr;
  return boost::static_pointer_cast<cc::Vehicle>(world_->GetActor(agent));
}

vector<SharedPtr<cc::Vehicle>> CarlaSimulatorNode::agentVehicles() {
  vector<SharedPtr<cc::Vehicle>> vehicles;
  for (const auto& agent : agent_policies_)
    vehicles.push_back(agentVehicle(agent.first));
  return vehicles;
}

vector<SharedPtr<const cc::Vehicle>> CarlaSimulatorNode::agentVehicles() const {
  vector<SharedPtr<const cc::Vehicle>> vehicles;
  for (const auto& agent : agent_policies_)
    vehicles.push_back(agentVehicle(agent.first));
  return vehicles;
}

vector<SharedPtr<cc::Vehicle>> CarlaSimulatorNode::vehicles() {
  vector<SharedPtr<cc::Vehicle>> vehicles = agentVehicles();
  vehicles.push_back(egoVehicle());
  return vehicles;
}

vector<SharedPtr<const cc::Vehicle>> CarlaSimulatorNode::vehicles() const {
  vector<SharedPtr<const cc::Vehicle>> vehicles = agentVehicles();
  vehicles.push_back(egoVehicle());
  return vehicles;
}

void CarlaSimulatorNode::publishMap() const {

  vector<SharedPtr<cc::Waypoint>> waypoints = world_->GetMap()->GenerateWaypoints(5.0);
  vector<SharedPtr<const cc::Waypoint>> const_waypoints;
  for (const auto& waypoint : waypoints)
    const_waypoints.push_back(waypoint);

  visualization_msgs::MarkerPtr waypoints_msg =
    createWaypointMsg(const_waypoints);
  visualization_msgs::MarkerPtr junctions_msg =
    createJunctionMsg(world_->GetMap()->GetTopology());
  visualization_msgs::MarkerArrayPtr road_ids_msg =
    createRoadIdsMsg(world_->GetMap()->GetMap().GetMap().GetRoads());

  visualization_msgs::MarkerArrayPtr map_msg(new visualization_msgs::MarkerArray);
  map_msg->markers.push_back(*waypoints_msg);
  map_msg->markers.push_back(*junctions_msg);
  map_msg->markers.insert(
      map_msg->markers.begin(),
      road_ids_msg->markers.begin(), road_ids_msg->markers.end());

  map_pub_.publish(map_msg);
  return;
}

void CarlaSimulatorNode::publishTraffic() const {

  // Ego vehicle transform.
  tf_broadcaster_.sendTransform(*(createVehicleTransformMsg(egoVehicle(), "ego")));

  // Traffic msg.
  visualization_msgs::MarkerArrayPtr vehicles_msg =
    createVehiclesMsg(this->vehicles());
  visualization_msgs::MarkerArrayPtr vehicle_ids_msg =
    createVehicleIdsMsg(this->vehicles());
  visualization_msgs::MarkerArrayPtr lattice_msg =
    createTrafficManagerMsg(traffic_manager_);

  visualization_msgs::MarkerArrayPtr traffic_msg(
      new visualization_msgs::MarkerArray);
  traffic_msg->markers.insert(
      traffic_msg->markers.end(),
      vehicles_msg->markers.begin(), vehicles_msg->markers.end());
  traffic_msg->markers.insert(
      traffic_msg->markers.end(),
      vehicle_ids_msg->markers.begin(), vehicle_ids_msg->markers.end());
  traffic_msg->markers.insert(
      traffic_msg->markers.end(),
      lattice_msg->markers.begin(), lattice_msg->markers.end());

  traffic_pub_.publish(traffic_msg);
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
  ego_ready_ = true;

  if (ego_ready_ && agents_ready_) {
    ROS_INFO_NAMED("carla_simulator", "tick world by ego client.");
    world_->Tick();
    manageTraffic();
    world_->Tick();
    publishTraffic();
    sendEgoGoal();
    sendAgentsGoal();
  }

  return;
}

void CarlaSimulatorNode::sendAgentsGoal() {

  clp::AgentPlanGoal goal;
  goal.ego_policy.id = ego_policy_.first;
  goal.ego_policy.desired_speed = ego_policy_.second;

  for (const auto agent_policy : agent_policies_) {
    goal.agent_policies.push_back(clp::Policy());
    goal.agent_policies.back().id = agent_policy.first;
    goal.agent_policies.back().desired_speed = agent_policy.second;
  }

  agents_client_.sendGoal(
      goal,
      bst::bind(&CarlaSimulatorNode::agentsPlanDoneCallback, this, _1, _2),
      bst::bind(&CarlaSimulatorNode::agentsPlanActiveCallback, this),
      bst::bind(&CarlaSimulatorNode::agentsPlanFeedbackCallback, this, _1));

  agents_ready_ = false;
  return;
}

void CarlaSimulatorNode::agentsPlanDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const clp::AgentPlanResultConstPtr& result) {

  ROS_INFO_NAMED("carla_simulator", "agentsPlanDoneCallback().");
  agents_ready_ = true;

  if (ego_ready_ && agents_ready_) {
    ROS_INFO_NAMED("carla_simulator", "tick world by agents client.");
    world_->Tick();
    manageTraffic();
    world_->Tick();
    publishTraffic();
    sendEgoGoal();
    sendAgentsGoal();
  }

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


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
#include <functional>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>

#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
//#include <carla/client/Sensor.h>
//#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
//#include <carla/image/ImageIO.h>
//#include <carla/image/ImageView.h>
//#include <carla/sensor/data/Image.h>

#include <conformal_lattice_planner/EgoPlanAction.h>
#include <conformal_lattice_planner/AgentPlanAction.h>

using namespace std;
namespace bst = boost;
namespace cc = carla::client;
namespace cg = carla::geom;
namespace crpc = carla::rpc;
//namespace csd = carla::sensor::data;
namespace clp = conformal_lattice_planner;

namespace carla {

/// Prototypes for creating visualization msgs.
visualization_msgs::MarkerPtr createWaypointMsg(
    const vector<carla::SharedPtr<cc::Waypoint>>& waypoints);
visualization_msgs::MarkerPtr createJunctionMsg(
    const cc::Map::TopologyList& waypoint_pairs);
visualization_msgs::MarkerPtr createVehicleMarkerMsg(
    const carla::SharedPtr<cc::Actor>& vehicle);
geometry_msgs::TransformStampedPtr createVehicleTransformMsg(
    const carla::SharedPtr<cc::Actor>& vehicle);

class CarlaSimulatorNode {

public:

  using Ptr = std::shared_ptr<CarlaSimulatorNode>;
  using ConstPtr = std::shared_ptr<const CarlaSimulatorNode>;

private:

  SharedPtr<cc::World> world_ = nullptr;
  size_t ego_;
  std::unordered_set<size_t> agents_;

  /// ROS interface.
  mutable ros::NodeHandle nh_;
  mutable tf2_ros::TransformBroadcaster tf_broadcaster_;
  mutable ros::Publisher map_pub_;
  mutable ros::Publisher ego_marker_pub_;
  //ros::Publisher agent_markers_pub_;

  mutable actionlib::SimpleActionClient<clp::EgoPlanAction> ego_client_;
  //actionlib::SimpleActionClient<clp::AgentPlanAction> agent_client;

public:

  CarlaSimulatorNode(ros::NodeHandle nh) :
    nh_(nh), ego_client_(nh_, "ego_plan", false) {}

  /// Delete the copy constructor.
  CarlaSimulatorNode(const CarlaSimulatorNode&) = delete;

  /// Delete the copy assignment.
  CarlaSimulatorNode& operator=(const CarlaSimulatorNode&) = delete;

  /// Initialize the simulator ros node.
  bool initialize();

private:

  /// Spawn the ego vehicle.
  void spawnEgo();

  /// Manage agents around the ego vehicle.
  /// Add agents if there is empty space around the ego.
  /// Delete agents that are too far from the ego.
  void manageAgents();

  /// Publish the map visualization markers.
  void publishMap() const;

  /// Publish the vehicle visualization markers.
  void publishTraffic() const;

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
  void egoPlanFeedbackCallback(const clp::EgoPlanFeedbackConstPtr& feedback) {}
  /// @}

  /**
   * @name Agent plan callbacks
   */
  /// @{
  /// @}

}; // End class CarlaSimulatorNode.

bool CarlaSimulatorNode::initialize() {
  // Create publishers.
  map_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("town_map", 1, true);
  ego_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("ego_object", 1, true);

  // TODO: load the variables from parameters.
  string host = "localhost";
  uint16_t port = 2000;
  string map_name = "/Game/Carla/Maps/Town04";

  // Get the world.
  ROS_INFO("Connect to the server and load new map.");
  cc::Client client = cc::Client(host, port);
  client.SetTimeout(std::chrono::seconds(10));
  vector<string> map_names = client.GetAvailableMaps();
  world_ = bst::make_shared<cc::World>(client.LoadWorld(map_name));

  // Publish the map.
  ROS_INFO("Publish global map once for all.");
  publishMap();

  // Initialize the ego vehicle.
  ROS_INFO("Spawn the ego vehicle.");
  spawnEgo();

  world_->Tick();
  // Publish the ego vehicle marker.
  ROS_INFO("Publish Ego and Agents.");
  publishTraffic();

  // Wait for the planner servers.
  ROS_INFO("Waiting for planner action servers.");
  ego_client_.waitForServer(ros::Duration(2.0));

  ROS_INFO("CARLA simulator node initialization finishes.");
  return true;
}

void CarlaSimulatorNode::spawnEgo() {

  SharedPtr<cc::BlueprintLibrary> blueprint_library =
    world_->GetBlueprintLibrary();
  SharedPtr<cc::BlueprintLibrary> vehicle_library =
    blueprint_library->Filter("vehicle");
  // TODO: Load a specific vehicle blueprint.
  auto ego_blueprint = (*vehicle_library)[0];

  // TODO: Load the deired ego initial state as parameters.
  array<float, 3> ego_pt{0, 0, 0};
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
  ROS_INFO("Initial Ego x:%f y:%f z:%f r:%f p:%f y:%f",
      ego_transform.location.x, ego_transform.location.y, ego_transform.location.z,
      ego_transform.rotation.roll, ego_transform.rotation.pitch, ego_transform.rotation.yaw);
  SharedPtr<cc::Actor> ego_actor = world_->SpawnActor(ego_blueprint, ego_transform);
  ego_actor->SetSimulatePhysics(false);
  ego_ = ego_actor->GetId();

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

  // Publish the ego marker and tf.
  ego_marker_pub_.publish(createVehicleMarkerMsg(world_->GetActor(ego_)));
  tf_broadcaster_.sendTransform(*(createVehicleTransformMsg(world_->GetActor(ego_))));

  // TODO: Publish the agents' markers.
  return;
}

void CarlaSimulatorNode::sendEgoGoal() {

  clp::EgoPlanGoal goal;
  goal.ego = ego_;
  for (const size_t agent : agents_)
    goal.agents.push_back(agent);

  ego_client_.sendGoal(
      goal,
      bst::bind(&CarlaSimulatorNode::egoPlanDoneCallback, this, _1, _2),
      bst::bind(&CarlaSimulatorNode::egoPlanActiveCallback, this),
      bst::bind(&CarlaSimulatorNode::egoPlanFeedbackCallback, this, _1));

  return;
}

void CarlaSimulatorNode::egoPlanDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const clp::EgoPlanResultConstPtr& result) {

  return;
}

using CarlaSimulatorNodePtr = CarlaSimulatorNode::Ptr;
using CarlaSimulatorNodeConstPtr = CarlaSimulatorNode::ConstPtr;
} // End namespace carla.


int main(int argc, char** argv) {
  ros::init(argc, argv, "carla_simulator_node");
  ros::NodeHandle nh;

  carla::CarlaSimulatorNodePtr carla_sim =
    std::make_shared<carla::CarlaSimulatorNode>(nh);
  if (!carla_sim->initialize()) {
    ROS_ERROR("Cannot initialize the CARLA simulator.");
  }

  ros::spin();
  return 0;
}


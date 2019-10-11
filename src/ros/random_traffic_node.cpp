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

#include <array>
#include <limits>
#include <random>
#include <unordered_set>
#include <boost/timer/timer.hpp>

#include <ros/convert_to_visualization_msgs.h>
#include <ros/random_traffic_node.h>

using namespace planner;
using namespace router;

namespace carla {

void RandomTrafficNode::spawnVehicles() {

  // The start position.
  // TODO: This may be load from the ROS parameter server.
  std::array<double, 3> start_pt{0, 0, 0};

  // Find the available spawn point cloest to the start point.
  std::vector<CarlaTransform> spawn_points =
    map_->GetRecommendedSpawnPoints();
  CarlaTransform start_transform;
  double min_distance_sq = std::numeric_limits<double>::max();

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

  // Start waypoint of the lattice.
  boost::shared_ptr<CarlaWaypoint> start_waypoint =
    fast_map_->waypoint(start_transform.location);

  // Initialize the traffic manager.
  traffic_manager_ = boost::make_shared<TrafficManager<LoopRouter>>(
      start_waypoint, 150.0, loop_router_, map_, fast_map_);

  // Spawn the ego vehicle.
  // The ego vehicle is at 50m on the lattice, and there is an 100m buffer
  // in the front of the ego vehicle.
  boost::shared_ptr<const CarlaWaypoint> ego_waypoint =
    traffic_manager_->front(start_waypoint, 50.0)->waypoint();
  if (!ego_waypoint) {
    throw std::runtime_error("Cannot find the ego waypoint on the traffic lattice.");
  }
  ROS_INFO_NAMED("carla_simulator", "Ego vehicle initial transform\nx:%f y:%f z:%f",
      ego_waypoint->GetTransform().location.x,
      ego_waypoint->GetTransform().location.y,
      ego_waypoint->GetTransform().location.z);

  if (!spawnEgoVehicle(ego_waypoint, 25, false)) {
    throw std::runtime_error("Cannot spawn the ego vehicle.");
  }

  // Spawn the agent vehicles.
  {
    boost::shared_ptr<const CarlaWaypoint> waypoint0 = ego_waypoint;
    boost::shared_ptr<const CarlaWaypoint> waypoint1 = ego_waypoint->GetRight();
    boost::shared_ptr<const CarlaWaypoint> waypoint2 = waypoint1->GetRight();
    boost::shared_ptr<const CarlaWaypoint> waypoint3 = waypoint2->GetRight();
    boost::shared_ptr<const CarlaWaypoint> agent_waypoint = nullptr;

    // First lane, 80m, 20m/s.
    agent_waypoint = traffic_manager_->front(waypoint0, 40.0)->waypoint();
    if (!spawnAgentVehicle(agent_waypoint, 20.0, false)) {
      throw std::runtime_error("Cannot spawn an agent vehicle.");
    }

    // Second lane, 60m, 20m/s
    agent_waypoint = traffic_manager_->back(waypoint1, 30.0)->waypoint();
    if (!spawnAgentVehicle(agent_waypoint, 20.0, false)) {
      throw std::runtime_error("Cannot spawn an agent vehicle.");
    }
  }

  // Let the server know about the vehicles.
  world_->Tick();

  // Spawn the following camera of the ego vehicle.
  spawnCamera();

  return;
}

boost::optional<size_t> RandomTrafficNode::spawnEgoVehicle(
    const boost::shared_ptr<const CarlaWaypoint>& waypoint,
    const double policy_speed,
    const bool noisy_speed) {

  // Get the blueprint of the ego vehicle.
  boost::shared_ptr<CarlaBlueprintLibrary> blueprint_library =
    world_->GetBlueprintLibrary()->Filter("vehicle");
  auto blueprint = blueprint_library->at("vehicle.audi.tt");

  // Make sure the vehicle will fall onto the ground instead of fall endlessly.
  CarlaTransform transform = waypoint->GetTransform();
  transform.location.z += 0.05;

  boost::shared_ptr<CarlaActor> actor = world_->TrySpawnActor(blueprint, transform);
  boost::shared_ptr<CarlaVehicle> vehicle = boost::static_pointer_cast<CarlaVehicle>(actor);

  if (!actor) {
    // Cannot spawn the actor.
    // There might be a collision at the waypoint or something.
    ROS_ERROR_NAMED("carla simulator", "Cannot spawn the ego vehicle.");
    return boost::none;
  }

  // Disable the vehicle physics.
  actor->SetSimulatePhysics(false);

  if (traffic_manager_->addVehicle(
        std::make_tuple(vehicle->GetId(), transform, vehicle->GetBoundingBox())) != 1) {
    // Cannot add this vehicle to the traffic lattice.
    // This is either because the vehicle already exists or it causes a collision on the lattice.
    vehicle->Destroy();
    ROS_ERROR_NAMED("carla simulator", "Cannot add ego vehicle to the traffic lattice.");
    return boost::none;
  }

  // Set the ego vehicle policy.
  const size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine rand_gen(seed);
  std::uniform_real_distribution<double> uni_real_dist(-2.0, 2.0);

  if (noisy_speed) {
    ego_policy_ = std::make_pair(vehicle->GetId(), policy_speed+uni_real_dist(rand_gen));
    ego_speed_  = std::make_pair(vehicle->GetId(), policy_speed+uni_real_dist(rand_gen));
  } else {
    ego_policy_ = std::make_pair(vehicle->GetId(), policy_speed);
    ego_speed_  = std::make_pair(vehicle->GetId(), policy_speed);
  }

  return vehicle->GetId();
}

boost::optional<size_t> RandomTrafficNode::spawnAgentVehicle(
    const boost::shared_ptr<const CarlaWaypoint>& waypoint,
    const double policy_speed,
    const bool noisy_speed) {

  // Get the blueprint of the vehicle, which is randomly chosen from the
  // vehicle blueprint library.
  boost::shared_ptr<CarlaBlueprintLibrary> blueprint_library =
    world_->GetBlueprintLibrary()->Filter("vehicle");
  auto blueprint = (*blueprint_library)[std::rand() % blueprint_library->size()];

  // Make sure the vehicle will fall onto the ground instead of fall endlessly.
  CarlaTransform transform = waypoint->GetTransform();
  transform.location.z += 1.5;

  boost::shared_ptr<CarlaActor> actor = world_->TrySpawnActor(blueprint, transform);
  boost::shared_ptr<CarlaVehicle> vehicle = boost::static_pointer_cast<CarlaVehicle>(actor);

  if (!actor) {
    // Cannot spawn the actor.
    // There might be a collision at the waypoint or something.
    ROS_ERROR_NAMED("carla simulator", "Cannot spawn the agent vehicle.");
    return boost::none;
  }

  // Disable the vehicle physics.
  actor->SetSimulatePhysics(false);

  if (traffic_manager_->addVehicle(
        std::make_tuple(vehicle->GetId(), transform, vehicle->GetBoundingBox())) != 1) {
    // Cannot add this vehicle to the traffic lattice.
    // This is either because the vehicle already exists or it causes a collision on the lattice.
    vehicle->Destroy();
    ROS_ERROR_NAMED("carla simulator", "Cannot add the agent vehicle to the traffic lattice.");
    return boost::none;
  }

  // Set the agent vehicle policy
  const size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine rand_gen(seed);
  std::uniform_real_distribution<double> uni_real_dist(-2.0, 2.0);

  if (noisy_speed) {
    agent_policies_[vehicle->GetId()] = policy_speed + uni_real_dist(rand_gen);
    agent_speed_[vehicle->GetId()]    = policy_speed + uni_real_dist(rand_gen);
  } else {
    agent_policies_[vehicle->GetId()] = policy_speed;
    agent_speed_[vehicle->GetId()]    = policy_speed;
  }

  return vehicle->GetId();
}

void RandomTrafficNode::manageTraffic() {

  // Check the position of the ego vehicle on the lattice.
  // We wanto to make sure it is at the 50m distance.
  const boost::shared_ptr<CarlaWaypoint> ego_waypoint =
    fast_map_->waypoint(egoVehicle()->GetTransform().location);
  boost::shared_ptr<const TrafficManager<LoopRouter>> const_traffic_manager =
    boost::const_pointer_cast<const TrafficManager<LoopRouter>>(traffic_manager_);
  const double ego_distance = const_traffic_manager->closestNode(ego_waypoint, 1.0)->distance();
  const double shift_distance = ego_distance<50.0 ? 0.0 : 2.0;

  // Get all vehicles for the server.
  std::vector<boost::shared_ptr<const CarlaVehicle>> vehicles =
    const_cast<const RandomTrafficNode*>(this)->vehicles();

  // Update the traffic on the lattice.
  std::unordered_set<size_t> disappear_vehicles;
  if (!traffic_manager_->moveTrafficForward(
        vehicles, shift_distance, disappear_vehicles)) {
    ROS_ERROR_NAMED("carla simulator", "Collision detected");
  }

  // Remove the vehicles that disappear.
  for (const size_t id : disappear_vehicles) {
    if (id == ego_policy_.first) {
      ROS_ERROR_NAMED("carla simulator", "The ego vehicle is removed.");
      throw std::runtime_error("The ego vehicle is removed from the simulation.");
    }

    // Remove the vehicle from the carla server.
    boost::shared_ptr<CarlaVehicle> vehicle = agentVehicle(id);
    if (!world_->GetActor(id)->Destroy())
      ROS_WARN_NAMED("carla simulator", "Cannot destroy an agent");

    // Remove the vehicle from the object.
    agent_policies_.erase(id);
    agent_speed_.erase(id);
  }

  // Spawn more vehicles if the number of agents around the ego vehicle
  // does not meet the requirement.
  // At most one vehicle is spawned every time this function is called.
  if (agent_policies_.size() < 5) {

    const double min_distance = 20.0;
    boost::optional<std::pair<double, boost::shared_ptr<const CarlaWaypoint>>> front =
      traffic_manager_->frontSpawnWaypoint(min_distance);
    boost::optional<std::pair<double, boost::shared_ptr<const CarlaWaypoint>>> back =
      traffic_manager_->backSpawnWaypoint(min_distance);

    double front_distance = front ? front->first : 0.0;
    double back_distance = back ? back->first : 0.0;
    if (!traffic_manager_->back(front->second, 10.0)) front_distance = 0.0;
    if (!traffic_manager_->front(back->second, 10.0)) back_distance = 0.0;

    // Waypoint to spawn the new vehicle.
    boost::shared_ptr<const CarlaWaypoint> spawn_waypoint = nullptr;
    double policy_speed = 0.0;

    const size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine rand_gen(seed);
    std::uniform_real_distribution<double> uni_real_dist(0.0, 5.0);

    if (front_distance>=back_distance && front_distance>=min_distance) {
      // Spawn a new vehicle at the front of the lattice.
      const double distance = 3.0 + uni_real_dist(rand_gen);
      boost::shared_ptr<const CarlaWaypoint> waypoint = front->second;
      spawn_waypoint = traffic_manager_->back(waypoint, distance)->waypoint();
      policy_speed = 20.0;
    }

    if (front_distance<back_distance && back_distance>=min_distance) {
      // Spawn a new vehicle at the back of the lattice.
      const double distance = 3.0 + uni_real_dist(rand_gen);
      boost::shared_ptr<const CarlaWaypoint> waypoint = back->second;
      spawn_waypoint = traffic_manager_->front(waypoint, distance)->waypoint();
      policy_speed = 22.0;
    }

    if (!spawn_waypoint) {
      ROS_WARN_NAMED("carla simulator",
          "Cannot find a spawn waypoint for a new agent vehicle.");
      return;
    }

    if (!spawnAgentVehicle(spawn_waypoint, policy_speed)) {
      ROS_WARN_NAMED("carla simulator",
          "Cannot spawn a new agent vehicle at the given waypoint.");
      return;
    }
  }

  return;
}

void RandomTrafficNode::tickWorld() {

  // This tick is for update the vehicle transforms set by the planners.
  world_->Tick();
  manageTraffic();
  // This tick is for adding or removing vehicles informed by the \c manageTraffic() function.
  world_->Tick();
  publishTraffic();
  sendEgoGoal();
  sendAgentsGoal();

  return;
}

void RandomTrafficNode::publishTraffic() const {

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

} // End namespace carla.


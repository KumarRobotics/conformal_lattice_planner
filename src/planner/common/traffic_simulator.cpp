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
#include <limits>
#include <string>
#include <boost/format.hpp>

#include <planner/common/utils.h>
#include <planner/common/traffic_simulator.h>

namespace planner {

const std::tuple<size_t, typename TrafficSimulator::CarlaTransform, double, double, double>
  TrafficSimulator::updatedAgentTuple(
      const size_t id, const double accel, const double dt) const {

    const Vehicle& agent = snapshot_.vehicle(id);

    // The updated speed.
    const double updated_speed = agent.speed() + accel*dt;

    // Computed updated transform of the vehicle.
    // All agents are assumed to be lane followers.
    // 1) If we can find the next waypoint for the agent on the route,
    //    this is what we preferred.
    // 2) If we cannot, this implies the agent is leaving the route.
    //    we will use the carla map API to find an accessible next waypoint.
    boost::shared_ptr<CarlaWaypoint> waypoint =
      fast_map_->waypoint(agent.transform().location);
    const double movement = agent.speed()*dt + 0.5*accel*dt*dt;

    boost::shared_ptr<CarlaWaypoint> next_waypoint = nullptr;

    //std::printf("agent:%lu speed:%f accel:%f dt:%f movement:%f\n",
    //    agent.id(), agent.speed(), accel, dt, movement);

    try {
      // Prefer the next waypoint on the route.
      if (movement == 0.0) next_waypoint = waypoint;
      else next_waypoint = router_->frontWaypoint(waypoint, movement);
      // It is normal that we cannot find the next waypoint on the route.
      // It is treated as an exception so that, all exceptions (together with
      // the ones thrown by \c frontWayoint) can be handed by the following
      // \c catch.
      if (!next_waypoint) {
        throw std::runtime_error(
          "TrafficSimulator::updateAgentTupel(): "
          "next waypoint is not available.\n");
      }
    } catch (...) {
      // Otherwise, we have to settle with some waypoints outside the route.
      // Find the next waypoint candidates.
      std::vector<boost::shared_ptr<CarlaWaypoint>> next_waypoints;

      if (movement == 0.0) next_waypoints.push_back(waypoint);
      else next_waypoints = waypoint->GetNext(movement);

      if (next_waypoints.size() == 0) {
        std::string error_msg(
            "TrafficSimulator::updatedAgentTuple(): "
            "cannot find a next waypoint for an agent.\n");
        std::string agent_msg =
          (boost::format("agent %1%: x:%2% y:%3% z:%4% speed:%5% accel:%6% movement:%7%\n")
            % agent.id()
            % agent.transform().location.x
            % agent.transform().location.y
            % agent.transform().location.z
            % agent.speed()
            % accel
            % movement).str();
        std::string waypoint_msg =
          (boost::format("waypoint %1% x:%2% y:%3% z:%4% r:%5% p:%6% y:%7% road:%8% lane:%9%.\n")
           % waypoint->GetId()
           % waypoint->GetTransform().location.x
           % waypoint->GetTransform().location.y
           % waypoint->GetTransform().location.z
           % waypoint->GetTransform().rotation.roll
           % waypoint->GetTransform().rotation.pitch
           % waypoint->GetTransform().rotation.yaw
           % waypoint->GetRoadId()
           % waypoint->GetLaneId()).str();
        throw std::runtime_error(error_msg + agent_msg + waypoint_msg);
      }

      // Select the one with the least angle difference.
      next_waypoint = next_waypoints.front();
      for (size_t i = 1; i < next_waypoints.size(); ++i) {
        const double diff1 = utils::shortestAngle(
            waypoint->GetTransform().rotation.yaw,
            next_waypoint->GetTransform().rotation.yaw);
        const double diff2 = utils::shortestAngle(
            waypoint->GetTransform().rotation.yaw,
            next_waypoints[i]->GetTransform().rotation.yaw);

        if (std::fabs(diff2) < std::fabs(diff1))
          next_waypoint = next_waypoints[i];
      }
    }

    double update_curvature = 0.0;
    CarlaTransform update_transform;

    update_transform = next_waypoint->GetTransform();
    update_curvature = utils::curvatureAtWaypoint(next_waypoint, map_);

    return std::make_tuple(id, update_transform, updated_speed, accel, update_curvature);
}

const double TrafficSimulator::remainingTime(
    const double speed, const double accel, const double distance) const {

  if (speed < 0.0) {
    std::string error_msg = (boost::format(
          "TrafficSimulator::remainingTime(): "
          "the input speed [%1%] < 0.0.\n") % speed).str();
    throw std::runtime_error(error_msg);
  }

  // Just a number large enough.
  double t = 100.0;

  if (accel < 0.0) t = -speed / accel;

  if (speed*t+0.5*accel*t*t > distance) {
    if (accel != 0.0) {
      const double final_speed = std::sqrt(speed*speed + 2.0*accel*distance);
      t = (final_speed - speed) / accel;
    } else t = distance / speed;
  }

  return t;
}

const double TrafficSimulator::ttcCost(const double ttc) const {
  // The cost map for ttc.
  // [0, 1) -> 4
  // [1, 2) -> 2
  // [2, 3) -> 1
  // {3, +) -> 0
  static std::unordered_map<int, double> cost_map {
    {0, 4.0}, {1, 2.0}, {2, 1.0}
  };

  if (ttc < 0.0) {
    std::string error_msg = (
        boost::format("TrafficSimulator::ttcCost(): the input ttc [%1%] < 0.0.\n") % ttc).str();
    throw std::runtime_error(error_msg);
  }

  const int ttc_key = static_cast<int>(ttc);
  if (ttc_key <= 2) return cost_map[ttc_key];
  else return 0.0;
}

const double TrafficSimulator::ttcCost() const {
  boost::optional<std::pair<size_t, double>> ego_lead =
    snapshot_.trafficLattice()->front(snapshot_.ego().id());

  if (!ego_lead) return ttcCost(10.0);
  else return ttcCost(ego_lead->second/snapshot_.ego().speed());
}

const double TrafficSimulator::accelCost(const double accel) const {
  // The cost map for brake.
  // [0, 1) -> 0
  // [1, 2) -> 1
  // [2, 4) -> 2
  // [4, 6) -> 4
  // [6, 8) -> 8
  // [8, +) -> 16
  static std::unordered_map<int, double> cost_map {
    {0, 0.0},
    {1, 1.0},
    {2, 2.0}, {3, 2.0},
    {4, 4.0}, {5, 4.0},
    {6, 6.0}, {7, 6.0},
  };

  if (accel >= 0.0) return 0.0;

  const int brake_key = static_cast<int>(-accel);
  if (brake_key < 8) return cost_map[brake_key];
  else return 6.0;
}

const double TrafficSimulator::accelCost() const {
  // We consider four vehicles in computing the accel cost.
  // The ego and the followers of the ego vehicle.
  boost::optional<std::pair<size_t, double>> back =
    snapshot_.trafficLattice()->back(snapshot_.ego().id());
  boost::optional<std::pair<size_t, double>> left_back =
    snapshot_.trafficLattice()->leftBack(snapshot_.ego().id());
  boost::optional<std::pair<size_t, double>> right_back =
    snapshot_.trafficLattice()->rightBack(snapshot_.ego().id());

  double ego_brake_cost = accelCost(snapshot_.ego().acceleration());
  double agent_brake_cost = 0.0;
  if (back)       agent_brake_cost += accelCost(snapshot_.vehicle(back->first).acceleration());
  if (left_back)  agent_brake_cost += accelCost(snapshot_.vehicle(left_back->first).acceleration());
  if (right_back) agent_brake_cost += accelCost(snapshot_.vehicle(right_back->first).acceleration());

  return ego_brake_cost + 0.5*agent_brake_cost;
}

const bool TrafficSimulator::simulate(
    const ContinuousPath& path, const double default_dt, const double max_time,
    double& time, double& cost) {

  //std::printf("simulate(): \n");

  // Reset the output to 0.
  time = 0.0;
  cost = 0.0;

  // The actual simulation time step, which may vary for different iterations.
  double dt = default_dt;
  // The distance that ego has travelled on the input path.
  double ego_distance = 0.0;

  // FIXME: This is just a trial for defining the stage costs.
  std::vector<double> ttc_cost;
  std::vector<double> brake_cost;

  while (time < max_time && dt >= default_dt) {

    // Used to store the updated status of all vehicles.
    std::vector<std::tuple<size_t, CarlaTransform, double, double, double>> updated_tuples;

    // The acceleration to be applied by the ego vehicle.
    const double ego_accel = egoAcceleration();

    // Compute the actual time step.
    const double remaining_time = remainingTime(
        snapshot_.ego().speed(), ego_accel, path.range()-ego_distance);
    dt = default_dt;
    dt = dt <= remaining_time ? dt : remaining_time;
    dt = dt <= max_time-time  ? dt : max_time-time;

    //std::printf("============================================\n");
    //std::cout << snapshot_.string("start snapshot:\n");
    //std::printf("ego accel:%f\n", ego_accel);
    //std::printf("default dt:%f max time:%f remaining time:%f\n",
    //    default_dt, max_time, remaining_time);
    //std::printf("time:%f dt:%f\n", time, dt);

    // Update the distance of the ego on the path.
    ego_distance += snapshot_.ego().speed()*dt + 0.5*ego_accel*dt*dt;
    if (ego_distance > path.range()) ego_distance = path.range();
    //std::printf("ego distance:%f path range:%f\n", ego_distance, path.range());

    // Store the updated status of the ego.
    std::pair<CarlaTransform, double> ego_transform = path.transformAt(ego_distance);
    updated_tuples.push_back(std::make_tuple(
          snapshot_.ego().id(),
          ego_transform.first,
          snapshot_.ego().speed()+ego_accel*dt,
          ego_accel,
          ego_transform.second));

    // Take care of the agents.
    for (const auto& item : snapshot_.agents()) {
      const Vehicle& agent = item.second;
      const double agent_accel = agentAcceleration(agent.id());
      updated_tuples.push_back(updatedAgentTuple(agent.id(), agent_accel, dt));
      //std::printf("agent %lu accel: %f\n", agent.id(), agent_accel);
    }

    // Update the snapshot.
    if (!snapshot_.updateTraffic(updated_tuples)) {
      //std::printf("Collision detected in the simulation.\n");
      return false;
    }

    //std::cout << snapshot_.string("end simulation snapshot:\n");

    // TODO: Accumulate the cost.
    ttc_cost.push_back(ttcCost());
    brake_cost.push_back(accelCost());

    //std::printf("ttc cost: %f\n", ttcCost());
    //std::printf("brake cost: %f\n", accelCost());

    // Tick the time.
    time += dt;
  }

  // TODO: Should I use mean or max?
  double average_ttc_cost = 0.0;
  for (const auto c : ttc_cost) average_ttc_cost += c;
  average_ttc_cost /= ttc_cost.size();

  double average_brake_cost = 0.0;
  for (const auto c : brake_cost) average_brake_cost += c;
  average_brake_cost /= brake_cost.size();

  //std::printf("average ttc cost: %f\n", average_ttc_cost);
  //std::printf("average brake cost: %f\n", average_brake_cost);
  //std::printf("\n");

  cost = average_ttc_cost + average_brake_cost;
  if (path.laneChangeType() != VehiclePath::LaneChangeType::KeepLane)
    cost += 1.0;

  return true;
}

} // End namespace planner.

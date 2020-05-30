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

#include <planner/common/traffic_manager.h>
#include <node/simulator/simulator_node.h>

namespace node {

class RandomTrafficNode : public SimulatorNode {

private:

  using Base = SimulatorNode;
  using This = RandomTrafficNode;

public:

  using Ptr = boost::shared_ptr<This>;
  using ConstPtr = boost::shared_ptr<const This>;

protected:

  using CarlaSensor = carla::client::Sensor;
  using CarlaSensorData = carla::sensor::SensorData;
  using CarlaBGRAImage = carla::sensor::data::Image;

protected:

  /// Traffic lattice.
  boost::shared_ptr<planner::TrafficManager> traffic_manager_;

  /// Nominal policy speed of all vehicles.
  const double nominal_policy_speed_ = 20.0;

public:

  RandomTrafficNode(ros::NodeHandle nh) : Base(nh) {}

protected:

  virtual void spawnVehicles() override;

  virtual boost::optional<size_t> spawnEgoVehicle(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double policy_speed,
      const bool noisy_speed = true) override;

  virtual boost::optional<size_t> spawnAgentVehicle(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double policy_speed,
      const bool noisy_speed = true) override;

  /// Manager (add/delete) the vehicles in the simulation.
  void manageTraffic();

  virtual void tickWorld() override;

  virtual void publishTraffic() const override;

}; // End class RandomTrafficNode.

using RandomTrafficNodePtr = RandomTrafficNode::Ptr;
using RandomTrafficNodeConstPtr = RandomTrafficNode::ConstPtr;

} // End namespace node.

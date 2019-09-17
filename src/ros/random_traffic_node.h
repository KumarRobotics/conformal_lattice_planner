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

#include <boost/optional.hpp>

#include <image_transport/image_transport.h>

#include <carla/client/Sensor.h>
#include <carla/sensor/data/Image.h>

#include <conformal_lattice_planner/loop_router.h>
#include <conformal_lattice_planner/traffic_manager.h>
#include <ros/simulator_node.h>

namespace carla {

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

  /// Loop router, the router is predefined on Town04.
  boost::shared_ptr<router::LoopRouter> loop_router_;

  /// Traffic lattice.
  boost::shared_ptr<planner::TrafficManager<router::LoopRouter>> traffic_manager_;

  // A camera following the ego vehicle to generate the third person view.
  boost::shared_ptr<CarlaSensor> following_cam_ = nullptr;


  /// ROS image transport.
  mutable image_transport::ImageTransport img_transport_;

  /// Publishing images for the following camera.
  mutable image_transport::Publisher following_img_pub_;

public:

  RandomTrafficNode(ros::NodeHandle nh) :
    Base(nh),
    loop_router_(new router::LoopRouter),
    img_transport_(nh_) {}

protected:

  virtual void spawnVehicles() override;

  boost::optional<size_t> spawnEgoVehicle(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double policy_speed,
      const bool noisy_policy_speed = true,
      const bool noisy_start_speed = true);

  boost::optional<size_t> spawnAgentVehicle(
      const boost::shared_ptr<const CarlaWaypoint>& waypoint,
      const double policy_speed,
      const bool noisy_policy_speed = true,
      const bool noisy_start_speed = true);

  void spawnCamera();

  /// Manager (add/delete) the vehicles in the simulation.
  void manageTraffic();

  virtual void tickWorld() override;

  virtual void publishTraffic() const override;

  /// Publish the following image.
  void publishImage(const boost::shared_ptr<CarlaSensorData>& data) const;

}; // End class RandomTrafficNode.

using RandomTrafficNodePtr = RandomTrafficNode::Ptr;
using RandomTrafficNodeConstPtr = RandomTrafficNode::ConstPtr;

} // End namespace carla.

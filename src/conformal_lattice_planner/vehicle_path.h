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
#include <map>
#include <string>
#include <carla/geom/Transform.h>
#include <conformal_lattice_planner/kn_path_gen.h>

namespace planner {

class ContinuousPath;
class DiscretePath;

class VehiclePath {

public:

  enum LaneChangeType {
    KeepLane = 0,
    LeftLaneChange = 1,
    RightLaneChange = 2,
    None = 3
  };

protected:

  using CarlaTransform = carla::geom::Transform;

protected:

  LaneChangeType lane_change_type_ = LaneChangeType::None;

public:

  VehiclePath() = default;

  VehiclePath(const LaneChangeType lane_change_type) :
    lane_change_type_(lane_change_type) {}

  virtual ~VehiclePath() {}

  /// Get the carla compatible left-handed transform at the start of the path.
  virtual const std::pair<CarlaTransform, double>
    startTransform() const = 0;

  /// Get the carla compatible left-handed transform at the end of the path.
  virtual const std::pair<CarlaTransform, double>
    endTransform() const = 0;

  /// Return the range of the path.
  virtual const double range() const = 0;

  /// Return the lane change type of the path.
  virtual const LaneChangeType laneChangeType() const {
    return lane_change_type_;
  }

  /// Get the carla compatible left-handed transform and curvature at s,
  /// i.e. distance from the start of the path.
  virtual const std::pair<CarlaTransform, double> transformAt(const double s) const = 0;

  /// Returns samples on the path from the start to the end with 0.1m resolution.
  virtual const std::vector<std::pair<CarlaTransform, double>> samples() const;

protected:

  NonHolonomicPath::State carlaTransformToPathState(
      const std::pair<CarlaTransform, double>& transform) const;

  std::pair<CarlaTransform, double> pathStateToCarlaTransform(
      const NonHolonomicPath::State& state,
      const CarlaTransform& base_transform) const;

  std::pair<CarlaTransform, double> interpolateTransform(
      const std::pair<CarlaTransform, double>& t1,
      const std::pair<CarlaTransform, double>& t2,
      const double w) const;

}; // End class VehiclePath.

class ContinuousPath : public VehiclePath {

private:

  using Base = VehiclePath;
  using This = ContinuousPath;

protected:

  std::pair<CarlaTransform, double> start_;
  std::pair<CarlaTransform, double> end_;

  NonHolonomicPath path_;

public:

  ContinuousPath(const std::pair<CarlaTransform, double>& start,
                 const std::pair<CarlaTransform, double>& end,
                 const LaneChangeType& lane_change_type);

  ContinuousPath(const DiscretePath& discrete_path);

  virtual ~ContinuousPath() {}

  virtual const std::pair<CarlaTransform, double>
    startTransform() const override { return start_; }

  virtual const std::pair<CarlaTransform, double>
    endTransform() const override { return end_; }

  virtual const double range() const override { return path_.sf; }

  virtual const std::pair<CarlaTransform, double>
    transformAt(const double s) const override;

  std::string string(const std::string& prefix="") const;

}; // End class ContinuousPath.

/**
 * TODO: Complete the implementation for this class later.
 */
class DiscretePath : public VehiclePath {

private:

  using Base = VehiclePath;
  using This = DiscretePath;

protected:

  /// Stores the samples on path..
  //std::map<double, CarlaTransform> samples_;
  std::map<double, std::pair<CarlaTransform, double>> samples_;

public:

  DiscretePath(const std::pair<CarlaTransform, double>& start,
               const std::pair<CarlaTransform, double>& end,
               const LaneChangeType& lane_change_type);

  DiscretePath(const ContinuousPath& continuous_path);

  virtual ~DiscretePath() {}

  virtual const std::pair<CarlaTransform, double>
    startTransform() const override {
    return samples_.begin()->second;
  }

  virtual const std::pair<CarlaTransform, double>
    endTransform() const override {
    return samples_.rbegin()->second;
  }

  virtual const double range() const override {
    return samples_.rbegin()->first;
  }

  virtual const std::pair<CarlaTransform, double>
    transformAt(const double s) const override;

  virtual const std::vector<std::pair<CarlaTransform, double>>
    samples() const override;

  virtual void append(const DiscretePath& path);

  virtual void append(const ContinuousPath& path) {
    DiscretePath discrete_path(path);
    append(discrete_path);
    return;
  }

  std::string string(const std::string& prefix="") const;

}; // End class DiscretePath.

} // End namespace planner.


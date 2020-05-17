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
#include <map>
#include <string>
#include <carla/geom/Transform.h>

#include <planner/common/kn_path_gen.h>

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
  std::map<double, std::pair<CarlaTransform, double>> samples_;

  double resolution_ = 0.5;

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

  //virtual const std::vector<std::pair<CarlaTransform, double>>
  //  samples() const override;

  virtual void append(const DiscretePath& path);

  virtual void append(const ContinuousPath& path) {
    DiscretePath discrete_path(path);
    append(discrete_path);
    return;
  }

  std::string string(const std::string& prefix="") const;

}; // End class DiscretePath.

} // End namespace planner.


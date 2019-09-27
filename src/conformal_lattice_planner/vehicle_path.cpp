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

#include <stdexcept>
#include <algorithm>
#include <conformal_lattice_planner/vehicle_path.h>
#include <conformal_lattice_planner/utils.h>

namespace planner {

NonHolonomicPath::State VehiclePath::carlaTransformToPathState(
    const CarlaTransform& transform) const {
  const CarlaTransform transform_rh = utils::convertTransform(transform);
  return NonHolonomicPath::State(transform_rh.location.x,
                                 transform_rh.location.y,
                                 transform_rh.rotation.yaw,
                                 0.0);
}

carla::geom::Transform VehiclePath::pathStateToCarlaTransform(
    const NonHolonomicPath::State& state,
    const CarlaTransform& base_transform) const {

  // The input state only tells x, y, and theta/yaw.
  // We will get the rest from the base transform.
  CarlaTransform transform = utils::convertTransform(base_transform);
  transform.location.x = state.x;
  transform.location.y = state.y;
  transform.rotation.yaw = state.theta;

  // Convert to left handed coordinates.
  utils::convertTransformInPlace(transform);
  return transform;
}

VehiclePath::CarlaTransform VehiclePath::interpolateTransform(
    const CarlaTransform& t1,
    const CarlaTransform& t2,
    const double w) const {

  if (w < 0.0 || w > 1.0) {
    throw std::runtime_error(
        "Weight for the first transform should be within the range of [0, 1].");
  }

  CarlaTransform t;
  t.location = t1.location*w + t2.location*(1.0-w);
  t.rotation.roll  = t1.rotation.roll *w + t2.rotation.roll *(1.0-w);
  t.rotation.pitch = t1.rotation.pitch*w + t2.rotation.pitch*(1.0-w);
  t.rotation.yaw   = t1.rotation.yaw  *w + t2.rotation.yaw  *(1.0-w);

  return t;
}

const std::vector<VehiclePath::CarlaTransform> VehiclePath::samples() const {

  double s = 0.0;
  std::vector<CarlaTransform> samples;
  for (; s < range(); s+=0.1)
    samples.push_back(transformAt(s));

  if (s < range())
    samples.push_back(transformAt(range()));

  return samples;
}

ContinuousPath::ContinuousPath(
    const CarlaTransform& start,
    const CarlaTransform& end,
    const LaneChangeType& lane_change_type) :
  Base  (lane_change_type),
  start_(start),
  end_  (end) {

  // Convert the start and end to right handed coordinate system.
  const NonHolonomicPath::State start_state = carlaTransformToPathState(start_);
  const NonHolonomicPath::State end_state = carlaTransformToPathState(end_);

  path_.optimizePathFast(start_state, end_state);
  return;
}

ContinuousPath::ContinuousPath(const DiscretePath& discrete_path) :
  Base  (discrete_path.laneChangeType()),
  start_(discrete_path.startTransform()),
  end_  (discrete_path.endTransform()) {

  // Convert the start and end to right handed coordinate system.
  const NonHolonomicPath::State start_state = carlaTransformToPathState(start_);
  const NonHolonomicPath::State end_state = carlaTransformToPathState(end_);

  path_.optimizePathFast(start_state, end_state);
  return;
}

const ContinuousPath::CarlaTransform ContinuousPath::transformAt(const double s) const {

  if (s < 0.0 || s > path_.sf)
    throw std::out_of_range("The input distance is out of the range of the path.");

  const NonHolonomicPath::State start_state = carlaTransformToPathState(start_);
  const NonHolonomicPath::State state = path_.evaluate(start_state, s);

  // Generate a base transform by interpolating start and end.
  const double ratio = s / path_.sf;
  CarlaTransform base_transform = interpolateTransform(start_, end_, 1.0-ratio);

  return pathStateToCarlaTransform(state, base_transform);
}

DiscretePath::DiscretePath(
    const CarlaTransform& start,
    const CarlaTransform& end,
    const LaneChangeType& lane_change_type) :
  Base(lane_change_type) {

  // Convert the start and end to right handed coordinate system.
  const NonHolonomicPath::State start_state = carlaTransformToPathState(start);
  const NonHolonomicPath::State end_state = carlaTransformToPathState(end);

  // Compute the Kelly-Navy path.
  NonHolonomicPath path;
  path.optimizePathFast(start_state, end_state);

  // Sample the path with 0.1m resolution.
  double s = 0.0;
  for (; s <= path.sf; s += 0.1) {
    const double ratio = s / range();
    CarlaTransform base_transform = interpolateTransform(start, end, 1.0-ratio);

    NonHolonomicPath::State state = path.evaluate(start_state, s);
    samples_[s] = pathStateToCarlaTransform(state, base_transform);
  }

  if (s < path.sf) {
    NonHolonomicPath::State state = path.evaluate(start_state, path.sf);
    samples_[path.sf] = pathStateToCarlaTransform(state, end);
  }

  if (samples_.empty())
    throw std::runtime_error("Empty discrete path.");

  return;
}

DiscretePath::DiscretePath(const ContinuousPath& continuous_path) :
  Base(continuous_path.laneChangeType()) {

  double s = 0.0;
  for (; s <= continuous_path.range(); s += 0.1)
    samples_[s] = continuous_path.transformAt(s);

  if (s < continuous_path.range()) {
    samples_[continuous_path.range()] =
      continuous_path.transformAt(continuous_path.range());
  }

  if (samples_.empty())
    throw std::runtime_error("Empty discrete path.");

  return;
}

const DiscretePath::CarlaTransform DiscretePath::transformAt(const double s) const {

  if (s < 0.0 || s > range())
    throw std::out_of_range("The input distance is out of the range of the path.");

  if (s == 0.0) return samples_.begin()->second;
  if (s == samples_.rbegin()->first) return samples_.rbegin()->second;

  std::map<double, CarlaTransform>::const_iterator iter = std::find_if(
      samples_.begin(), samples_.end(),
      [&s](const std::pair<double, CarlaTransform>& sample)->bool{
        return sample.first > s;
      });

  if (iter==samples_.begin() || iter==samples_.end())
    throw std::runtime_error("The input distance is out of the range of the path.");

  std::map<double, CarlaTransform>::const_iterator iter1 = --iter;
  std::map<double, CarlaTransform>::const_iterator iter2 = iter;
  const double ratio = (iter2->first-s) / (iter2->first-iter1->first);
  return interpolateTransform(iter1->second, iter2->second, ratio);
}

const std::vector<DiscretePath::CarlaTransform> DiscretePath::samples() const {
  std::vector<CarlaTransform> samples;
  for (const auto& sample : samples_) samples.push_back(sample.second);
  return samples;
}

} // End namespace planner.

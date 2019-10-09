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
#include <stdexcept>
#include <algorithm>
#include <boost/format.hpp>
#include <conformal_lattice_planner/vehicle_path.h>
#include <conformal_lattice_planner/utils.h>

namespace planner {

NonHolonomicPath::State VehiclePath::carlaTransformToPathState(
    const std::pair<CarlaTransform, double>& transform) const {
  const CarlaTransform transform_rh = utils::convertTransform(transform.first);
  return NonHolonomicPath::State(transform_rh.location.x,
                                 transform_rh.location.y,
                                 transform_rh.rotation.yaw / 180.0 * M_PI,
                                 -transform.second);
}

std::pair<carla::geom::Transform, double> VehiclePath::pathStateToCarlaTransform(
    const NonHolonomicPath::State& state,
    const CarlaTransform& base_transform) const {

  // The input state only tells x, y, and theta/yaw.
  // We will get the rest from the base transform.
  CarlaTransform transform = utils::convertTransform(base_transform);
  transform.location.x = state.x;
  transform.location.y = state.y;
  transform.rotation.yaw = state.theta / M_PI * 180.0;

  // Convert to left handed coordinates.
  utils::convertTransformInPlace(transform);
  return std::make_pair(transform, -state.kappa);
}

std::pair<carla::geom::Transform, double> VehiclePath::interpolateTransform(
    const std::pair<CarlaTransform, double>& t1,
    const std::pair<CarlaTransform, double>& t2,
    const double w) const {

  if (w < 0.0 || w > 1.0) {
    throw std::runtime_error(
        "Weight for the first transform should be within the range of [0, 1].");
  }

  auto unrollAngle = [](const double angle)->double{
    return std::remainder(angle, 360.0);
  };

  const CarlaTransform& ct1 = t1.first;
  const CarlaTransform& ct2 = t2.first;
  CarlaTransform ct;
  ct.location       = ct1.location*w + ct2.location*(1.0-w);
  ct.rotation.roll  = unrollAngle(ct1.rotation.roll) *w + unrollAngle(ct2.rotation.roll) *(1.0-w);
  ct.rotation.pitch = unrollAngle(ct1.rotation.pitch)*w + unrollAngle(ct2.rotation.pitch)*(1.0-w);
  ct.rotation.yaw   = unrollAngle(ct1.rotation.yaw)  *w + unrollAngle(ct2.rotation.yaw)  *(1.0-w);

  const double c = t1.second*w + t2.second*(1.0-w);

  return std::make_pair(ct, c);
}
const std::vector<std::pair<carla::geom::Transform, double>>
VehiclePath::samples() const {

  double s = 0.0;
  std::vector<std::pair<CarlaTransform, double>> samples;
  for (; s < range(); s+=0.1)
    samples.push_back(transformAt(s));

  if (s < range())
    samples.push_back(transformAt(range()));

  return samples;
}

ContinuousPath::ContinuousPath(
    const std::pair<CarlaTransform, double>& start,
    const std::pair<CarlaTransform, double>& end,
    const LaneChangeType& lane_change_type) :
  Base  (lane_change_type),
  start_(start),
  end_  (end) {

  // Convert the start and end to right handed coordinate system.
  const NonHolonomicPath::State start_state = carlaTransformToPathState(start_);
  const NonHolonomicPath::State end_state = carlaTransformToPathState(end_);

  const bool success = path_.optimizePath(start_state, end_state);
  //std::printf("path a:%f b:%f c:%f d:%f sf:%f\n", path_.a, path_.b, path_.c, path_.d, path_.sf);

  if (!success) {
    std::printf("start state x:%f y:%f theta:%f kappa:%f\n",
        start_state.x, start_state.y, start_state.theta, start_state.kappa);
    std::printf("end state x:%f y:%f theta:%f kappa:%f\n",
        end_state.x, end_state.y, end_state.theta, end_state.kappa);
    throw std::runtime_error("Path optimization diverges.\n");
  }
  return;
}

ContinuousPath::ContinuousPath(const DiscretePath& discrete_path) :
  Base  (discrete_path.laneChangeType()),
  start_(discrete_path.startTransform()),
  end_  (discrete_path.endTransform()) {

  // Convert the start and end to right handed coordinate system.
  const NonHolonomicPath::State start_state = carlaTransformToPathState(start_);
  const NonHolonomicPath::State end_state = carlaTransformToPathState(end_);

  const bool success = path_.optimizePath(start_state, end_state);
  if (!success) {
    std::printf("start state x:%f y:%f theta:%f kappa:%f\n",
        start_state.x, start_state.y, start_state.theta, start_state.kappa);
    std::printf("end state x:%f y:%f theta:%f kappa:%f\n",
        end_state.x, end_state.y, end_state.theta, end_state.kappa);
    throw std::runtime_error("Path optimization diverges.\n");
  }
  return;
}

const std::pair<ContinuousPath::CarlaTransform, double>
ContinuousPath::transformAt(const double s) const {

  if (s < 0.0 || s > path_.sf) {
    std::printf("s:%f sf:%f\n", s, path_.sf);
    throw std::out_of_range("The input distance is out of the range of the path.");
  }

  const NonHolonomicPath::State start_state = carlaTransformToPathState(start_);
  const NonHolonomicPath::State state = path_.evaluate(start_state, s);

  // Generate a base transform by interpolating start and end.
  const double ratio = s / path_.sf;
  std::pair<CarlaTransform, double> base_transform =
    interpolateTransform(start_, end_, 1.0-ratio);

  return pathStateToCarlaTransform(state, base_transform.first);
}

std::string ContinuousPath::string(const std::string& prefix) const {
  boost::format transform_format("x:%1% y:%2% yaw:%3% curvature:%4%\n");
  std::string output = prefix;
  output += "start: ";
  output += (transform_format % start_.first.location.x
                              % start_.first.location.y
                              % start_.first.rotation.yaw
                              % start_.second).str();
  output += "end: ";
  output += (transform_format % end_.first.location.x
                              % end_.first.location.y
                              % end_.first.rotation.yaw
                              % end_.second).str();

  boost::format path_format("a:%1% b:%2% c:%3% d:%4% sf:%5%\n");
  output += "path: ";
  output += (path_format % path_.a % path_.b % path_.c % path_.d % path_.sf).str();
  return output;
}

DiscretePath::DiscretePath(
    const std::pair<CarlaTransform, double>& start,
    const std::pair<CarlaTransform, double>& end,
    const LaneChangeType& lane_change_type) :
  Base(lane_change_type) {

  // Convert the start and end to right handed coordinate system.
  const NonHolonomicPath::State start_state = carlaTransformToPathState(start);
  const NonHolonomicPath::State end_state = carlaTransformToPathState(end);

  // Compute the Kelly-Navy path.
  NonHolonomicPath path;
  const bool success = path.optimizePath(start_state, end_state);
  if (!success) {
    std::printf("start state x:%f y:%f theta:%f kappa:%f\n",
        start_state.x, start_state.y, start_state.theta, start_state.kappa);
    std::printf("end state x:%f y:%f theta:%f kappa:%f\n",
        end_state.x, end_state.y, end_state.theta, end_state.kappa);
    throw std::runtime_error("Path optimization diverges.\n");
  }

  // Sample the path with 0.1m resolution.
  double s = 0.0;
  for (; s <= path.sf; s += 0.1) {
    const double ratio = s / path.sf;
    std::pair<CarlaTransform, double> base_transform =
      interpolateTransform(start, end, 1.0-ratio);

    NonHolonomicPath::State state = path.evaluate(start_state, s);
    samples_[s] = pathStateToCarlaTransform(state, base_transform.first);
  }

  std::printf("Check the end of the path.\n");
  if (s < path.sf) {
    NonHolonomicPath::State state = path.evaluate(start_state, path.sf);
    samples_[path.sf] = pathStateToCarlaTransform(state, end.first);
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

const std::pair<DiscretePath::CarlaTransform, double>
DiscretePath::transformAt(const double s) const {

  if (s < 0.0 || s > range()) {
    std::printf("s:%f sf:%f\n", s, range());
    throw std::out_of_range("The input distance is out of the range of the path.");
  }

  if (s == 0.0) return samples_.begin()->second;
  if (s == samples_.rbegin()->first) return samples_.rbegin()->second;

  auto iter = std::find_if(samples_.begin(), samples_.end(),
      [&s](const std::pair<double, std::pair<CarlaTransform, double>>& sample)->bool{
        return sample.first > s;
      });

  if (iter==samples_.begin() || iter==samples_.end())
    throw std::runtime_error("The input distance is out of the range of the path.");

  auto iter1 = --iter; ++iter;
  auto iter2 = iter;
  const double ratio = (iter2->first-s) / (iter2->first-iter1->first);
  return interpolateTransform(iter1->second, iter2->second, ratio);
}

const std::vector<std::pair<DiscretePath::CarlaTransform, double>>
DiscretePath::samples() const {
  std::vector<std::pair<CarlaTransform, double>> samples;
  for (const auto& sample : samples_) samples.push_back(sample.second);
  return samples;
}

void DiscretePath::append(const DiscretePath& path) {
  // Check if the start of the input path matches the end of this path.
  // Only location is compared.
  const double gap = (endTransform().first.location -
                      path.startTransform().first.location).Length();
  std::printf("gap: %f\n", gap);
  if (gap > 0.1) throw std::runtime_error("gap > 0.1m");

  // Append the samples in the input path to this path.
  // The first sample in the input path should be ignored.
  const double offset = (--samples_.end())->first;
  for (auto iter = ++(path.samples_.begin());
       iter != path.samples_.end(); ++iter) {
    samples_[iter->first+offset] = iter->second;
  }

  return;
}

std::string DiscretePath::string(const std::string& prefix) const {
  boost::format transform_format("x:%1% y:%2% yaw:%3% curvature:%4%\n");
  std::string output = prefix;
  output += "start: ";
  const std::pair<CarlaTransform, double> start = startTransform();
  output += (transform_format % start.first.location.x
                              % start.first.location.y
                              % start.first.rotation.yaw
                              % start.second).str();
  output += "end: ";
  const std::pair<CarlaTransform, double> end = endTransform();
  output += (transform_format % end.first.location.x
                              % end.first.location.y
                              % end.first.rotation.yaw
                              % end.second).str();

  output += (boost::format("path: range:%1% sample size:%2%\n") % range() % samples_.size()).str();
  return output;
}

} // End namespace planner.

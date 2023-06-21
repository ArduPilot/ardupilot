/**
 * @file speed_data.cc
 **/

#include "speed_data.h"

#include <algorithm>
#include <utility>

#include "linear_interpolation.h"


namespace planning {

SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
    : speed_vector_(std::move(speed_points)) {}

void SpeedData::AppendSpeedPoint(const float s, const float time,
                                 const float v, const float a,
                                 const float da) {
  if (!speed_vector_.empty()) {
    if (speed_vector_.back().t() < time) {
        speed_vector_.push_back(SpeedPoint(s, time, v, a, da));
        return;
    }
  }
  speed_vector_.push_back(SpeedPoint(s, time, v, a, da));
}

const std::vector<SpeedPoint>& SpeedData::speed_vector() const {
  return speed_vector_;
}

void SpeedData::set_speed_vector(std::vector<SpeedPoint> speed_points) {
  speed_vector_ = std::move(speed_points);
}

bool SpeedData::EvaluateByTime(const float t,
                               SpeedPoint* const speed_point) const {
  if (speed_vector_.size() < 2) {
    return false;
  }
  if (!(speed_vector_.front().t() < t + 1.0e-6 &&
        t - 1.0e-6 < speed_vector_.back().t())) {
    return false;
  }

  auto comp = [](const SpeedPoint& sp, const float t0) {
    return sp.t() < t0;
  };

  auto it_lower =
      std::lower_bound(speed_vector_.begin(), speed_vector_.end(), t, comp);
  if (it_lower == speed_vector_.end()) {
    *speed_point = speed_vector_.back();
  } else if (it_lower == speed_vector_.begin()) {
    *speed_point = speed_vector_.front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    float t0 = p0.t();
    float t1 = p1.t();

    float s = lerp(p0.s(), t0, p1.s(), t1, t);
    float v = lerp(p0.v(), t0, p1.v(), t1, t);
    float a = lerp(p0.a(), t0, p1.a(), t1, t);
    float j = lerp(p0.da(), t0, p1.da(), t1, t);

    *speed_point = SpeedPoint(s, t, v, a, j);
  }
  return true;
}

float SpeedData::TotalTime() const {
  if (speed_vector_.empty()) {
    return 0.0;
  }
  return speed_vector_.back().t() - speed_vector_.front().t();
}

void SpeedData::Clear() { speed_vector_.clear(); }


}  // namespace planning


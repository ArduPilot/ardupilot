#include "discretized_traj.h"
#include "linear_interpolation.h"
#include <algorithm>

namespace planning {

TrajPoint linear_interpolate_trajectory(const TrajPoint &p0, const TrajPoint &p1, const float s)
{
    const float s0 = p0.s;
    const float s1 = p1.s;
    if (std::abs(s1  - s0) < planning::kMathEpsilon) {
        return p0;
    }

    TrajPoint pt;
    const float weight = (s - s0) / (s1 - s0);
    pt.s = s;
    pt.x = (1 - weight) * p0.x + weight * p1.x;
    pt.y = (1 - weight) * p0.y + weight * p1.y;
    pt.theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
    pt.kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
    pt.velocity = (1 - weight) * p0.velocity + weight * p1.velocity;
    pt.left_bound = (1 - weight) * p0.left_bound + weight * p1.left_bound;
    pt.right_bound = (1 - weight) * p0.right_bound + weight * p1.right_bound;
    return pt;
}


DiscretizedTraj::DiscretizedTraj(const DiscretizedTraj &rhs, int16_t begin, int16_t end)
{
    if (end < 0) {
        end = rhs._data.size();
    }
    _data.resize(end - begin);

    std::copy_n(std::next(rhs._data.begin(), begin), _data.size(), _data.begin());
}

Trajectory::const_iterator DiscretizedTraj::query_lower_bound_station_point(const float station) const
{
    if (station >= _data.back().s) {
        return _data.end() - 1;
    } else if (station < _data.front().s) {
        return _data.begin();
    }

  return std::lower_bound(
    _data.begin(), _data.end(), station,
    [](const TrajPoint &t, float s) {
      return t.s < s;
    });
}

Trajectory::const_iterator DiscretizedTraj::query_nearest_point(const Vec2d &point, float *out_distance) const
{
    auto nearest_iter = _data.begin();
    float nearest_distance = std::numeric_limits<float>::max();
    for (auto iter = _data.begin(); iter != _data.end(); iter++) {
        const float dx = iter->x - point.x();
        const float dy = iter->y - point.y();
        const float distance = dx * dx  + dy * dy;
        if (distance < nearest_distance) {
            nearest_iter = iter;
            nearest_distance = distance;
        }
    }

    if (out_distance != nullptr) {
        *out_distance = sqrtf(nearest_distance);
    }
    return nearest_iter;
}

TrajPoint DiscretizedTraj::evaluate_station(const float station) const
{
    auto iter = query_lower_bound_station_point(station);

    if (iter  == _data.begin()) {
        iter = std::next(iter);
    }

    auto prev = std::prev(iter, 1);
    return linear_interpolate_trajectory(*prev, *iter, station);
}

Vec2d DiscretizedTraj::get_projection(const Vec2d &xy) const
{
    unsigned long point_idx = std::distance(_data.begin(), query_nearest_point(xy));
    auto project_point = _data[point_idx];
    unsigned long index_start = std::max((unsigned long)0, point_idx - 1);
    unsigned long index_end =  (unsigned long)point_idx + 1;
    if ((index_end + 1) >= _data.size()) {
        index_end = _data.size() - 1;
    }

    if (index_start < index_end) {
        const float v0x = xy.x() - _data[index_start].x;
        const float v0y = xy.y() - _data[index_start].y;

        const float v1x = _data[index_end].x - _data[index_start].x;
        const float v1y = _data[index_end].y - _data[index_start].y;

        const float v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
        const float dot = v0x * v1x + v0y * v1y;

        const float delta_s = dot / v1_norm;
        project_point = linear_interpolate_trajectory(_data[index_start], _data[index_end], _data[index_start].s + delta_s);
    }

  const float nr_x = xy.x() - project_point.x, nr_y = xy.y() - project_point.y;
  const float lateral = copysign(hypotf(nr_x, nr_y), nr_y * cosf(project_point.theta) - nr_x * sinf(project_point.theta));
  return {project_point.s, lateral};
}

Vec2d DiscretizedTraj::get_cartesian(const float station, const float lateral) const
{
    auto ref = evaluate_station(station);
    return {ref.x - lateral * sinf(ref.theta), ref.y + lateral * cosf(ref.theta)};
}
}
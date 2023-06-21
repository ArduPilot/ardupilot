#include "speed_limit.h"

#include <algorithm>
#include <float.h>

namespace planning {

void SpeedLimit::append_speed_limit(const float s, const float v)
{
    if (!_speed_limit_points.empty()) {
        if (s > _speed_limit_points.back().first) {
            _speed_limit_points.emplace_back(s,v);
            return;
        }
    }

    _speed_limit_points.emplace_back(s,v);
}

// use const xxxx const, means can not change content inner or outer
const std::vector<std::pair<float, float>>& SpeedLimit::speed_limit_points() const
{
    return _speed_limit_points;
}

float SpeedLimit::get_speed_limit_by_s(const float s) const
{
    // some checks
    if (_speed_limit_points.size() < 2) {
        return FLT_MAX;
    }

    if (s > _speed_limit_points.front().first) {
        return FLT_MAX;
    }

    // lamada expression must be add ;
    auto compare_s = [](const std::pair<float, float>& point, const float s0) {
        return point.first < s0;
    };

    auto it_lower = std::lower_bound(_speed_limit_points.begin(), _speed_limit_points.end(), s, compare_s);

    if (it_lower == _speed_limit_points.end()) {
        return (it_lower - 1)->second;
    } 
    return it_lower->second;
}

void SpeedLimit::clear()
{
    _speed_limit_points.clear();
}

} // namespace planning
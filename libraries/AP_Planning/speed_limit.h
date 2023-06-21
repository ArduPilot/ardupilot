#pragma once

#include <utility>
#include <vector>

namespace planning {

class SpeedLimit {
public:
    SpeedLimit() = default;

    // add a (s,v) point to S-V list
    void append_speed_limit(const float s, const float v);

    // return S-V list
    const std::vector<std::pair<float, float>>& speed_limit_points() const;

    // return v when give in s
    float get_speed_limit_by_s(const float s) const;

    // clear S-V list
    void clear();

private:
    // use a vector to represent speed limit
    // the first number is s, the second number is v
    // it means at distance s from the start point, the speed limit is v
    std::vector<std::pair<float, float>> _speed_limit_points; 
};
} //  namespace planning
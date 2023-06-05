#pragma once

#include <utility>
#include <vector>
#include <cassert>
#include "vec2d.h"

namespace planning {
struct TrajPoint {
    float s{0.0f};
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};
    float kappa{0.0f};
    float velocity{0.0f};
    float left_bound{0.0f};
    float right_bound{0.0f};
};

typedef std::vector<TrajPoint> Trajectory;

TrajPoint linear_interpolate_trajectory(const TrajPoint &p0, const TrajPoint &p1, const float s);

/**
 * @class DiscretizedTraj
 */
class DiscretizedTraj {
    public:
        DiscretizedTraj() = default;

        DiscretizedTraj(const DiscretizedTraj &rhs, int16_t begin, int16_t end = -1);

        explicit DiscretizedTraj(Trajectory points) : _data(std::move(points)) {}

        inline const Trajectory &data() const { return _data; }

        Trajectory::const_iterator query_lower_bound_station_point(const float station) const;

        Trajectory::const_iterator query_nearest_point(const Vec2d &point, float *out_distance = nullptr) const;

        TrajPoint evaluate_station(const float station) const;

        Vec2d get_projection(const Vec2d &xy) const;

        Vec2d get_cartesian(const float station, const float lateral) const;

    private:
        Trajectory _data;
};
}
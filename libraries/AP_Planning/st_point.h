#pragma once

#include <string>

#include "vec2d.h"

namespace planning {

class STPoint : public Vec2d {
public:
    STPoint() = default;
    STPoint(const float s, const float t);
    explicit STPoint(const Vec2d& vec2d_point);

    float s() const;
    float t() const;
    void set_s(const float s);
    void set_t(const float t);
};
} // namespace planning
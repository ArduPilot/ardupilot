#pragma once

#include "vec2d.h"

namespace planning {
class Pose {
public:
    Pose() = default;
    Pose(float x, float y, float theta): x_(x), y_(y), theta_(theta) {}

    float x() const { return x_; }
    float y() const { return y_; }
    float theta() const { return theta_; }

    void x(float x) { x_ = x; }
    void y(float y) { y_ = y; }
    void theta(float theta) { theta_ = theta; }

    operator Vec2d() const { return { x_, y_}; }

    Pose relativeTo(const Pose &coord) const;

    Pose extend(const float length) const;

    Pose transform(const Pose &relative) const;

private:
    float x_, y_, theta_;
};
}
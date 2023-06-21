#include "pose.h"

namespace planning {
Pose Pose::relativeTo(const Pose &coord) const
{
    const float dx = x_ - coord.x();
    const float dy = y_ - coord.y();
    return {
        dx * cosf(coord.theta()) + dy * sinf(coord.theta()),
        -dx * sinf(coord.theta()) + dy * cosf(coord.theta()),
        theta_ - coord.theta()
    };
}

Pose Pose::extend(const float length) const
{
    return transform({length , 0, 0});
}

Pose Pose::transform(const Pose &relative) const
{
    return {
      x_ + relative.x() * cosf(theta_) - relative.y() * sinf(theta_),
      y_ + relative.x() * sinf(theta_) + relative.y() * cosf(theta_),
      theta_ + relative.theta_
    };
}
}
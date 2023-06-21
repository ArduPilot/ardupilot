#include "publishable_trajectory.h"

namespace planning {
    PublishableTrajectory::PublishableTrajectory(const float header_time, const DiscretizedTrajectory& discretized_trajectory)
    : DiscretizedTrajectory(discretized_trajectory),
      header_time_(header_time) {}

    float PublishableTrajectory::header_time() const { return header_time_; }
}
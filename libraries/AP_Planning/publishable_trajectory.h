#pragma once

#include "discretized_trajectory.h"

namespace planning {

    class PublishableTrajectory : public DiscretizedTrajectory {
    public:
        PublishableTrajectory() = default;

        PublishableTrajectory(const float header_time,
                                const DiscretizedTrajectory& discretized_trajectory);

        float header_time() const;

    private:
        float header_time_ = 0.0;
    };

}  // namespace planning
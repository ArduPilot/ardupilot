#pragma once

#include <string>
#include <utility>
#include <vector>

#include "publishable_trajectory.h"

namespace planning {
    struct VehicleState {
        float x;
        float y;
        float z;
        float timestamp;
        float heading;
        float kappa;
        float linear_velocity;
        float angular_velocity;
        float linear_acceleration;
    };

    class TrajectoryStitcher {
    public:
        TrajectoryStitcher() = delete;

        static void TransformLastPublishedTrajectory(
            const float x_diff, const float y_diff, const float theta_diff,
            PublishableTrajectory* prev_trajectory);

        static std::vector<TrajectoryPoint> ComputeStitchingTrajectory(
            const VehicleState& vehicle_state, const float current_timestamp,
            const float planning_cycle_time, const size_t preserved_points_num,
            const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
            const bool FLAGS_enable_trajectory_stitcher);

        static std::vector<TrajectoryPoint> ComputeReinitStitchingTrajectory(
            const float planning_cycle_time,
            const VehicleState& vehicle_state);

    private:
        static std::pair<float, float> ComputePositionProjection(
            const float x, const float y,
            const TrajectoryPoint& matched_trajectory_point);

        static TrajectoryPoint ComputeTrajectoryPointFromVehicleState(
            const float planning_cycle_time,
            const VehicleState& vehicle_state);
    };

    VehicleState Predict(const float predicted_time_horizon,
                        const VehicleState& cur_vehicle_state);

}  // namespace planning


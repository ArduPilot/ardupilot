#include "trajectory_stitcher.h"
#include "math_utils.h"

#include <algorithm>

namespace planning {

VehicleState Predict(const float predicted_time_horizon, const VehicleState& cur_vehicle_state)
{
    VehicleState predicted_vehicle_state = cur_vehicle_state;
    // Kinematic bicycle model centered at rear axis center by Euler forward
    // discretization
    // Assume constant control command and constant z axis position
    if (predicted_time_horizon <= 0) {
        return cur_vehicle_state;
    }

    float dt = 0.1f;
    float cur_x = cur_vehicle_state.x;
    float cur_y = cur_vehicle_state.y;
    float cur_z = cur_vehicle_state.z;
    float cur_phi = cur_vehicle_state.heading;
    float cur_v = cur_vehicle_state.linear_velocity;
    float cur_a = cur_vehicle_state.linear_acceleration;
    float next_x = cur_x;
    float next_y = cur_y;
    float next_phi = cur_phi;
    float next_v = cur_v;
    if (dt >= predicted_time_horizon) {
        dt = predicted_time_horizon;
    }

    float countdown_time = predicted_time_horizon;
    bool finish_flag = false;
    static constexpr float kepsilon = 1e-8;
    while (countdown_time > kepsilon && !finish_flag) {
        countdown_time -= dt;
        if (countdown_time < kepsilon) {
            dt = countdown_time + dt;
            finish_flag = true;
        }

        float intermidiate_phi = cur_phi + 0.5 * dt * cur_v * cur_vehicle_state.kappa;
        next_phi = cur_phi + dt * (cur_v + 0.5 * dt * cur_a) * cur_vehicle_state.kappa;
        next_x = cur_x + dt * (cur_v + 0.5 * dt * cur_a) * std::cos(intermidiate_phi);
        next_y = cur_y + dt * (cur_v + 0.5 * dt * cur_a) * std::sin(intermidiate_phi);

        next_v = cur_v + dt * cur_a;
        cur_x = next_x;
        cur_y = next_y;
        cur_phi = next_phi;
        cur_v = next_v;
    }

    predicted_vehicle_state.x = next_x;
    predicted_vehicle_state.y = next_y;
    predicted_vehicle_state.z = cur_z;
    predicted_vehicle_state.heading = next_phi;
    predicted_vehicle_state.kappa = cur_vehicle_state.kappa;
    predicted_vehicle_state.linear_velocity = next_v;
    predicted_vehicle_state.linear_acceleration = cur_vehicle_state.linear_acceleration;
    return predicted_vehicle_state;
}



TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const float planning_cycle_time, const VehicleState& vehicle_state) {
        TrajectoryPoint point;
        point.path_point.s = 0;
        point.path_point.x = vehicle_state.x;
        point.path_point.y = vehicle_state.y;
        point.path_point.z = vehicle_state.z;
        point.path_point.theta = vehicle_state.heading;
        point.path_point.kappa = vehicle_state.kappa;
        point.v = vehicle_state.linear_velocity;
        point.a = vehicle_state.linear_acceleration;
        point.relative_time = planning_cycle_time;
        return point;
    }


std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const float planning_cycle_time, const VehicleState& vehicle_state) {
        TrajectoryPoint reinit_point;
        static constexpr float kEpsilon_v = 0.1;
        static constexpr float kEpsilon_a = 0.4;
        // TODO: adjust kEpsilon if corrected IMU acceleration provided
        if (std::abs(vehicle_state.linear_velocity) < kEpsilon_v &&
            std::abs(vehicle_state.linear_acceleration) < kEpsilon_a) {
            reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                                  vehicle_state);
        } else {
            VehicleState predicted_vehicle_state;
            predicted_vehicle_state = Predict(planning_cycle_time, vehicle_state);
            reinit_point = ComputeTrajectoryPointFromVehicleState(
                planning_cycle_time, predicted_vehicle_state);
        }

  return std::vector<TrajectoryPoint>(1, reinit_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const float x_diff, const float y_diff, const float theta_diff,
        PublishableTrajectory* prev_trajectory) {
        if (!prev_trajectory) {
            return;
        }

        // R^-1
        float cos_theta = std::cos(theta_diff);
        float sin_theta = -std::sin(theta_diff);

        // -R^-1 * t
        auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
        auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

    std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
        [&cos_theta, &sin_theta, &tx, &ty, &theta_diff](TrajectoryPoint& p) {
            auto x = p.path_point.x;
            auto y = p.path_point.y;
            auto theta = p.path_point.theta;

            auto x_new = cos_theta * x - sin_theta * y + tx;
            auto y_new = sin_theta * x + cos_theta * y + ty;
            auto theta_new = NormalizeAngle(theta - theta_diff);

            p.path_point.x = x_new;
            p.path_point.y = y_new;
            p.path_point.theta = theta_new;
        });
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const float current_timestamp,
    const float planning_cycle_time, const size_t preserved_points_num,
    const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
    const bool FLAGS_enable_trajectory_stitcher) {
        if (!FLAGS_enable_trajectory_stitcher) {
            return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
        }
        if (!prev_trajectory) {
            return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
        }

        size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

        if (prev_trajectory_size == 0) {
            return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
        }

        // td = t -tp
        const float veh_rel_time =
            current_timestamp - prev_trajectory->header_time();

        // find the position of current time in the last planned track
        size_t time_matched_index =
            prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

        if (time_matched_index == 0 &&
            veh_rel_time < prev_trajectory->StartPoint().relative_time) {
            return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
        }
        if (time_matched_index + 1 >= prev_trajectory_size) {
            return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
        }

        // get position at last planned track
        auto time_matched_point = prev_trajectory->TrajectoryPointAt(
            static_cast<uint32_t>(time_matched_index));

        // find the postion of current position on the planned trajectory of the previous cycle
        size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
            {vehicle_state.x, vehicle_state.y}, 1.0e-6);

        auto frenet_sd = ComputePositionProjection(
            vehicle_state.x, vehicle_state.y,
            prev_trajectory->TrajectoryPointAt(
                static_cast<uint32_t>(position_matched_index)));

        if (replan_by_offset) {
            auto lon_diff = time_matched_point.path_point.s - frenet_sd.first;
            auto lat_diff = frenet_sd.second;

            // lat error > 0.5m
            if (std::fabs(lat_diff) > 0.5f) {
                return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                        vehicle_state);
            }

            // lon error > 2.5m
            if (std::fabs(lon_diff) > 2.5f) {
                return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                    vehicle_state);
            }
        } 

        // The planning cycle represents the period of time, and the robot has moved for same time
        float forward_rel_time = veh_rel_time + planning_cycle_time;

        size_t forward_time_index =
            prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

        auto matched_index = std::min(time_matched_index, position_matched_index);

        std::vector<TrajectoryPoint> stitching_trajectory(
            prev_trajectory->begin() +
                std::max(0, static_cast<int>(matched_index - preserved_points_num)),
            prev_trajectory->begin() + forward_time_index + 1);

        const float zero_s = stitching_trajectory.back().path_point.s;
        for (auto& tp : stitching_trajectory) {
            tp.relative_time = tp.relative_time + prev_trajectory->header_time() -
                                current_timestamp;
            tp.path_point.s = tp.path_point.s - zero_s;
        }
        return stitching_trajectory;
}

std::pair<float, float> TrajectoryStitcher::ComputePositionProjection(
    const float x, const float y, const TrajectoryPoint& p) {
        Vec2d v(x - p.path_point.x, y - p.path_point.y);
        Vec2d n(std::cos(p.path_point.theta), std::sin(p.path_point.theta));

        std::pair<float, float> frenet_sd;
        frenet_sd.first = v.InnerProd(n) + p.path_point.s;
        frenet_sd.second = v.CrossProd(n);
        return frenet_sd;
}

}
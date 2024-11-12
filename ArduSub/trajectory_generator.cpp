#include "trajectory_generator.h"
#include <algorithm>
#include <cmath>
#include <iostream>

// Constructor
TrajectoryGenerator::TrajectoryGenerator() {
    // Initialize with an empty trajectory
    trajectory_data.clear();
}

// Set manually defined trajectory points
void TrajectoryGenerator::set_manual_trajectory(const std::vector<TrajectoryPoint>& trajectory_points) {
    trajectory_data = trajectory_points;
}

// Generate a linear trajectory between two points over a specified duration
std::vector<TrajectoryPoint> TrajectoryGenerator::generate_linear_trajectory(const std::vector<double>& start_position,
                                                                             const std::vector<double>& end_position,
                                                                             double duration, double step_size) {
    std::vector<TrajectoryPoint> generated_trajectory;
    int steps = static_cast<int>(duration / step_size);
    for (int i = 0; i <= steps; ++i) {
        double alpha = static_cast<double>(i) / steps;
        std::vector<double> position;
        for (size_t j = 0; j < start_position.size(); ++j) {
            position.push_back(start_position[j] + alpha * (end_position[j] - start_position[j]));
        }
        TrajectoryPoint point;
        point.time = i * step_size;
        point.position = position;
        generated_trajectory.push_back(point);
    }
    return generated_trajectory;
}

// Generate a sinusoidal trajectory
std::vector<TrajectoryPoint> TrajectoryGenerator::generate_sinusoidal_trajectory(const std::vector<double>& amplitude,
                                                                                 const std::vector<double>& frequency,
                                                                                 const std::vector<double>& phase,
                                                                                 double duration, double step_size) {
    std::vector<TrajectoryPoint> generated_trajectory;
    int steps = static_cast<int>(duration / step_size);
    for (int i = 0; i <= steps; ++i) {
        double current_time = i * step_size;
        std::vector<double> position;
        for (size_t j = 0; j < amplitude.size(); ++j) {
            double value = amplitude[j] * std::sin(2 * M_PI * frequency[j] * current_time + phase[j]);
            position.push_back(value);
        }
        TrajectoryPoint point;
        point.time = current_time;
        point.position = position;
        generated_trajectory.push_back(point);
    }
    return generated_trajectory;
}

// Update the trajectory points with new data
void TrajectoryGenerator::update_trajectory(const std::vector<TrajectoryPoint>& new_trajectory_points) {
    trajectory_data = new_trajectory_points;
}

// Get the desired trajectory point at a given time
TrajectoryPoint TrajectoryGenerator::get_trajectory_at_time(double current_time) {
    if (trajectory_data.empty()) {
        std::cerr << "Error: Trajectory data is empty." << std::endl;
        return TrajectoryPoint{0.0, {0.0, 0.0, 0.0}};
    }

    // Find the trajectory point closest to the given time
    auto it = std::lower_bound(trajectory_data.begin(), trajectory_data.end(), current_time,
                               [](const TrajectoryPoint& point, double time) {
                                   return point.time < time;
                               });

    if (it == trajectory_data.end()) {
        return trajectory_data.back();
    }
    if (it == trajectory_data.begin()) {
        return *it;
    }

    // Perform linear interpolation between the two closest points
    auto prev_it = std::prev(it);
    double alpha = (current_time - prev_it->time) / (it->time - prev_it->time);
    TrajectoryPoint interpolated_point;
    interpolated_point.time = current_time;
    for (size_t i = 0; i < prev_it->position.size(); ++i) {
        interpolated_point.position.push_back(prev_it->position[i] + alpha * (it->position[i] - prev_it->position[i]));
    }
    return interpolated_point;
}

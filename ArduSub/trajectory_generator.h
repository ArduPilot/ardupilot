#pragma once

#include <vector>
#include <functional>

// Define a structure to hold a trajectory point
struct TrajectoryPoint {
    double time;                       // Time in seconds
    std::vector<double> position;      // Position at the given time (x, y, z, etc.)
};

class TrajectoryGenerator {
public:
    // Constructor
    TrajectoryGenerator();

    // Set manually defined trajectory points
    void set_manual_trajectory(const std::vector<TrajectoryPoint>& trajectory_points);

    // Generate a linear trajectory between two points over a specified duration
    std::vector<TrajectoryPoint> generate_linear_trajectory(const std::vector<double>& start_position,
                                                            const std::vector<double>& end_position,
                                                            double duration, double step_size);

    // Generate a sinusoidal trajectory
    std::vector<TrajectoryPoint> generate_sinusoidal_trajectory(const std::vector<double>& amplitude,
                                                                const std::vector<double>& frequency,
                                                                const std::vector<double>& phase,
                                                                double duration, double step_size);

    // Update the trajectory points with new data
    void update_trajectory(const std::vector<TrajectoryPoint>& new_trajectory_points);

    // Get the desired trajectory point at a given time
    TrajectoryPoint get_trajectory_at_time(double current_time);

private:
    // Container to hold the trajectory data
    std::vector<TrajectoryPoint> trajectory_data;
};

// Configuration file for MPC
const std::string CONFIG_FILE_PATH = "mpc_config_file.yaml";

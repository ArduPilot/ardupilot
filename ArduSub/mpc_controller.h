#pragma once

#include <vector>
#include <functional>
#include <iostream>
#include "trajectory_generator.h"

class MPC_Controller {
public:
    // Constructor
    MPC_Controller();

    // Initialize the controller with system matrices, prediction horizon, and sampling time
    void init(int prediction_horizon, double sampling_time, 
              const std::vector<std::vector<double>>& Q, 
              const std::vector<std::vector<double>>& R, 
              const std::vector<std::vector<double>>& F);

    // 更新系统矩阵 A 和 B
    void update_system_matrices(const std::vector<std::vector<double>>& A_new,
                                const std::vector<std::vector<double>>& B_new);

    // 计算控制量
    void calculate_control(double& roll_output, double& pitch_output, 
                           double& yaw_output, double& throttle_output);

    // 增广矩阵解算函数
    void augmented_matrix_solver();

    // Set the desired state for the MPC to track
    void set_desired_state(const std::vector<double>& state);

    // Update the current state of the vehicle
    void update_current_state(const std::vector<double>& state);

    // Set control bounds for the outputs (min and max values)
    void set_control_bounds(const std::vector<double>& bounds);

    // Load configuration from file
    void load_config(const std::string& config_path);

private:
    // Helper function to clamp values within bounds
    double clamp_value(double value, double lower_bound, double upper_bound);

    // Current state of the vehicle
    std::vector<double> current_state;

    // Desired state for the vehicle to track
    std::vector<double> desired_state;

    // Control bounds for the outputs (min and max values)
    std::vector<double> control_bounds;  // {lower_bound, upper_bound}

    // System matrices for the model predictive control
    std::vector<std::vector<double>> A;  // 状态转移矩阵
    std::vector<std::vector<double>> B;  // 控制输入矩阵
    std::vector<std::vector<double>> Q;  // 状态成本矩阵
    std::vector<std::vector<double>> R;  // 控制成本矩阵
    std::vector<std::vector<double>> F;  // 增广矩阵

    int prediction_horizon;  // 预测步数
    double sampling_time;  // 采样时间

    // Trajectory generator instance
    TrajectoryGenerator trajectory_generator;
};

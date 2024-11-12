#include "mpc_controller.h"
#include <algorithm>
#include <iostream>

// Constructor for initializing the MPC controller
MPC_Controller::MPC_Controller() {
    // Initialize default values
    prediction_horizon = 10;
    sampling_time = 0.1;
    control_bounds = {-5.0, 5.0};
}

// Initialize the controller with system matrices, prediction horizon, and sampling time
void MPC_Controller::init(int prediction_horizon, double sampling_time,
                          const std::vector<std::vector<double>>& Q,
                          const std::vector<std::vector<double>>& R,
                          const std::vector<std::vector<double>>& F) {
    this->prediction_horizon = prediction_horizon;
    this->sampling_time = sampling_time;
    this->Q = Q;
    this->R = R;
    this->F = F;
}

// Update system matrices A and B
void MPC_Controller::update_system_matrices(const std::vector<std::vector<double>>& A_new,
                                            const std::vector<std::vector<double>>& B_new) {
    A = A_new;
    B = B_new;
}

// Augmented matrix solver for MPC
void MPC_Controller::augmented_matrix_solver() {
    // Define matrices E, M, H for MPC computation
    std::vector<std::vector<double>> E(prediction_horizon, std::vector<double>(6, 0.0));
    std::vector<std::vector<double>> M(prediction_horizon, std::vector<double>(6, 0.0));
    std::vector<std::vector<double>> H(prediction_horizon, std::vector<double>(6, 0.0));

    // Compute E, M, H based on A, B, Q, R, F
    for (int k = 0; k < prediction_horizon; ++k) {
        // E, M, H matrix calculations, usually involving linear algebra operations
        // Placeholder logic here, real logic depends on control requirements

        // This is a simple placeholder for the logic
        for (int i = 0; i < 6; ++i) {
            E[k][i] = A[i][i] * Q[i][i];
            M[k][i] = B[i][i] * R[i][i];
            H[k][i] = F[i][i];
        }
    }

    // Implement the rolling optimization step here to determine the desired acceleration output
    // Placeholder for optimization logic
}

// Calculate control outputs based on the desired state and current state
void MPC_Controller::calculate_control(double& roll_output, double& pitch_output,
                                       double& yaw_output, double& throttle_output) {
    // Run the augmented matrix solver to compute desired control
    augmented_matrix_solver();

    // Placeholder values for output
    roll_output = clamp_value(0.0, control_bounds[0], control_bounds[1]);
    pitch_output = clamp_value(0.0, control_bounds[0], control_bounds[1]);
    yaw_output = clamp_value(0.0, control_bounds[0], control_bounds[1]);
    throttle_output = clamp_value(0.0, control_bounds[0], control_bounds[1]);
}

// Set the desired state for the MPC to track
void MPC_Controller::set_desired_state(const std::vector<double>& state) {
    if (state.size() != 6) {
        std::cerr << "Error: Desired state must have 6 elements." << std::endl;
        return;
    }
    desired_state = state;
}

// Update the current state of the vehicle
void MPC_Controller::update_current_state(const std::vector<double>& state) {
    if (state.size() != 6) {
        std::cerr << "Error: Current state must have 6 elements." << std::endl;
        return;
    }
    current_state = state;
}

// Set control bounds for the outputs (min and max values)
void MPC_Controller::set_control_bounds(const std::vector<double>& bounds) {
    if (bounds.size() != 2) {
        std::cerr << "Error: Control bounds must have 2 elements." << std::endl;
        return;
    }
    control_bounds = bounds;
}

// Helper function to clamp values within bounds
double MPC_Controller::clamp_value(double value, double lower_bound, double upper_bound) {
    return std::max(lower_bound, std::min(value, upper_bound));
}

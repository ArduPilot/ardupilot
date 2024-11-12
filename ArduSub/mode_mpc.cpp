#include "modeMPC.h"
#include "Sub.h"
#include "AP_Motors/AP_Motors.h"
#include "AC_AttitudeControl/AC_AttitudeControl_Sub.h"
#include "yaml-cpp/yaml.h"  // 用于读取配置文件
#include <iostream>
#include <fstream>

// Initialize the MPC control mode
bool ModeMPC::init(bool ignore_checks)
{
    sub.set_neutral_controls();  // 确保设置中性控制

    // 从配置文件中读取 Q, R, F 矩阵
    try {
        YAML::Node config = YAML::LoadFile("mpc_config_file.yaml");
        std::vector<std::vector<double>> Q = config["Q"].as<std::vector<std::vector<double>>>();
        std::vector<std::vector<double>> R = config["R"].as<std::vector<std::vector<double>>>();
        std::vector<std::vector<double>> F = config["F"].as<std::vector<std::vector<double>>>();
        
        int prediction_horizon = config["prediction_horizon"].as<int>();
        double sampling_time = config["sampling_time"].as<double>();

        // 初始化 MPC 控制器
        sub.mpc_controller.init(prediction_horizon, sampling_time, Q, R, F);
        std::cout << "MPC controller initialized successfully." << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error loading MPC configuration: " << e.what() << std::endl;
        return false;
    }

    return true;
}

// Run the MPC controller
void ModeMPC::run()
{
    // 如果未解锁，则保持所有内容处于空闲状态
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        sub.attitude_control.set_throttle_out(0, true, sub.g.throttle_filt);
        sub.attitude_control.relax_attitude_controllers();
        return;
    }

    // 当已解锁时，将电机状态设置为油门无限制
    sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 更新 A 和 B 矩阵
    std::vector<std::vector<double>> A;
    std::vector<std::vector<double>> B;

    try {
        sub.mpc_controller.update_system_matrices(A, B);  // 动态获取 A 和 B 矩阵
    } catch (const std::exception &e) {
        std::cerr << "Error updating system matrices: " << e.what() << std::endl;
        return;
    }

    // 运行 MPC 控制算法以确定期望的电机输出
    double roll_output = 0.0, pitch_output = 0.0, yaw_output = 0.0, throttle_output = 0.0;
    try {
        sub.mpc_controller.calculate_control(roll_output, pitch_output, yaw_output, throttle_output);
    } catch (const std::exception &e) {
        std::cerr << "Error during MPC calculation: " << e.what() << std::endl;
        return;
    }

    // 验证输出是否在允许的范围内
    if (std::isnan(roll_output) || std::isnan(pitch_output) || std::isnan(yaw_output) || std::isnan(throttle_output)) {
        std::cerr << "Error: MPC controller output contains NaN values." << std::endl;
        return;
    }

    // 设置电机输出
    sub.motors.set_roll(static_cast<float>(roll_output));
    sub.motors.set_pitch(static_cast<float>(pitch_output));
    sub.motors.set_yaw(static_cast<float>(yaw_output));
    sub.motors.set_throttle(static_cast<float>(throttle_output));
}

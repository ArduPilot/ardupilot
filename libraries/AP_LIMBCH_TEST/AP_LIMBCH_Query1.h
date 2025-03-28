#pragma once

#include <AP_HAL/AP_HAL.h>
// #include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
// #include <../ArduCopter/GCS_Copter.h>


#include <iostream>
#include <vector>
#include <cstdint>


struct Query1Msg {
    // 字节 0: 字符长度
    uint8_t char_length;

    // 字节 1: 识别码
    uint8_t id_code;

    // 字节 2-3: 预留位
    uint16_t reserved_2_3;

    // 字节 4-5: 存储定义
    uint16_t store_def;

    // 字节 6-9: 预留位
    uint32_t reserved_6_9;

    // 字节 10-11: 发动机运行状态
    uint16_t engine_state;

    // 字节 12-13: 发动机转速
    uint16_t engine_rpm;

    // 字节 14-17: 预留位
    uint32_t reserved_14_17;

    // 字节 18-19: 距暖车结束时间
    uint16_t warmup_time_remaining;

    // 字节 20-23: 预留位
    uint32_t reserved_20_23;

    // 字节 24-25: 怠速位置设置
    uint16_t idle_position;

    // 字节 26-27: 干扰脉冲计数
    uint16_t interference_count;

    // 字节 28-29: 进气量
    uint16_t air_intake;

    // 字节 30-31: 进气压力
    uint16_t air_pressure;

    // 字节 32-33: 转速出错计数
    uint16_t rpm_error_count;

    // 字节 34-35: 喷射时间
    uint16_t injection_time;

    // 字节 36-37: 点火提前角
    uint16_t ignition_angle;

    // 字节 38-39: 预留位
    uint16_t reserved_38_39;

    // 字节 40-41: 节气门位置传感器电压
    uint16_t throttle_position_voltage;

    // 字节 42-43: 电池电量传感器电压
    uint16_t battery_voltage;

    // 字节 44-45: 空燃比传感器电压
    uint16_t afr_sensor_voltage;

    // 字节 46-47: 发动机温度传感器电压
    uint16_t engine_temp_voltage;

    // 字节 48-49: 进气温度传感器电压
    uint16_t intake_temp_voltage;

    // 字节 50-51: 外界气压传感器电压
    uint16_t external_pressure_voltage;

    // 字节 52-53: 进气压力传感器电压
    uint16_t intake_pressure_voltage;

    // 字节 54-55: 进气量传感器电压
    uint16_t intake_quantity_voltage;

    // 字节 56-57: 空燃比
    int16_t afr_value;

    // 字节 58-73: 预留位
    uint16_t reserved_58_73;

    // 字节 74-75: 节气门位置
    uint16_t throttle_position;

    // 字节 76-77: 发动机温度（冷却液温度 1）
    uint16_t engine_cooling_temp;

    // 字节 78-79: 蓄电池电压（ECU 工作电压）
    uint16_t ecu_battery_voltage;

    // 字节 80-81: 进气温度
    uint16_t intake_temp;

    // 字节 82-83: 进气压力
    uint16_t intake_pressure;

    // 字节 84-85: 传感器状况
    uint16_t sensor_status;

    // 字节 86: 检验字节
    uint8_t checksum;
};
    

class AP_LIMBCH_Query1 {
    public:
        AP_LIMBCH_Query1();
    
        /* Do not allow copies */
        AP_LIMBCH_Query1(const AP_LIMBCH_Query1 &other) = delete;
        AP_LIMBCH_Query1 &operator=(const AP_LIMBCH_Query1&) = delete;
    
        // init - perform require initialisation including detecting which protocol to use
        void init(const AP_SerialManager& serial_manager);
    
        // update flight control mode. The control mode is vehicle type specific
        bool send_request(void);
        Query1Msg accepting_request(void);
        
    private:
        AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
        std::vector<uint8_t> response_buffer; // 存储接收到的数据
};



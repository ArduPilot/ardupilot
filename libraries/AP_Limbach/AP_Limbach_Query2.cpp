#define AP_SERIALMANAGER_OPEN_BAUD 115200
#define AP_SERIALMANAGER_LIMBACH_BUFSIZE_RX 64
#define AP_SERIALMANAGER_LIMBACH_BUFSIZE_TX 64

#include "AP_Limbach_Query2.h"
#include <../libraries/AP_HAL/utility/functor.h>
#include <../ArduCopter/GCS_Copter.h>

AP_Limbach_Query2::AP_Limbach_Query2(void)
{
    _port = NULL;
}

void AP_Limbach_Query2::init(const AP_SerialManager &serial_manager)
{
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_LIMBACH, 0)))
    {

        // gcs().send_text(MAV_SEVERITY_CRITICAL, "AP_Limbach_Query2::init");

        _port->begin(AP_SERIALMANAGER_OPEN_BAUD, AP_SERIALMANAGER_LIMBACH_BUFSIZE_RX, AP_SERIALMANAGER_LIMBACH_BUFSIZE_TX);
    }
}

bool AP_Limbach_Query2::send_request()
{

    if (_port == NULL)
    {
        return false;
    }

    // gcs().send_text(MAV_SEVERITY_INFO, "AP_Limbach_Query2::send_request");

    int16_t numc = _port->available();

    if (numc <= 0)
    {
        return false;
    }

    uint8_t query[3] = {0X3, 0X0B, 0XF2};

    // 发送查询指令
    for (int i = 0; i < 3; ++i)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "send_request = %u", static_cast<int>(query[i]));
        _port->write(query[i]);
    }
    return true;
}

Query2Msg AP_Limbach_Query2::accepting_request()
{

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "AP_Limbach_Query2::accepting_request()");

    Query2Msg msg;

    int16_t numc = _port->available();

    if (numc == 101)
    {
        response_buffer.resize(numc);
        // 逐字节读取数据并存储到缓冲区
        for (int i = 0; i < numc; ++i)
        {
            response_buffer[i] = _port->read();
        }

        if (response_buffer[0] == 101)
        {

    // response_buffer = {
    //     0x65, 0x0B, 0x00, 0x00, 0x0A, 0x00, 0x14, 0x00, 0x1E, 0x00,
    //     0x28, 0x00, 0x32, 0x00, 0x3C, 0x00, 0x46, 0x00, 0x50, 0x00,
    //     0x5A, 0x00, 0x64, 0x00, 0x6E, 0x00, 0x78, 0x00, 0x82, 0x00,
    //     0x8C, 0x00, 0x96, 0x00, 0xA0, 0x00, 0xAA, 0x00, 0xB4, 0x00,
    //     0xBE, 0x00, 0xC8, 0x00, 0xD2, 0x00, 0xDC, 0x00, 0xE6, 0x00,
    //     0xF0, 0x00, 0xFA, 0x00, 0x04, 0x00, 0x0E, 0x00, 0x18, 0x00,
    //     0x22, 0x00, 0x2C, 0x00, 0x36, 0x00, 0x40, 0x00, 0x4A, 0x00,
    //     0x54, 0x00, 0x5E, 0x00, 0x68, 0x00, 0x72, 0x00, 0x7C, 0x00,
    //     0x86, 0x00, 0x90, 0x00, 0x9A, 0x00, 0xA4, 0x00, 0xAE, 0x00,
    //     0xB8, 0x00, 0xC2, 0x00, 0xCC, 0x00, 0xD6, 0x00, 0xE0, 0x00,
    //     0xEA, 0x00, 0xF4, 0x00, 0xFE, 0x00, 0x28, 0x00, 0x30, 0x00,
    //     0x46
    // };

            // 字节 0: 字符长度定义
            uint8_t char_length = response_buffer[0];
            // std::cout << "Character Length: " << static_cast<int>(char_length) << std::endl;

            // 字节 1: 识别码
            uint8_t id_code = response_buffer[1];
            // std::cout << "ID Code: " << static_cast<int>(id_code) << std::endl;

            // 字节 2-9: 预留位
            uint64_t reserved_2_9 = 0;
            for (int i = 0; i < 8; ++i)
            {
                reserved_2_9 |= (uint64_t)response_buffer[2 + i] << (8 * (7 - i));
            }
            // std::cout << "Reserved (2-9): " << reserved_2_9 << std::endl;

            // 字节 10-11: 进气量决定的喷射量
            uint16_t intake_injection = (response_buffer[11] << 8) | response_buffer[10];
            // std::cout << "Intake Injection Amount: " << intake_injection << std::endl;

            // 字节 12-13: 进气量修正曲线决定的喷射量
            uint16_t intake_correction_injection = (response_buffer[13] << 8) | response_buffer[12];
            // std::cout << "Intake Correction Injection: " << intake_correction_injection << std::endl;

            // 字节 14-15: 基本特性曲线决定的喷射量
            uint16_t base_curve_injection = (response_buffer[15] << 8) | response_buffer[14];
            // std::cout << "Base Curve Injection: " << base_curve_injection << std::endl;

            // 字节 16-17: 进气管负压曲线决定的喷射量
            uint16_t intake_pressure_injection = (response_buffer[17] << 8) | response_buffer[16];
            // std::cout << "Intake Pressure Injection: " << intake_pressure_injection << std::endl;

            // 字节 18-19: 喷射的基本量
            uint16_t base_injection = (response_buffer[19] << 8) | response_buffer[18];
            // std::cout << "Base Injection: " << base_injection << std::endl;

            // 字节 20-21: 进气修正后的喷射量
            uint16_t intake_correction_injection_2 = (response_buffer[21] << 8) | response_buffer[20];
            // std::cout << "Intake Correction Injection 2: " << intake_correction_injection_2 << std::endl;

            // 字节 22-23: 怠速位置传感器决定的喷射量
            uint16_t idle_position_injection = (response_buffer[23] << 8) | response_buffer[22];
            // std::cout << "Idle Position Injection: " << idle_position_injection << std::endl;

            // 字节 24-25: 暖机过程曲线决定的喷射量
            uint16_t warmup_curve_injection = (response_buffer[25] << 8) | response_buffer[24];
            // std::cout << "Warmup Curve Injection: " << warmup_curve_injection << std::endl;

            // 字节 26-27: 加速时决定的喷射量
            uint16_t acceleration_injection = (response_buffer[27] << 8) | response_buffer[26];
            // std::cout << "Acceleration Injection: " << acceleration_injection << std::endl;

            // 字节 28-29: 进气门的开合时间
            uint16_t intake_valve_time = (response_buffer[29] << 8) | response_buffer[28];
            // std::cout << "Intake Valve Time: " << intake_valve_time << std::endl;

            // 字节 30-31: RACE 模式决定的喷射量
            uint16_t race_mode_injection = (response_buffer[31] << 8) | response_buffer[30];
            // std::cout << "Race Mode Injection: " << race_mode_injection << std::endl;

            // 字节 32-33: 空燃比控制器决定的喷射量
            uint16_t afr_control_injection = (response_buffer[33] << 8) | response_buffer[32];
            // std::cout << "AFR Control Injection: " << afr_control_injection << std::endl;

            // 字节 34-35: 点火时间曲线决定的点火提前角
            uint16_t ignition_curve_advance = (response_buffer[35] << 8) | response_buffer[34];
            // std::cout << "Ignition Curve Advance: " << ignition_curve_advance << std::endl;

            // 字节 36-37: 进气温度曲线决定的点火提前角
            uint16_t intake_temp_advance = (response_buffer[37] << 8) | response_buffer[36];
            // std::cout << "Intake Temperature Advance: " << intake_temp_advance << std::endl;

            // 字节 38-39: 进气压力曲线决定的点火提前角
            uint16_t intake_pressure_advance = (response_buffer[39] << 8) | response_buffer[38];
            // std::cout << "Intake Pressure Advance: " << intake_pressure_advance << std::endl;

            // 字节 40-41: 发动机温度曲线决定的点火提前角
            uint16_t engine_temp_advance = (response_buffer[41] << 8) | response_buffer[40];
            // std::cout << "Engine Temperature Advance: " << engine_temp_advance << std::endl;

            // 字节 42-43: 加速时决定的点火提前角
            uint16_t acceleration_advance = (response_buffer[43] << 8) | response_buffer[42];
            // std::cout << "Acceleration Advance: " << acceleration_advance << std::endl;

            // 字节 44-45: RACE 模式决定的点火提前角
            uint16_t race_mode_advance = (response_buffer[45] << 8) | response_buffer[44];
            // std::cout << "Race Mode Advance: " << race_mode_advance << std::endl;

            // 字节 46-49: 26 次存储内总时间
            uint32_t total_storage_time = (response_buffer[49] << 24) | (response_buffer[48] << 16) |
                                          (response_buffer[47] << 8) | response_buffer[46];
            // std::cout << "Total Storage Time (26 entries): " << total_storage_time << std::endl;

            // 字节 50-53: 旋转总数
            uint32_t total_rotations = (response_buffer[53] << 24) | (response_buffer[52] << 16) |
                                       (response_buffer[51] << 8) | response_buffer[50];
            // std::cout << "Total Rotations: " << total_rotations << std::endl;

            // 字节 54-55: 燃油消耗（0.1l/h）
            uint16_t fuel_consumption = (response_buffer[55] << 8) | response_buffer[54];
            // std::cout << "Fuel Consumption (0.1l/h): " << fuel_consumption << std::endl;

            // 字节 56-57: 出错存储器中的错误数量
            uint16_t error_count = (response_buffer[57] << 8) | response_buffer[56];
            // std::cout << "Error Count in Memory: " << error_count << std::endl;

            // 字节 58-99: 预留位
            uint32_t reserved_58_99 = 0; // 这部分没有实际数据
            // std::cout << "Reserved (58-99): " << reserved_58_99 << std::endl;

            // 字节 100: 检验字节
            uint16_t checksum = (response_buffer[101] << 8) | response_buffer[100];
            // std::cout << "Checksum: " << checksum << std::endl;

            // Creating the msg object to store the parsed data

            msg.char_length = char_length;
            msg.id_code = id_code;
            msg.reserved_2_9 = reserved_2_9;
            msg.intake_injection = intake_injection;
            msg.intake_correction_injection = intake_correction_injection;
            msg.base_curve_injection = base_curve_injection;
            msg.intake_pressure_injection = intake_pressure_injection;
            msg.base_injection = base_injection;
            msg.intake_correction_injection_2 = intake_correction_injection_2;
            msg.idle_position_injection = idle_position_injection;
            msg.warmup_curve_injection = warmup_curve_injection;
            msg.acceleration_injection = acceleration_injection;
            msg.intake_valve_time = intake_valve_time;
            msg.race_mode_injection = race_mode_injection;
            msg.afr_control_injection = afr_control_injection;
            msg.ignition_curve_advance = ignition_curve_advance;
            msg.intake_temp_advance = intake_temp_advance;
            msg.intake_pressure_advance = intake_pressure_advance;
            msg.engine_temp_advance = engine_temp_advance;
            msg.acceleration_advance = acceleration_advance;
            msg.race_mode_advance = race_mode_advance;
            msg.total_storage_time = total_storage_time;
            msg.total_rotations = total_rotations;
            msg.fuel_consumption = fuel_consumption;
            msg.error_count = error_count;
            msg.reserved_58_99 = reserved_58_99;
            msg.checksum = checksum;

            // Sending information to GCS using send_text
            gcs().send_text(MAV_SEVERITY_INFO, "Character Length: %u", msg.char_length);
            gcs().send_text(MAV_SEVERITY_INFO, "ID Code: %u", msg.id_code);
            gcs().send_text(MAV_SEVERITY_INFO, "Reserved (2-9): %lu", msg.reserved_2_9);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Injection Amount: %u", msg.intake_injection);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Correction Injection: %u", msg.intake_correction_injection);
            gcs().send_text(MAV_SEVERITY_INFO, "Base Curve Injection: %u", msg.base_curve_injection);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Pressure Injection: %u", msg.intake_pressure_injection);
            gcs().send_text(MAV_SEVERITY_INFO, "Base Injection: %u", msg.base_injection);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Correction Injection 2: %u", msg.intake_correction_injection_2);
            gcs().send_text(MAV_SEVERITY_INFO, "Idle Position Injection: %u", msg.idle_position_injection);
            gcs().send_text(MAV_SEVERITY_INFO, "Warmup Curve Injection: %u", msg.warmup_curve_injection);
            gcs().send_text(MAV_SEVERITY_INFO, "Acceleration Injection: %u", msg.acceleration_injection);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Valve Time: %u", msg.intake_valve_time);
            gcs().send_text(MAV_SEVERITY_INFO, "Race Mode Injection: %u", msg.race_mode_injection);
            gcs().send_text(MAV_SEVERITY_INFO, "AFR Control Injection: %u", msg.afr_control_injection);
            gcs().send_text(MAV_SEVERITY_INFO, "Ignition Curve Advance: %u", msg.ignition_curve_advance);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Temperature Advance: %u", msg.intake_temp_advance);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Pressure Advance: %u", msg.intake_pressure_advance);
            gcs().send_text(MAV_SEVERITY_INFO, "Engine Temperature Advance: %u", msg.engine_temp_advance);
            gcs().send_text(MAV_SEVERITY_INFO, "Acceleration Advance: %u", msg.acceleration_advance);
            gcs().send_text(MAV_SEVERITY_INFO, "Race Mode Advance: %u", msg.race_mode_advance);
            gcs().send_text(MAV_SEVERITY_INFO, "Total Storage Time (26 entries): %u", msg.total_storage_time);
            gcs().send_text(MAV_SEVERITY_INFO, "Total Rotations: %u", msg.total_rotations);
            gcs().send_text(MAV_SEVERITY_INFO, "Fuel Consumption (0.1l/h): %u", msg.fuel_consumption);
            gcs().send_text(MAV_SEVERITY_INFO, "Error Count in Memory: %u", msg.error_count);
            gcs().send_text(MAV_SEVERITY_INFO, "Reserved (58-99): %u", msg.reserved_58_99);
            gcs().send_text(MAV_SEVERITY_INFO, "Checksum: %u", msg.checksum);
        }
    }
    return msg;
}
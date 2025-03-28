#define AP_SERIALMANAGER_OPEN_BAUD         115200
#define AP_SERIALMANAGER_LIMBACH_BUFSIZE_RX        64
#define AP_SERIALMANAGER_LIMBACH_BUFSIZE_TX        64

#include "AP_Limbach_Query1.h"
#include <../libraries/AP_HAL/utility/functor.h>
#include <../ArduCopter/GCS_Copter.h>

AP_Limbach_Query1::AP_Limbach_Query1(void)
{
    _port = NULL;
}

void AP_Limbach_Query1::init(const AP_SerialManager& serial_manager)
{
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_LIMBACH, 0))) {
        
        // gcs().send_text(MAV_SEVERITY_CRITICAL,"AP_Limbach_Query1::init");

        _port->begin(AP_SERIALMANAGER_OPEN_BAUD, AP_SERIALMANAGER_LIMBACH_BUFSIZE_RX, AP_SERIALMANAGER_LIMBACH_BUFSIZE_TX);
    }
}

bool AP_Limbach_Query1::send_request()
{

    if (_port == NULL){
        return false;
    }

    // gcs().send_text(MAV_SEVERITY_INFO, "AP_Limbach_Query1::send_request");

    int16_t numc = _port->available();

    if (numc <= 0) {
        return false; 
    }

    uint8_t query[3] ={0x3,0x4,0xF9};

    // 发送查询指令
    for (int i = 0; i < 3; ++i) {
        gcs().send_text(MAV_SEVERITY_INFO, "send_request = %u", static_cast<int>(query[i]));
        _port->write(query[i]);
    }
    return true;
    
}

Query1Msg AP_Limbach_Query1::accepting_request(){

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "AP_Limbach_Query1::accepting_request()");

    Query1Msg msg;
   
    int16_t numc = _port->available();
    
    if (numc == 87) {
        response_buffer.resize(numc);
        // 逐字节读取数据并存储到缓冲区
        for (int i = 0; i < numc; ++i) {
            response_buffer[i] = _port->read();
        }

        if(response_buffer[0]==87){


    // response_buffer = {
    //     0x57, 0x04, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    //     0x01, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
    //     0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00,
    //     0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
    //     0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00,
    //     0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
    //     0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00,
    //     0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
    //     0x81
    // };

            // 字节 0: 字符长度定义
            uint8_t char_length = response_buffer[0];
            // std::cout << "Character Length: " << static_cast<int>(char_length) << std::endl;
            
            // 字节 1: 识别码
            uint8_t id_code = response_buffer[1];
            // std::cout << "ID Code: " << static_cast<int>(id_code) << std::endl;

            // 字节 2-3: 预留位
            uint16_t reserved_2_3 = (response_buffer[3] << 8) | response_buffer[2];
            // std::cout << "Reserved (2-3): " << reserved_2_3 << std::endl;

            // 字节 4-5: 存储定义
            uint16_t store_def = (response_buffer[5] << 8) | response_buffer[4];
            // std::cout << "Storage Definition: " << store_def << std::endl;

            // 字节 6-9: 预留位
            uint32_t reserved_6_9 = (response_buffer[9] << 24) | (response_buffer[8] << 16) |
                                    (response_buffer[7] << 8) | response_buffer[6];
            // std::cout << "Reserved (6-9): " << reserved_6_9 << std::endl;

            // 字节 10-11: 发动机运行状态
            uint16_t engine_state = (response_buffer[11] << 8) | response_buffer[10];
            // std::cout << "Engine State: " << engine_state << std::endl;

            // 字节 12-13: 发动机转速
            uint16_t engine_rpm = (response_buffer[13] << 8) | response_buffer[12];
            // std::cout << "Engine RPM: " << engine_rpm << std::endl;

            // 字节 14-17: 预留位
            uint32_t reserved_14_17 = (response_buffer[17] << 24) | (response_buffer[16] << 16) |
                                    (response_buffer[15] << 8) | response_buffer[14];
            // std::cout << "Reserved (14-17): " << reserved_14_17 << std::endl;

            // 字节 18-19: 距暖车结束时间
            uint16_t warmup_time_remaining = (response_buffer[19] << 8) | response_buffer[18];
            // std::cout << "Warm-up Time Remaining: " << warmup_time_remaining << std::endl;

            // 字节 20-23: 预留位
            uint32_t reserved_20_23 = (response_buffer[23] << 24) | (response_buffer[22] << 16) |
                                    (response_buffer[21] << 8) | response_buffer[20];
            // std::cout << "Reserved (20-23): " << reserved_20_23 << std::endl;

            // 字节 24-25: 怠速位置设置
            uint16_t idle_position = (response_buffer[25] << 8) | response_buffer[24];
            // std::cout << "Idle Position: " << idle_position << std::endl;

            // 字节 26-27: 干扰脉冲计数
            uint16_t interference_count = (response_buffer[27] << 8) | response_buffer[26];
            // std::cout << "Interference Pulse Count: " << interference_count << std::endl;

            // 字节 28-29: 进气量
            uint16_t air_intake = (response_buffer[29] << 8) | response_buffer[28];
            // std::cout << "Air Intake: " << air_intake << std::endl;

            // 字节 30-31: 进气压力
            uint16_t air_pressure = (response_buffer[31] << 8) | response_buffer[30];
            // std::cout << "Air Pressure: " << air_pressure << std::endl;

            // 字节 32-33: 转速出错计数
            uint16_t rpm_error_count = (response_buffer[33] << 8) | response_buffer[32];
            // std::cout << "RPM Error Count: " << rpm_error_count << std::endl;

            // 字节 34-35: 喷射时间
            uint16_t injection_time = (response_buffer[35] << 8) | response_buffer[34];
            // std::cout << "Injection Time: " << injection_time << std::endl;

            // 字节 36-37: 点火提前角
            uint16_t ignition_angle = (response_buffer[37] << 8) | response_buffer[36];
            // std::cout << "Ignition Angle: " << ignition_angle << std::endl;

            // 字节 38-39: 预留位
            uint16_t reserved_38_39 = (response_buffer[39] << 8) | response_buffer[38];
            // std::cout << "Reserved (38-39): " << reserved_38_39 << std::endl;

            // 字节 40-41: 节气门位置传感器电压
            uint16_t throttle_position_voltage = (response_buffer[41] << 8) | response_buffer[40];
            // std::cout << "Throttle Position Voltage: " << throttle_position_voltage << std::endl;

            // 字节 42-43: 电池电量传感器电压
            uint16_t battery_voltage = (response_buffer[43] << 8) | response_buffer[42];
            // std::cout << "Battery Voltage: " << battery_voltage << std::endl;

            // 字节 44-45: 空燃比传感器电压
            uint16_t afr_sensor_voltage = (response_buffer[45] << 8) | response_buffer[44];
            // std::cout << "AFR Sensor Voltage: " << afr_sensor_voltage << std::endl;

            // 字节 46-47: 发动机温度传感器电压
            uint16_t engine_temp_voltage = (response_buffer[47] << 8) | response_buffer[46];
            // std::cout << "Engine Temperature Voltage: " << engine_temp_voltage << std::endl;

            // 字节 48-49: 进气温度传感器电压
            uint16_t intake_temp_voltage = (response_buffer[49] << 8) | response_buffer[48];
            // std::cout << "Intake Temperature Voltage: " << intake_temp_voltage << std::endl;

            // 字节 50-51: 外界气压传感器电压
            uint16_t external_pressure_voltage = (response_buffer[51] << 8) | response_buffer[50];
            // std::cout << "External Pressure Voltage: " << external_pressure_voltage << std::endl;

            // 字节 52-53: 进气压力传感器电压
            uint16_t intake_pressure_voltage = (response_buffer[53] << 8) | response_buffer[52];
            // std::cout << "Intake Pressure Voltage: " << intake_pressure_voltage << std::endl;

            // 字节 54-55: 进气量传感器电压
            uint16_t intake_quantity_voltage = (response_buffer[55] << 8) | response_buffer[54];
            // std::cout << "Intake Quantity Voltage: " << intake_quantity_voltage << std::endl;

            // 字节 56-57: 空燃比
            int16_t afr_value = (response_buffer[57] << 8) | response_buffer[56];
            // std::cout << "AFR Value: " << afr_value << std::endl;

            // 字节 58-73: 预留位
            uint16_t reserved_58_73 = 0;  // 这部分没有实际数据
            // std::cout << "Reserved (58-73): " << reserved_58_73 << std::endl;

            // 字节 74-75: 节气门位置
            uint16_t throttle_position = (response_buffer[75] << 8) | response_buffer[74];
            // std::cout << "Throttle Position: " << throttle_position << std::endl;

            // 字节 76-77: 发动机温度（冷却液温度 1）
            uint16_t engine_cooling_temp = (response_buffer[77] << 8) | response_buffer[76];
            // std::cout << "Engine Cooling Temperature (1): " << engine_cooling_temp << std::endl;

            // 字节 78-79: 蓄电池电压（ECU 工作电压）
            uint16_t ecu_battery_voltage = (response_buffer[79] << 8) | response_buffer[78];
            // std::cout << "ECU Battery Voltage: " << ecu_battery_voltage << std::endl;

            // 字节 80-81: 进气温度
            uint16_t intake_temp = (response_buffer[81] << 8) | response_buffer[80];
            // std::cout << "Intake Temperature: " << intake_temp << std::endl;

            // 字节 82-83: 进气压力
            uint16_t intake_pressure = (response_buffer[83] << 8) | response_buffer[82];
            // std::cout << "Intake Pressure: " << intake_pressure << std::endl;

            // 字节 84-85: 传感器状况
            uint16_t sensor_status = (response_buffer[85] << 8) | response_buffer[84];
            // std::cout << "Sensor Status: " << sensor_status << std::endl;

            // 字节 86: 检验字节
            uint8_t checksum = response_buffer[86];
            // std::cout << "Checksum: " << static_cast<int>(checksum) << std::endl;
        
            // 将 response_buffer 解析到结构体中
     
            msg.char_length = char_length;
            msg.id_code = id_code;
            msg.reserved_2_3 = reserved_2_3;
            msg.store_def = store_def;
            msg.reserved_6_9 = reserved_6_9;
            msg.engine_state = engine_state;
            msg.engine_rpm = engine_rpm;
            msg.reserved_14_17 = reserved_14_17;
            msg.warmup_time_remaining = warmup_time_remaining;
            msg.reserved_20_23 = reserved_20_23;
            msg.idle_position = idle_position;
            msg.interference_count = interference_count;
            msg.air_intake = air_intake;
            msg.air_pressure = air_pressure;
            msg.rpm_error_count = rpm_error_count;
            msg.injection_time = injection_time;
            msg.ignition_angle = ignition_angle;
            msg.reserved_38_39 = reserved_38_39;
            msg.throttle_position_voltage = throttle_position_voltage;
            msg.battery_voltage = battery_voltage;
            msg.afr_sensor_voltage = afr_sensor_voltage;
            msg.engine_temp_voltage = engine_temp_voltage;
            msg.intake_temp_voltage = intake_temp_voltage;
            msg.external_pressure_voltage = external_pressure_voltage;
            msg.intake_pressure_voltage = intake_pressure_voltage;
            msg.intake_quantity_voltage = intake_quantity_voltage;
            msg.afr_value = afr_value;
            msg.reserved_58_73 = reserved_58_73;
            msg.throttle_position = throttle_position;
            msg.engine_cooling_temp = engine_cooling_temp;
            msg.ecu_battery_voltage = ecu_battery_voltage;
            msg.intake_temp = intake_temp;
            msg.intake_pressure = intake_pressure;
            msg.sensor_status = sensor_status;
            msg.checksum = checksum;

            // Sending some information to GCS using send_text
            gcs().send_text(MAV_SEVERITY_INFO, "Character Length: %u", char_length);
            gcs().send_text(MAV_SEVERITY_INFO, "ID Code: %u", id_code);
            gcs().send_text(MAV_SEVERITY_INFO, "Engine State: %u", engine_state);
            gcs().send_text(MAV_SEVERITY_INFO, "Engine RPM: %u", engine_rpm);
            gcs().send_text(MAV_SEVERITY_INFO, "Warm-up Time Remaining: %u", warmup_time_remaining);
            gcs().send_text(MAV_SEVERITY_INFO, "Idle Position: %u", idle_position);
            gcs().send_text(MAV_SEVERITY_INFO, "Air Intake: %u", air_intake);
            gcs().send_text(MAV_SEVERITY_INFO, "Air Pressure: %u", air_pressure);
            gcs().send_text(MAV_SEVERITY_INFO, "Injection Time: %u", injection_time);
            gcs().send_text(MAV_SEVERITY_INFO, "Ignition Angle: %u", ignition_angle);
            gcs().send_text(MAV_SEVERITY_INFO, "Throttle Position Voltage: %u", throttle_position_voltage);
            gcs().send_text(MAV_SEVERITY_INFO, "Battery Voltage: %u", battery_voltage);
            gcs().send_text(MAV_SEVERITY_INFO, "AFR Sensor Voltage: %u", afr_sensor_voltage);
            gcs().send_text(MAV_SEVERITY_INFO, "Engine Temperature Voltage: %u", engine_temp_voltage);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Temperature Voltage: %u", intake_temp_voltage);
            gcs().send_text(MAV_SEVERITY_INFO, "External Pressure Voltage: %u", external_pressure_voltage);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Pressure Voltage: %u", intake_pressure_voltage);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Quantity Voltage: %u", intake_quantity_voltage);
            gcs().send_text(MAV_SEVERITY_INFO, "AFR Value: %d", afr_value);
            gcs().send_text(MAV_SEVERITY_INFO, "Throttle Position: %u", throttle_position);
            gcs().send_text(MAV_SEVERITY_INFO, "Engine Cooling Temperature: %u", engine_cooling_temp);
            gcs().send_text(MAV_SEVERITY_INFO, "ECU Battery Voltage: %u", ecu_battery_voltage);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Temperature: %u", intake_temp);
            gcs().send_text(MAV_SEVERITY_INFO, "Intake Pressure: %u", intake_pressure);
            gcs().send_text(MAV_SEVERITY_INFO, "Sensor Status: %u", sensor_status);
            gcs().send_text(MAV_SEVERITY_INFO, "Checksum: %u", checksum);
        }

    }
    return msg;
}
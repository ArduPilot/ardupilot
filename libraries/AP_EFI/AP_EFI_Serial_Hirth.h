/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "AP_EFI_config.h"

#if AP_EFI_SERIAL_HIRTH_ENABLED
#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

/*!
 * class definition for Hirth 4103 ECU
 */
class AP_EFI_Serial_Hirth: public AP_EFI_Backend {
public:
    AP_EFI_Serial_Hirth(AP_EFI &_frontend);

    void update() override;

    enum class Error_Excess_Temp_Bitfield : uint16_t {
        CHT_1_LOW = 1U<<0, // true if CHT1 is too low
        CHT_1_HIGH = 1U<<1, // true if CHT1 is too high
        CHT_1_ENRC_ACTIVE = 1U<<2, // true if CHT1 Enrichment is active
        CHT_2_LOW = 1U<<3, // true if CHT2 is too low
        CHT_2_HIGH = 1U<<4, // true if CHT2 is too high
        CHT_2_ENRC_ACTIVE = 1U<<5, // true if CHT2 Enrichment is active
        EGT_1_LOW = 1U<<6, // true if EGT1 is too low
        EGT_1_HIGH = 1U<<7, // true if EGT1 is too high
        EGT_1_ENRC_ACTIVE = 1U<<8, // true if EGT1 Enrichment is active
        EGT_2_LOW = 1U<<9, // true if EGT2 is too low
        EGT_2_HIGH = 1U<<10, // true if EGT2 is too high
        EGT_2_ENRC_ACTIVE = 1U<<11, // true if EGT2 Enrichment is active
    };

    enum class Sensor_Status_Bitfield : uint8_t {
        ENGINE_TEMP_SENSOR_OK = 1U<<1, // true if engine temperature sensor is OK
        AIR_TEMP_SENSOR_OK = 1U<<2, // true if air temperature sensor is OK
        AIR_PRESSURE_SENSOR_OK = 1U<<3, // true if intake air pressure sensor is OK
        THROTTLE_SENSOR_OK = 1U<<4, // true if throttle sensor is OK
    };

private:
    // serial port instance
    AP_HAL::UARTDriver *port;

    // periodic refresh 
    uint32_t last_request_ms;
    uint32_t last_packet_ms;
    uint32_t last_req_send_throttle_ms;

    // raw bytes - max size
    uint8_t raw_data[256];

    // request and response data
    uint8_t requested_code;

    // meta-data for a response
    struct {
        uint8_t quantity;
        uint8_t code;
        uint8_t checksum;
    } res_data;

    // TRUE - Request is sent; waiting for response
    // FALSE - Response is already received
    bool waiting_response;

    // Expected bytes from response
    uint8_t expected_bytes;

    // Throttle values
    uint16_t last_throttle;    
    uint16_t throttle_to_hirth;

    uint32_t last_fuel_integration_ms;

    // custom status for Hirth
    uint16_t sensor_status;

    uint16_t error_excess_temperature;
    uint32_t crc_fail_cnt;
    uint32_t uptime;
    uint32_t ack_fail_cnt;

    struct PACKED Record1 {
        uint8_t reserved1[2];
        uint16_t save_in_flash;  // "1 = data are saved in flash automatically"
        uint8_t reserved2[4];
        uint16_t engine_status;
        uint16_t rpm;
        uint8_t reserved3[12];
        uint16_t number_of_interfering_pulses;
        uint16_t reserved4[2];
        uint16_t number_of_speed_errors;
        uint16_t injection_time;
        uint16_t ignition_angle;
        uint16_t reserved5;
        uint16_t voltage_throttle;
        uint16_t reserved6;
        uint8_t reserved7[2];
        uint16_t voltage_engine_temperature;
        uint16_t voltage_air_temperature;
        uint8_t reserved8[2];
        uint16_t voltage_int_air_pressure;
        uint8_t reserved9[20];
        int16_t throttle;
        int16_t engine_temperature;
        int16_t battery_voltage;
        int16_t air_temperature;
        int16_t reserved10;
        uint16_t sensor_ok;
    };
    static_assert(sizeof(Record1) == 84, "incorrect Record1 length");

    struct PACKED Record2 {
        uint8_t reserved1[12];
        int16_t injection_rate_from_basic_graphic_map;
        int16_t reserved2;
        int16_t basic_injection_rate;
        int16_t injection_rate_from_air_correction;
        int16_t reserved3;
        int16_t injection_rate_from_warming_up_characteristic_curve;
        int16_t injection_rate_from_acceleration_enrichment;
        int16_t turn_on_time_of_intake_valves;
        int16_t injection_rate_from_race_switch;
        int16_t reserved4;
        int16_t injection_angle_from_ignition_graphic_map;
        int16_t injection_angle_from_air_temperature_characteristic_curve;
        int16_t injection_angle_from_air_pressure_characteristic_curve;
        int16_t ignition_angle_from_engine_temperature_characteristic_curve;
        int16_t ignition_angle_from_acceleration;
        int16_t ignition_angle_from_race_switch;
        uint32_t total_time_in_26ms;
        uint32_t total_number_of_rotations;
        uint16_t fuel_consumption;
        uint16_t number_of_errors_in_error_memory;
        int16_t voltage_input1_throttle_target;
        int16_t reserved5;
        int16_t position_throttle_target;
        int16_t throttle_percent_times_10;  // percent * 0.1
        int16_t reserved6[3];
        uint16_t time_of_injector_opening_percent_times_10;
        uint8_t reserved7[10];
        uint32_t no_of_logged_data;
        uint8_t reserved8[12];
    };
    static_assert(sizeof(Record2) == 98, "incorrect Record2 length");

    struct PACKED Record3 {
        int16_t voltage_excess_temperature_1;
        int16_t voltage_excess_temperature_2;
        int16_t voltage_excess_temperature_3;
        int16_t voltage_excess_temperature_4;
        int16_t voltage_excess_temperature_5;
        uint8_t reserved1[6];
        int16_t excess_temperature_1;  // cht1
        int16_t excess_temperature_2;  // cht2
        int16_t excess_temperature_3;  // egt1
        int16_t excess_temperature_4;  // egt2
        int16_t excess_temperature_5;
        uint8_t reserved2[6];
        int16_t enrichment_excess_temperature_cylinder_1;
        int16_t enrichment_excess_temperature_cylinder_2;
        int16_t enrichment_excess_temperature_cylinder_3;
        int16_t enrichment_excess_temperature_cylinder_4;
        uint8_t reserved3[6];
        uint16_t error_excess_temperature_bitfield;
        uint16_t mixing_ratio_oil_pump1;
        uint16_t mixing_ratio_oil_pump2;
        uint16_t ouput_value_water_pump;
        uint16_t ouput_value_fuel_pump;
        uint16_t ouput_value_exhaust_valve;
        uint16_t ouput_value_air_vane;
        uint16_t ouput_value_e_throttle;
        uint16_t number_of_injections_oil_pump_1;
        uint32_t system_time_in_ms;
        int16_t number_of_injections_oil_pump_2;
        uint16_t target_rpm;
        uint16_t FPC;
        uint16_t xenrichment_excess_temperature_cylinder_1;
        uint16_t xenrichment_excess_temperature_cylinder_2;
        uint16_t xenrichment_excess_temperature_cylinder_3;
        uint16_t xenrichment_excess_temperature_cylinder_4;
        uint16_t voltage_input_temperature_crankshaft_housing;
        int16_t temperature_crankshaft_housing;
        uint8_t reserved4[14];
    };
    static_assert(sizeof(Record3) == 100, "incorrect Record3 length");

    void check_response();
    void send_request();
    void decode_data();
    bool send_request_status();
    bool send_target_values(uint16_t);
    void log_status();
};

#endif // AP_EFI_SERIAL_HIRTH_ENABLED

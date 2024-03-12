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
/*
  simulate Hirth EFI system

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduPlane -A --serial5=sim:hirth --speedup=1
param set SERIAL5_PROTOCOL 24
param set SIM_EFI_TYPE 6
param set EFI_TYPE 6
reboot
status EFI_STATUS

./Tools/autotest/autotest.py --gdb --debug build.Plane test.Plane.Hirth

*/

#pragma once

#include <SITL/SITL.h>
#include <AP_HAL/utility/Socket_native.h>
#include "SIM_SerialDevice.h"

namespace SITL {

class EFI_Hirth : public SerialDevice {
public:

    using SerialDevice::SerialDevice;

    void update();

private:

    void update_receive();
    void update_send();

    void assert_receive_size(uint8_t receive_size);

    void handle_set_values();

    // maps from an on-wire number to a record number:
    enum class PacketCode : uint8_t {
        DataRecord1 = 4,
        DataRecord2 = 11,
        DataRecord3 = 13,
        SetValues = 201,
    };

    template <typename T>
    class PACKED PackedRecord {
    public:
        PackedRecord(PacketCode _code, T _record) :
            code(uint8_t(_code)),
            record(_record)
        { }
            const uint8_t length { sizeof(T) + 3 };  // 1 each of length, code and checksum
        const uint8_t code;
        T record;
        uint8_t checksum;

        void update() {
            record.update();
            update_checksum();
        }

        void update_checksum() {
            checksum = 256U - crc_sum_of_bytes((uint8_t*)this, length-1);
        }
    };

    void send_record1();
    void send_record2();
    void send_record3();

    class PACKED Record1 {
    public:
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

        void update();
    };

    class PACKED Record2 {
    public:
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
        int16_t injection_angle_from_ignition_angle_map;
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
        uint16_t throttle_percent_times_10;  // percent * 10
        int16_t reserved6[3];
        uint16_t time_of_injector_opening_percent_times_10;
        uint8_t reserved7[10];
        uint32_t no_of_logged_data;
        uint8_t reserved8[12];
    };

    class PACKED Record3 {
    public:
        uint16_t voltage_excess_temperature_1;
        uint16_t voltage_excess_temperature_2;
        uint16_t voltage_excess_temperature_3;
        uint16_t voltage_excess_temperature_4;
        uint16_t voltage_excess_temperature_5;
        uint8_t reserved1[6];
        uint16_t excess_temperature_1;  // cht1
        uint16_t excess_temperature_2;  // cht2
        uint16_t excess_temperature_3;  // egt1
        uint16_t excess_temperature_4;  // egt2
        uint16_t excess_temperature_5;
        uint8_t reserved2[6];
        uint16_t enrichment_excess_temperature_cylinder_1;
        uint16_t enrichment_excess_temperature_cylinder_2;
        uint16_t enrichment_excess_temperature_cylinder_3;
        uint16_t enrichment_excess_temperature_cylinder_4;
        uint8_t reserved3[6];
        uint16_t enrichment_excess_temperature_bitfield;
        uint16_t mixing_ratio_oil_pump1;
        uint16_t mixing_ratio_oil_pump2;
        uint16_t ouput_value_water_pump;
        uint16_t ouput_value_fuel_pump;
        uint16_t ouput_value_exhaust_valve;
        uint16_t ouput_value_air_vane;
        uint16_t ouput_value_e_throttle;
        uint16_t number_of_injections_oil_pump_1;
        uint32_t system_time_in_ms;
        uint16_t number_of_injections_oil_pump_2;
        uint16_t target_rpm;
        uint16_t FPC;
        // these appear to be duplicates of the above; one is probably
        // voltage?
        uint16_t xenrichment_excess_temperature_cylinder_1;
        uint16_t xenrichment_excess_temperature_cylinder_2;
        uint16_t xenrichment_excess_temperature_cylinder_3;
        uint16_t xenrichment_excess_temperature_cylinder_4;
        uint16_t voltage_input_temperature_crankshaft_housing;
        uint16_t temperature_crankshaft_housing;
        uint8_t reserved4[14];
    };

    class PACKED SetValues {
    public:
        int16_t throttle;  // percent * 10
        int16_t rpm;;
        int8_t reserved1[16];
    };

    // these records are just used for initial values of the fields;
    // they aren't used past that.
    Record1 record1;
    Record2 record2;
    Record3 record3;


    SetValues settings;

    PackedRecord<Record1> packed_record1{PacketCode::DataRecord1, record1};
    PackedRecord<Record2> packed_record2{PacketCode::DataRecord2, record2};
    PackedRecord<Record3> packed_record3{PacketCode::DataRecord3, record3};

    struct {
        PacketCode code;  // code which was requested by driver
        uint32_t time_ms;  // time that code was requested by driver
    } requested_data_record;

    uint8_t receive_buf[32];
    uint8_t receive_buf_ofs;

    float throttle;

    uint16_t engine_status_field_value() const;

    void init();
    bool init_done = false;

    // engine model:
    void update_engine_model();
    struct {
        float cht1_temperature;  // engine reports in deg-C
        float cht2_temperature;
        uint32_t last_update_ms;
    } engine;
};

}

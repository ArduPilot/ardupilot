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
 * AP_EFI_NWPMU.h
 *
 *      Author: Michael Du Breuil
 */
 
#pragma once

#include "AP_EFI_config.h"

#if AP_EFI_NWPWU_ENABLED

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

class AP_EFI_NWPMU : public CANSensor, public AP_EFI_Backend {
public:
    AP_EFI_NWPMU(AP_EFI &_frontend);
    
    void update() override;

private:
    void handle_frame(AP_HAL::CANFrame &frame) override;

    bool _emitted_version;

    enum class NWPMU_ID {
        GCU    = 0x0006C000, // output voltage and current consumption
        ECU_1  = 0x0CFFF048, // rpm, tps, fuel open, ignition angle
        ECU_2  = 0x0CFFF148, // baro, MAP, lambda, pressure type
        ECU_3  = 0x0CFFF248, // analog input 1-4
        ECU_4  = 0x0CFFF348, // analog input 5-8
        ECU_5  = 0x0CFFF548, // battery voltage, air temp, coolant temp, temp type
        ECU_6  = 0x0CFFF648, // analog input 5, 7, version
        ECU_7  = 0x0CFFF848, // measured lambda 1-2, target lambda
        ECU_8  = 0x0CFFF948, // PWM ACHT, Blank, Fuel
        ECU_9  = 0x0CFFFB48, // ignition compensation, cut precent
        ECU_10 = 0x0CFFFD48, // Fuel comp (accel, starting, air temp, coolant temp)
        ECU_11 = 0x0CFFFE48, // Fuel comp (baro, MAP)
        ECU_12 = 0x0CFFD048, // Ignition comp (air temp, cooland temp, MAP)
    };

    enum class NWPMU_PRESSURE_TYPE {
        psi = 0,
        kPa = 1,
    };

    enum class NWPMU_TEMPERATURE_TYPE {
        F = 0,
        C = 1,
    };

    struct ecu_1 {
        uint16_t rpm;
        uint16_t tps;
        uint16_t fuel_open_time; // 0.01 per bit (ms)
        uint16_t ignition_angle; // 0.01 per bit (deg)
    };

    struct ecu_2 {
        uint16_t baro;   // 0.01 per bit
        uint16_t map;    // 0.01 per bit
        uint16_t lambda; // 0.01 per bit
        uint8_t pressure_type; // bit 1 is set if kPa, otherwise is PSI
    };

    struct ecu_3 {
        uint16_t analog_1; // 0.001 per bit (v)
        uint16_t analog_2; // 0.001 per bit (v)
        uint16_t analog_afr; // 0.001 per bit (v)
        uint16_t analog_blank; // 0.001 per bit (v)
    };

    struct ecu_4 {
        uint16_t analog_gen_temp;   // 0.001 per bit
        uint16_t analog_fuel_pres;  // 0.001 per bit
        uint16_t analog_fuel_level; // 0.001 per bit
        uint16_t analog_baro_pres;  // 0.001 per bit
    };

    struct ecu_5 {
        uint16_t batt_volt; // 0.01 per bit (v)
        uint16_t air_temp;  // 0.1 per bit
        uint16_t coolant_temp; // 0.1 per bit
        uint8_t  temp_type; // 0 F, 1 C
    };

    struct ecu_6 {
        uint16_t analog_5; // 0.1 per bit
        uint16_t analog_7; // 0.1 per bit
        uint8_t firmware_major;
        uint8_t firmware_minor;
        uint8_t firmware_build;
    };
};

#endif // AP_EFI_NWPWU_ENABLED


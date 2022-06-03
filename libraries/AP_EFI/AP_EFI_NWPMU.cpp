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

#include <AP_HAL/AP_HAL.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#include "AP_EFI_NWPMU.h"

#if HAL_EFI_NWPWU_ENABLED

extern const AP_HAL::HAL& hal;

AP_EFI_NWPMU::AP_EFI_NWPMU(AP_EFI &_frontend) :
    CANSensor("NWPMU"),
    AP_EFI_Backend(_frontend)
{
    register_driver(AP_CANManager::Driver_Type_EFI_NWPMU);
}

void AP_EFI_NWPMU::handle_frame(AP_HAL::CANFrame &frame)
{
    const uint32_t id =  frame.id & AP_HAL::CANFrame::MaskExtID;

    WITH_SEMAPHORE(frontend.sem);

    switch ((NWPMU_ID)id) {
    case NWPMU_ID::ECU_1: {
        internal_state.last_updated_ms = AP_HAL::millis();
        struct ecu_1 data;
        memcpy(&data, frame.data, sizeof(data));
        internal_state.engine_speed_rpm = data.rpm;
        internal_state.throttle_position_percent = data.tps * 0.1f;
        internal_state.cylinder_status[0].ignition_timing_deg = data.ignition_angle * 0.1f;
        break;
    }

    case NWPMU_ID::ECU_2: {
        internal_state.last_updated_ms = AP_HAL::millis();
        struct ecu_2 data;
        memcpy(&data, frame.data, sizeof(data));
        switch ((NWPMU_PRESSURE_TYPE)data.pressure_type) {
        case NWPMU_PRESSURE_TYPE::kPa:
            internal_state.atmospheric_pressure_kpa = data.baro * 0.01f;
            internal_state.intake_manifold_pressure_kpa = data.baro * 0.01f;
            break;
        case NWPMU_PRESSURE_TYPE::psi:
            internal_state.atmospheric_pressure_kpa = data.baro * 0.0689476f;
            internal_state.intake_manifold_pressure_kpa = data.baro * 0.0689476f;
            break;
        default:
            break;
        }
        internal_state.cylinder_status[0].lambda_coefficient = data.lambda * 0.01f;
        break;
    }

    case NWPMU_ID::ECU_4: {
        internal_state.last_updated_ms = AP_HAL::millis();
        struct ecu_4 data;
        memcpy(&data, frame.data, sizeof(data));
        // remap the analog input for fuel pressure, 0.5 V == 0 PSI, 4.5V == 100 PSI
        internal_state.fuel_pressure = linear_interpolate(0, 689.476,
                                                          data.analog_fuel_pres * 0.001,
                                                          0.5f,4.5f);
        break;
    }

    case NWPMU_ID::ECU_5: {
        internal_state.last_updated_ms = AP_HAL::millis();
        struct ecu_5 data;
        memcpy(&data, frame.data, sizeof(data));
        switch((NWPMU_TEMPERATURE_TYPE)data.temp_type) {
        case NWPMU_TEMPERATURE_TYPE::C:
            internal_state.coolant_temperature = C_TO_KELVIN(data.coolant_temp * 0.1f);
            internal_state.cylinder_status[0].cylinder_head_temperature = C_TO_KELVIN(data.coolant_temp * 0.1f);
            break;
        case NWPMU_TEMPERATURE_TYPE::F:
            internal_state.coolant_temperature = F_TO_KELVIN(data.coolant_temp * 0.1f);
            internal_state.cylinder_status[0].cylinder_head_temperature = F_TO_KELVIN(data.coolant_temp * 0.1f);
            break;
        default:
            break;
        }
        break;
    }

    case NWPMU_ID::ECU_6: {
        internal_state.last_updated_ms = AP_HAL::millis();
        struct ecu_6 data;
        memcpy(&data, frame.data, sizeof(data));
        if (!_emitted_version && (AP_HAL::millis() > 10000)) { // don't emit a version early in the boot process
            gcs().send_text(MAV_SEVERITY_INFO, "NWPMU Version: %d.%d.%d",
                            data.firmware_major,
                            data.firmware_minor,
                            data.firmware_build);
            _emitted_version = true;
        }
        break;
    }
    case NWPMU_ID::GCU:
    case NWPMU_ID::ECU_3:
    case NWPMU_ID::ECU_7:
    case NWPMU_ID::ECU_8:
    case NWPMU_ID::ECU_9:
    case NWPMU_ID::ECU_10:
    case NWPMU_ID::ECU_11:
    case NWPMU_ID::ECU_12:
        break;
    }
}

void AP_EFI_NWPMU::update()
{
    // copy the data to the front end
    copy_to_frontend();
}

#endif // HAL_EFI_NWPWU_ENABLED

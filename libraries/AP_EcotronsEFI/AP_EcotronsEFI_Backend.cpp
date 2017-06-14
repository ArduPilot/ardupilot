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

#include "AP_EcotronsEFI_Backend.h"

extern const AP_HAL::HAL &hal;

AP_EcotronsEFI_Backend::AP_EcotronsEFI_Backend(EFI_State& _efi_state) : 
    efi_state(_efi_state) 
{
    _sem = hal.util->new_semaphore();
}

void AP_EcotronsEFI_Backend::copy_to_frontend() {
    copy_state(_internal_state, efi_state);
}

void AP_EcotronsEFI_Backend::copy_state(const EFI_State& src, EFI_State& dst) 
{
    // Copy POD vars
    dst.rpm = src.rpm;  
    dst.fuel_level_percent = src.fuel_level_percent;  
    dst.fuel_flow_rate = src.fuel_flow_rate;  
    dst.engine_load_percent = src.engine_load_percent;  
    dst.throttle_position_percent = src.throttle_position_percent;  
    dst.end_of_start = src.end_of_start;  
    dst.crank_sensor_error = src.crank_sensor_error;  
    dst.spark_dwell_time_ms = src.spark_dwell_time_ms;  
    dst.barometric_pressure = src.barometric_pressure;  
    dst.intake_manifold_pressure = src.intake_manifold_pressure;
    dst.intake_manifold_temperature = src.intake_manifold_temperature;
    dst.coolant_temperature = src.coolant_temperature;
    dst.battery_voltage = src.battery_voltage;

    // Copy arrays
    for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
        dst.ignition_timing_crank_angle[i] = src.ignition_timing_crank_angle[i]; 
    }

    for (int i = 0; i < ENGINE_MAX_INJECTORS; i++) {
        dst.injection_time_ms[i] = src.injection_time_ms[i];
    }
} 
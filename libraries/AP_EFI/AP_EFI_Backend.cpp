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

#include "AP_EFI_Backend.h"

extern const AP_HAL::HAL &hal;

AP_EFI_Backend::AP_EFI_Backend(EFI_State& _efi_state) : 
    efi_state(_efi_state) 
{
    _sem = hal.util->new_semaphore();
}

void AP_EFI_Backend::copy_to_frontend() {
    copy_state(_internal_state, efi_state);
}

void AP_EFI_Backend::copy_state(const EFI_State& src, EFI_State& dst) 
{

    dst.last_updated_ms = src.last_updated_ms;
    dst.engine_state = src.engine_state;
    dst.general_error = src.general_error;
    dst.crankshaft_sensor_status = src.crankshaft_sensor_status;
    dst.temperature_status = src.temperature_status;
    dst.fuel_pressure_status = src.fuel_pressure_status;
    dst.oil_pressure_status = src.oil_pressure_status;
    dst.detonation_status = src.detonation_status;
    dst.misfire_status = src.misfire_status;
    dst.debris_status  = src.debris_status;
    dst.engine_load_percent = src.engine_load_percent;
    dst.engine_speed_rpm = src.engine_speed_rpm;
    dst.spark_dwell_time_ms = src.spark_dwell_time_ms;
    dst.atmospheric_pressure_kpa = src.atmospheric_pressure_kpa;
    dst.intake_manifold_pressure_kpa = src.intake_manifold_pressure_kpa;
    dst.intake_manifold_temperature = src.intake_manifold_temperature;
    dst.coolant_temperature = src.coolant_temperature;
    dst.oil_pressure = src.oil_pressure;
    dst.oil_temperature = src.oil_temperature;
    dst.fuel_pressure = src.fuel_pressure;
    dst.fuel_consumption_rate_cm3pm = src.fuel_consumption_rate_cm3pm;
    dst.estimated_consumed_fuel_volume_cm3 = src.estimated_consumed_fuel_volume_cm3;
    dst.throttle_position_percent = src.throttle_position_percent;
    dst.ecu_index = src.ecu_index;
    dst.spark_plug_usage = src.spark_plug_usage;

    for (int i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
        dst.cylinder_status[i] = src.cylinder_status[i];
    }

} 
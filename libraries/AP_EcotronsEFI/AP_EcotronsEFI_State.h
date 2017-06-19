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

#define EFI_MAX_INSTANCES 2
#define EFI_MAX_BACKENDS 2

// Careful when changing these. Ecotrons sends back 4 
// cylinders and injectors worth of data, and the 
// mavlink message also expects 4 cylinders.
#define ENGINE_MAX_CYLINDERS 4
#define ENGINE_MAX_INJECTORS 4

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

// Time in milliseconds before we declare the EFI to be "unhealthy"
#define HEALTHY_LAST_RECEIVED_MS 3000

// This file was created to solve circular dependancy problems when using EFI_State

// Stores the current state read by the EFI system
// This mimics the Status message broadcast by the EFI through UAVCAN, and
// is very similar to the message format from RS232
struct EFI_State {
    // ECU Index provided by ecotrons
    uint8_t ecu_index;

    // Declaration of health of sensor by backend
    uint32_t last_updated_ms;

    // Revolutions per minute of the engine
    float rpm;

    // Total amount of fuel left (0 - 100%)
    float fuel_level_percent;

    // Fuel consumption rate in g/min
    float fuel_flow_rate;

    // Current load on the engine (0 - 100%)
    float engine_load_percent;

    // Throttle position from the throttle position sensor (TPS) (0 - 100%)
    float throttle_position_percent;

    // Has the engine exited the "starting" state?
    bool end_of_start;

    // Is there an error with the crank sensor?
    bool crank_sensor_error;

    // Time taken to charge the spark-plug coil (ms)
    float spark_dwell_time_ms;

    // External pressure around the engine (kPa)
    float barometric_pressure;

    // Pressure at the engine's intake manifold (kPa)
    float intake_manifold_pressure;

    // Temperature at the engine's intake manifold (degC)
    float intake_manifold_temperature;

    // Engine coolant temperature, approximately engine temp (degC)
    float coolant_temperature;

    // ECU Battery voltage (V)
    float battery_voltage;

    // Ignition timing for each cylinder (crank angle, degrees)
    float ignition_timing_crank_angle[ENGINE_MAX_CYLINDERS];

    // Injection time for each injector (ms)
    float injection_time_ms[ENGINE_MAX_INJECTORS];
};
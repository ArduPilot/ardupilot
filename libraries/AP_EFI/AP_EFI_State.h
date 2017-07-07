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
#define ENGINE_MAX_CYLINDERS 16

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

// Time in milliseconds before we declare the EFI to be "unhealthy"
#define HEALTHY_LAST_RECEIVED_MS 3000

/***************
 *
 * Status enums
 *
 ***************/

enum Engine_State {
    ENGINE_STATE_STOPPED  = 0,
    ENGINE_STATE_STARTING = 1,
    ENGINE_STATE_RUNNING  = 2,
    ENGINE_STATE_FAULT    = 3
};

enum Crankshaft_Sensor_Status {
    CRANKSHAFT_SENSOR_STATUS_NOT_SUPPORTED = 0,
    CRANKSHAFT_SENSOR_OK                   = 1,
    CRANKSHAFT_SENSOR_ERROR                = 2
};

enum Temperature_Status {
    TEMPERATURE_STATUS_NOT_SUPPORTED       = 0,
    TEMPERATURE_OK                         = 1,
    TEMPERATURE_BELOW_NOMINAL              = 2,
    TEMPERATURE_ABOVE_NOMINAL              = 3,
    TEMPERATURE_OVERHEATING                = 4,
    TEMPERATURE_EGT_ABOVE_NOMINAL          = 5
};

enum Fuel_Pressure_Status {
    FUEL_PRESSURE_STATUS_NOT_SUPPORTED = 0,
    FUEL_PRESSURE_OK                   = 1,
    FUEL_PRESSURE_BELOW_NOMINAL        = 2,
    FUEL_PRESSURE_ABOVE_NOMINAL        = 3
};

enum Oil_Pressure_Status {
    OIL_PRESSURE_STATUS_NOT_SUPPORTED = 0,
    OIL_PRESSURE_OK                   = 1,
    OIL_PRESSURE_BELOW_NOMINAL        = 2,
    OIL_PRESSURE_ABOVE_NOMINAL        = 3
};

enum Detonation_Status {
    DETONATION_STATUS_NOT_SUPPORTED = 0,
    DETONATION_NOT_OBSERVED         = 1,
    DETONATION_OBSERVED             = 2
};

enum Misfire_Status {
    MISFIRE_STATUS_NOT_SUPPORTED = 0,
    MISFIRE_NOT_OBSERVED         = 1,
    MISFIRE_OBSERVED             = 2
};

enum Debris_Status {
    DEBRIS_STATUS_NOT_SUPPORTED = 0,
    DEBRIS_NOT_DETECTED         = 1,
    DEBRIS_DETECTED             = 2
};

enum Spark_Plug_Usage {
    SPARK_PLUG_SINGLE        = 0,
    SPARK_PLUG_FIRST_ACTIVE  = 1,
    SPARK_PLUG_SECOND_ACTIVE = 2,
    SPARK_PLUG_BOTH_ACTIVE   = 3
};


/***************
 * Status structs.
 * EFIs may not provide all data in the message, therefore, the following guidelines should be followed.
 * All integer fields are required unless stated otherwise.
 * All floating point fields are optional unless stated otherwise; unknown/unapplicable fields will be NaN.
 ***************/

// TODO: not sure what to do with this yet - maybe different library?
struct Fuel_Tank_Status {
    
    // The estimated/measured amount of fuel, all fields are required.
    uint8_t available_fuel_volume_percent: 7; // 0% to 100%
    float   available_fuel_volume_cm3;        // centimeter^3

    // Estimate of the current fuel consumption rate ((centimeter^3)/minute)
    // The flow can be negative if the fuel is being transferred between the tanks or during refueling.
    // This field is required.
    float fuel_consumption_rate_cm3pm;

    // Fuel temperature (kelvin), optional field, to be set to NaN if not provided.
    float fuel_temperature;

    // ID of the current fuel tank.
    uint8_t fuel_tank_id;
};

// Per-cylinder status struct
struct Cylinder_Status {
    // Cylinder ignition timing (angular degrees of the crankshaft)
    float ignition_timing_deg;

    // Fuel injection time (millisecond)
    float injection_time_ms;

    // Cylinder head temperature (CHT) (kelvin)
    float cylinder_head_temperature;

    // Exhaust gas temperature (EGT) (kelvin)
    // If this cylinder is not equipped with an EGT sensor - will be NaN
    // If there is a single shared EGT sensor, will be the same value for all cylinders
    float exhaust_gas_temperature;

    // Estimated lambda coefficient (dimensionless ratio)
    // Useful for monitoring and tuning purposes.
    float lambda_coefficient;
};

// Stores the current state read by the EFI system
// All backends are required to fill in this state structure
struct EFI_State {
    // When this structure was last updated (milliseconds)
    uint32_t last_updated_ms;

    // Current overall engine state
    Engine_State engine_state                         : 2;
 
    // If there is an error that does not fit other error types
    bool general_error                                : 1;

    // Error/status fields 
    Crankshaft_Sensor_Status crankshaft_sensor_status : 2;
    Temperature_Status temperature_status             : 3;
    Fuel_Pressure_Status fuel_pressure_status         : 2;
    Oil_Pressure_Status oil_pressure_status           : 2;
    Detonation_Status detonation_status               : 2;
    Misfire_Status misfire_status                     : 2;
    Debris_Status debris_status                       : 2;

    // Engine load (percent)
    uint8_t engine_load_percent                       : 7;
    
    // Engine speed (revolutions per minute)
    uint32_t engine_speed_rpm                         : 17;

    // Spark dwell time (milliseconds)
    float spark_dwell_time_ms;

    // Atmospheric (barometric) pressure (kilopascal)
    float atmospheric_pressure_kpa;

    // Engine intake manifold pressure (kilopascal)
    float intake_manifold_pressure_kpa;

    // Engine intake manifold temperature (kelvin)
    float intake_manifold_temperature;

    // Engine coolant temperature (kelvin)
    float coolant_temperature;
    
    // Oil pressure (kilopascal)
    float oil_pressure;

    // Oil temperature (kelvin)
    float oil_temperature;

    // Fuel pressure (kilopascal)
    float fuel_pressure;

    // Instant fuel consumption estimate, which 
    // should be low-pass filtered in order to prevent aliasing effects.
    // (centimeter^3)/minute.
    float fuel_consumption_rate_cm3pm;

    // Estimate of the consumed fuel since the start of the engine (centimeter^3)
    // This variable is reset when the engine is stopped.
    float estimated_consumed_fuel_volume_cm3;

    // Throttle position (percent)
    uint8_t throttle_position_percent                 : 7;

    // The index of the publishing ECU.
    uint8_t ecu_index                                 : 6;

    // Spark plug activity report.
    // Can be used during pre-flight tests of the spark subsystem.
    // Use case is that usually on double spark plug engines, the 
    // engine switch has the positions OFF-LEFT-RIGHT-BOTH-START.
    // Gives pilots the possibility to test both spark plugs on 
    // ground before takeoff.
    Spark_Plug_Usage spark_plug_usage;

    // Status for each cylinder in the engine
    Cylinder_Status cylinder_status[ENGINE_MAX_CYLINDERS];

};

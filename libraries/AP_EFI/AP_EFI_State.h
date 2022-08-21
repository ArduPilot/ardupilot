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
#define ENGINE_MAX_CYLINDERS 1

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

// Time in milliseconds before we declare the EFI to be "unhealthy"
#define HEALTHY_LAST_RECEIVED_MS 3000

/***************
 *
 * Status enums
 *
 ***************/

enum class Engine_State : uint8_t {
    STOPPED  = 0,
    STARTING = 1,
    RUNNING  = 2,
    FAULT    = 3
};

enum class Crankshaft_Sensor_Status : uint8_t {
    NOT_SUPPORTED = 0,
    OK            = 1,
    ERROR         = 2
};

enum class Temperature_Status : uint8_t {
    NOT_SUPPORTED       = 0,
    OK                  = 1,
    BELOW_NOMINAL       = 2,
    ABOVE_NOMINAL       = 3,
    OVERHEATING         = 4,
    EGT_ABOVE_NOMINAL   = 5
};

enum class Fuel_Pressure_Status : uint8_t {
    NOT_SUPPORTED        = 0,
    OK                   = 1,
    BELOW_NOMINAL        = 2,
    ABOVE_NOMINAL        = 3
};

enum class Oil_Pressure_Status : uint8_t {
    NOT_SUPPORTED        = 0,
    OK                   = 1,
    BELOW_NOMINAL        = 2,
    ABOVE_NOMINAL        = 3
};

enum class Detonation_Status : uint8_t {
    NOT_SUPPORTED  = 0,
    NOT_OBSERVED   = 1,
    OBSERVED       = 2
};

enum class Misfire_Status : uint8_t {
    NOT_SUPPORTED = 0,
    NOT_OBSERVED  = 1,
    OBSERVED      = 2
};

enum class Debris_Status : uint8_t {
    NOT_SUPPORTED = 0,
    NOT_DETECTED  = 1,
    DETECTED      = 2
};

enum class Spark_Plug_Usage : uint8_t {
    SINGLE        = 0,
    FIRST_ACTIVE  = 1,
    SECOND_ACTIVE = 2,
    BOTH_ACTIVE   = 3
};


/***************
 * Status structs.
 * EFIs may not provide all data in the message, therefore, the following guidelines should be followed.
 * All integer fields are required unless stated otherwise.
 * All floating point fields are optional unless stated otherwise; unknown/unapplicable fields will be NaN.
 ***************/


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
    Engine_State engine_state;
 
    // If there is an error that does not fit other error types
    bool general_error;

    // Error/status fields 
    Crankshaft_Sensor_Status crankshaft_sensor_status;
    Temperature_Status temperature_status;
    Fuel_Pressure_Status fuel_pressure_status;
    Oil_Pressure_Status oil_pressure_status;
    Detonation_Status detonation_status;
    Misfire_Status misfire_status;
    Debris_Status debris_status;

    // Engine load (percent)
    uint8_t engine_load_percent;
    
    // Engine speed (revolutions per minute)
    uint32_t engine_speed_rpm;

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
    uint8_t throttle_position_percent;

    // The index of the publishing ECU.
    uint8_t ecu_index;

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

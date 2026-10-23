#include "AP_Airspeed_Scripting.h"

#if AP_AIRSPEED_SCRIPTING_ENABLED

#include <AP_HAL/AP_HAL.h>

// return the current differential_pressure in Pascal
bool AP_Airspeed_Scripting::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);
    if (last_pressure_ms == 0 || AP_HAL::millis() - last_pressure_ms > AP_AIRSPEED_SCRIPTING_TIMEOUT_MS) {
        return false;
    }
    pressure = last_pressure;
    return true;
}

// get last temperature
bool AP_Airspeed_Scripting::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);
    if (last_temperature_ms == 0 || AP_HAL::millis() - last_temperature_ms > AP_AIRSPEED_SCRIPTING_TIMEOUT_MS) {
        return false;
    }
    temperature = last_temperature;
    return true;
}

// true once the script has ever provided an airspeed reading directly
bool AP_Airspeed_Scripting::has_airspeed()
{
    WITH_SEMAPHORE(sem);
    return have_received_airspeed;
}

// return airspeed in m/s
bool AP_Airspeed_Scripting::get_airspeed(float &airspeed)
{
    WITH_SEMAPHORE(sem);
    if (last_airspeed_ms == 0 || AP_HAL::millis() - last_airspeed_ms > AP_AIRSPEED_SCRIPTING_TIMEOUT_MS) {
        return false;
    }
    airspeed = last_airspeed;
    return true;
}

bool AP_Airspeed_Scripting::handle_script_differential_pressure(float press_pa)
{
    WITH_SEMAPHORE(sem);
    if (have_received_airspeed) {
        return false;
    }
    last_pressure = press_pa;
    last_pressure_ms = AP_HAL::millis();
    return true;
}

bool AP_Airspeed_Scripting::handle_script_airspeed(float airspeed_ms)
{
    WITH_SEMAPHORE(sem);
    last_airspeed = airspeed_ms;
    last_airspeed_ms = AP_HAL::millis();

    // Once an airspeed reading has been received reject pressure readings and skip calibration
    have_received_airspeed = true;
    set_use_zero_offset();
    return true;
}

bool AP_Airspeed_Scripting::handle_script_temperature(float temperature_c)
{
    WITH_SEMAPHORE(sem);
    last_temperature = temperature_c;
    last_temperature_ms = AP_HAL::millis();
    return true;
}

#endif  // AP_AIRSPEED_SCRIPTING_ENABLED

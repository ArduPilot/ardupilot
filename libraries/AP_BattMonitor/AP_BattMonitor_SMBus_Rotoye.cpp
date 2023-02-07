#include "AP_BattMonitor_SMBus_Rotoye.h"

#include <AP_HAL/AP_HAL.h>

// Specific to Rotoye Batmon
#define BATTMONITOR_SMBUS_TEMP_EXT 0x48

// return the maximum of the internal and external temperature sensors
void AP_BattMonitor_SMBus_Rotoye::read_temp(void) {
    uint16_t t_int = 0;
    uint16_t t_ext = 0;

    /*  Both internal and external values will always be sent by Batmon.
        If no external thermistor is used, a zero-value is sent. */
    const bool have_temp_internal = read_word(BATTMONITOR_SMBUS_TEMP, t_int);
    const bool have_temp_external = read_word(BATTMONITOR_SMBUS_TEMP_EXT, t_ext);

    if (!have_temp_internal && !have_temp_external) {
        _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
        return;
    }

    _has_temperature = true;

    _state.temperature_time = AP_HAL::millis();
    _state.temperature = KELVIN_TO_C(0.1f * float(MAX(t_int, t_ext)));
}

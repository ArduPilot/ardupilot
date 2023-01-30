#include "AP_BattMonitor_SMBus_Rotoye.h"

#include <AP_HAL/AP_HAL.h>

// Specific to Rotoye Batmon
#define BATTMONITOR_SMBUS_BOARD_TEMP 0x08       // Temperature of BATMON PCB
#define BATTMONITOR_SMBUS_TEMP_EXTERNAL_1 0x48  // Temperature of first external thermistor
#define BATTMONITOR_SMBUS_TEMP_EXTERNAL_2 0x49  // Temperature of second external thermistor
#define BATTMONITOR_BATMON_DECI_CURRENT 0x41    // Current drained in deci-Ampere (int16)


// return the maximum of the internal and external temperature sensors
void AP_BattMonitor_SMBus_Rotoye::read_temp(void) {
    uint16_t t_ext_1 = 0;

    // If thermistor is not connected, an 0xFFFF value is sent
    read_word(BATTMONITOR_SMBUS_TEMP_EXTERNAL_1, t_ext_1);

    if (t_ext_1 == 0xFFFF){
        _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
        return;
    }
    _has_temperature = true;
    _state.temperature_time = AP_HAL::millis();
    _state.temperature = KELVIN_TO_C(0.1f * t_ext_1);
}

void AP_BattMonitor_SMBus_Rotoye::read_current(void) {
    uint16_t data;
    float curmA;

    // Read current in mA for existing BATMONs according to SBS spec
    if (read_word(BATTMONITOR_SMBUS_CURRENT, data)) {
        curmA = -(float)((int16_t)data) * 0.001f;

        // Read current in deciAmps for newer BATMON firmware (>v5.09)
        if (data == (uint16_t)INT16_MIN || data == INT16_MAX )
            if (read_word(BATTMONITOR_BATMON_DECI_CURRENT, data)) {
                if (data != 0xFFFF){ // backward compatibility for older BATMON versions
                    curmA = -(float)((int16_t)data) * 0.1f;
                }
            }
        _state.current_amps = curmA;
        _state.last_time_micros = AP_HAL::micros();
    }
}
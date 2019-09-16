#include "AP_BattMonitor_SMBus_Rotoye.h"

#include <AP_HAL/AP_HAL.h>

static const uint8_t cell_ids[] = {
    0x3f,  // cell 1
    0x3e,  // cell 2
    0x3d,  // cell 3
    0x3c,  // cell 4
    0x3b,  // cell 5
    0x3a,  // cell 6
    0x39,  // cell 7
    0x38,  // cell 8
    0x37,  // cell 9
    0x36   // cell 10
};

// Constructor
AP_BattMonitor_SMBus_Rotoye::AP_BattMonitor_SMBus_Rotoye(
    AP_BattMonitor &mon,
    AP_BattMonitor::BattMonitor_State &mon_state,
    AP_BattMonitor_Params &params,
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, mon_state, params, std::move(dev))
{
    _pec_supported = true;
}

void AP_BattMonitor_SMBus_Rotoye::timer()
{
    const uint32_t tnow = AP_HAL::micros();

    // read voltage (V)
    uint16_t data;
    if (read_word(BATTMONITOR_SMBUS_VOLTAGE, data)) {
        _state.voltage = (float)data * 0.001f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // read cell voltages
    for (uint8_t i = 0; i < ARRAY_SIZE(cell_ids); i++) {
        if (read_word(cell_ids[i], data)) {
            _has_cell_voltages = true;
            _state.cell_voltages.cells[i] = data;
        } else {
            _state.cell_voltages.cells[i] = UINT16_MAX;
        }
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
        return;
    }

    // read current (A)
    if (read_word(BATTMONITOR_SMBUS_CURRENT, data)) {
        _state.current_amps = -(float)((int16_t)data) * 0.001f;
        _state.last_time_micros = tnow;
    }

    read_full_charge_capacity();

    // FIXME: Perform current integration if the remaining capacity can't be requested
    read_remaining_capacity();

    read_temp();

    read_serial_number();
}


#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SMBUS_NEODESIGN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_BattMonitor.h"

#include "AP_BattMonitor_SMBus_NeoDesign.h"

#define BATTMONITOR_ND_CELL_COUNT                0x5C    // cell-count register
#define BATTMONITOR_ND_CELL_START                0x30    // first cell register

// Constructor
AP_BattMonitor_SMBus_NeoDesign::AP_BattMonitor_SMBus_NeoDesign(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params)
    : AP_BattMonitor_SMBus(mon, mon_state, params, AP_BATTMONITOR_SMBUS_BUS_INTERNAL)
{
    _pec_supported = true;
}

void AP_BattMonitor_SMBus_NeoDesign::timer()
{
    uint16_t data;
    // Get the cell count once, it's not likely to change in flight
    if (_cell_count == 0) {
        if (!read_word(BATTMONITOR_ND_CELL_COUNT, data)) {
            return; // something wrong, don't try anything else
        }
        // constrain maximum cellcount in case of i2c corruption
        if (data > max_cell_count) {
            _cell_count = max_cell_count;
        } else {
            _cell_count = data;
        }
    }

    bool read_all_cells = true;
    for(uint8_t i = 0; i < _cell_count; ++i) {
        if(read_word(BATTMONITOR_ND_CELL_START + i, data)) {
            _state.cell_voltages.cells[i] = data;
            _has_cell_voltages = true;
        } else {
            read_all_cells = false;
        }
    }

    const uint32_t tnow = AP_HAL::micros();

    if (read_all_cells && (_cell_count > 0)) {
        uint32_t summed = 0;
        for (int i = 0; i < _cell_count; i++) {
            summed += _state.cell_voltages.cells[i];
        }
        _state.voltage = (float)summed * 1e-3f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    } else if (read_word(BATTMONITOR_SMBUS_VOLTAGE, data)) {
            // fallback to the voltage register if we didn't manage to poll the cells
            _state.voltage = (float)data * 1e-3f;
            _state.last_time_micros = tnow;
            _state.healthy = true;
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
        // do not attempt to read any more data from battery
        return;
    }

    // read current (A)
    if (read_word(BATTMONITOR_SMBUS_CURRENT, data)) {
        _state.current_amps = -(float)((int16_t)data) * 1e-3f;
        _state.last_time_micros = tnow;
    }

    read_full_charge_capacity();
    read_remaining_capacity();
    read_temp();
}

#endif  // AP_BATTERY_SMBUS_NEODESIGN_ENABLED

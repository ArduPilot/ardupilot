#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_NeoDesign.h"

#define BATTMONITOR_ND_CELL_COUNT                0x5C    // cell-count register
#define BATTMONITOR_ND_CELL_START                0x30    // first cell register
#define MAX_ALLOWED_CELLS                        10

// Constructor
AP_BattMonitor_SMBus_NeoDesign::AP_BattMonitor_SMBus_NeoDesign(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, mon_state, params, std::move(dev))
{
    _pec_supported = false;
}

void AP_BattMonitor_SMBus_NeoDesign::timer()
{
    uint8_t buff[8];
    uint16_t data;
    uint32_t tnow = AP_HAL::micros();

    // Get the cell count once. Not likely to change in flight, LOL
    if (_cell_count == 0) {
        if (read_block_bare(BATTMONITOR_ND_CELL_COUNT, buff, 2) == 2) {
            _cell_count = MIN(MAX_ALLOWED_CELLS, buff[1] << 8 | buff[0]);
        } else {
            return; // Something wrong, don't try anything else
        }
    }

    if (_cell_count > 0) {
        for(uint8_t i = 0; i < _cell_count; ++i) {
            if(read_block_bare(BATTMONITOR_ND_CELL_START + i, buff, 2) == 2) {
                uint16_t cell = (buff[1] << 8 | buff[0]);
                _state.cell_voltages.cells[i] = cell;
                _has_cell_voltages = true;
            }
        }

        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // read voltage (V)
    if (read_word(BATTMONITOR_SMBUS_VOLTAGE, data)) {
        _state.voltage = (float)data / 1000.0f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
        // do not attempt to ready any more data from battery
        return;
    }

    // read current
    if (read_block_bare(BATTMONITOR_SMBUS_CURRENT, buff, 2) == 2) {
        uint16_t amps = (0xffff) - (buff[1] << 8 | buff[0]);
        _state.current_amps = (float)(amps) / 1000.0f;
        _state.last_time_micros = tnow;
    }

    read_full_charge_capacity();
    read_remaining_capacity();
    read_temp();
}

uint8_t AP_BattMonitor_SMBus_NeoDesign::read_block_bare(uint8_t reg, uint8_t* data, uint8_t max_len) const
{
    // read bytes
    if (!_dev->read_registers(reg, data, max_len)) {
        return 0;
    }

    // return success
    return max_len;
}


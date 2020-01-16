#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_NeoDesign.h"
#include <utility>

#define BATTMONITOR_ND_CELL_COUNT                0x5C    // cell-count register
#define BATTMONITOR_ND_CELL_START                0x30    // first cell register
#define BATTMONITOR_ND_CURRENT                   0x0a    // current register
#define BATTMONITOR_ND_TEMPERATURE               0x08    // temperature register

// Constructor
AP_BattMonitor_SMBus_NeoDesign::AP_BattMonitor_SMBus_NeoDesign(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, mon_state, params, std::move(dev))
    , _cell_count(0)
{
    _pec_supported = false;
}

void AP_BattMonitor_SMBus_NeoDesign::timer()
{
    uint8_t buff[8];
    uint32_t tnow = AP_HAL::micros();

    // Get the cell count once. Not likely to change in flight, LOL
    if (_cell_count == 0) {
        if (read_block_bare(BATTMONITOR_ND_CELL_COUNT, buff, 2)) {
            _cell_count = buff[1] << 8 | buff[0];
        } else {
            return; // Something wrong, don't try anything else
        }
    }

    if (_cell_count > 0) {
        uint8_t vbuf[2];

        float pack_voltage_mv = 0.0f;

        for(uint8_t i = 0; i < _cell_count; ++i) {
            if(read_block_bare(BATTMONITOR_ND_CELL_START + i, vbuf, 2)) {
                uint16_t cell = (vbuf[1] << 8 | vbuf[0]);
                _state.cell_voltages.cells[i] = cell;
                pack_voltage_mv += (float)cell;
                _has_cell_voltages = true;
            }
        }

        _state.voltage = pack_voltage_mv * 1e-3;
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
    if (read_block_bare(BATTMONITOR_ND_CURRENT, buff, 2) == 2) {
        uint16_t amps = (0xffff) - (buff[1] << 8 | buff[0]);

        _state.current_amps = (float)(amps) / 1000.0f;
        _state.last_time_micros = tnow;
    }

    read_full_charge_capacity();
    read_remaining_capacity();
    read_temperature();
}

bool AP_BattMonitor_SMBus_NeoDesign::read_temperature() 
{
    uint8_t buff[2];

    if (read_block_bare(BATTMONITOR_SMBUS_TEMP, buff, 2) == 2) {
        _state.temperature_time = AP_HAL::millis();
        uint16_t data = (buff[1] << 8 | buff[0]);
        _state.temperature = ((float)(data - 2731)) * 0.1f;
        return true;
    }

    return false;
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


#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SMBUS_SUI_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"

#include "AP_BattMonitor_SMBus_SUI.h"

extern const AP_HAL::HAL& hal;

#define REG_CELL_VOLTAGE        0x28
#define REG_CURRENT             0x2a

// maximum number of cells that we can read data for
#define SUI_MAX_CELL_READ       4

// Constructor
AP_BattMonitor_SMBus_SUI::AP_BattMonitor_SMBus_SUI(AP_BattMonitor &mon,
        AP_BattMonitor::BattMonitor_State &mon_state,
        AP_BattMonitor_Params &params,
        uint8_t _cell_count)
    : AP_BattMonitor_SMBus(mon, mon_state, params, AP_BATTMONITOR_SMBUS_BUS_INTERNAL),
      cell_count(_cell_count)
{
    _pec_supported = false;
}

void AP_BattMonitor_SMBus_SUI::init(void)
{
    AP_BattMonitor_SMBus::init();
    if (_dev) {
        _dev->set_retries(2);
    }
    if (_dev && timer_handle) {
        // run twice as fast for two phases
        _dev->adjust_periodic_callback(timer_handle, 50000);
    }
}

void AP_BattMonitor_SMBus_SUI::timer()
{
    uint32_t tnow = AP_HAL::micros();

    // we read in two phases as the device can stall if you read
    // current too rapidly after voltages
    phase_voltages = !phase_voltages;

    if (phase_voltages) {
        read_cell_voltages();
        update_health();
        return;
    }

    // read current
    int32_t current_ma;
    if (read_block_bare(REG_CURRENT, (uint8_t *)&current_ma, sizeof(current_ma))) {
        _state.current_amps = current_ma * -0.001;
        _state.last_time_micros = tnow;
    }

    read_full_charge_capacity();

    read_temp();
    read_serial_number();
    read_remaining_capacity();
    update_health();
}

// read_bare_block - returns true if successful
bool AP_BattMonitor_SMBus_SUI::read_block_bare(uint8_t reg, uint8_t* data, uint8_t len) const
{
    // read bytes
    if (!_dev->read_registers(reg, data, len)) {
        return false;
    }

    // return success
    return true;
}

void AP_BattMonitor_SMBus_SUI::read_cell_voltages()
{
    // read cell voltages
    uint16_t voltbuff[SUI_MAX_CELL_READ];

    if (!read_block(REG_CELL_VOLTAGE, (uint8_t *)voltbuff, sizeof(voltbuff))) {
        return;
    }
    float pack_voltage_mv = 0.0f;
        
    for (uint8_t i = 0; i < MIN(SUI_MAX_CELL_READ, cell_count); i++) {
        const uint16_t cell = voltbuff[i];
        _state.cell_voltages.cells[i] = cell;
        pack_voltage_mv += (float)cell;
    }

    if (cell_count >= SUI_MAX_CELL_READ) {
        // we can't read voltage of all cells. get overall pack voltage to work out
        // an average for remaining cells
        uint16_t total_mv;
        if (read_word(BATTMONITOR_SMBUS_VOLTAGE, total_mv)) {
            // if total voltage is below pack_voltage_mv then we will
            // read zero volts for the extra cells.
            total_mv = MAX(total_mv, pack_voltage_mv);
            const uint16_t cell_mv = (total_mv - pack_voltage_mv) / (cell_count - SUI_MAX_CELL_READ);
            for (uint8_t i = SUI_MAX_CELL_READ; i < cell_count; i++) {
                _state.cell_voltages.cells[i] = cell_mv;
            }
            pack_voltage_mv = total_mv;
        } else {
            // we can't get total pack voltage. Use average of cells we have so far
            const uint16_t cell_mv = pack_voltage_mv / SUI_MAX_CELL_READ;
            for (uint8_t i = SUI_MAX_CELL_READ; i < cell_count; i++) {
                _state.cell_voltages.cells[i] = cell_mv;
            }
            pack_voltage_mv += cell_mv * (cell_count - SUI_MAX_CELL_READ);
        }
    }

    _has_cell_voltages = true;

    // accumulate the pack voltage out of the total of the cells
    _state.voltage = pack_voltage_mv * 0.001;
    last_volt_read_us = AP_HAL::micros();
}

/*
  update healthy flag
 */
void AP_BattMonitor_SMBus_SUI::update_health()
{
    uint32_t now = AP_HAL::micros();
    _state.healthy = (now - last_volt_read_us < AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) &&
        (now - _state.last_time_micros < AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS);
}

#endif  // AP_BATTERY_SMBUS_SUI_ENABLED

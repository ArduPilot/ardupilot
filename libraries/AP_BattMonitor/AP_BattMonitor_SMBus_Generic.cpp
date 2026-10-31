#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SMBUS_GENERIC_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"

#include "AP_BattMonitor_SMBus_Generic.h"

uint8_t smbus_cell_ids[] = { 0x3f,  // cell 1
                             0x3e,  // cell 2
                             0x3d,  // cell 3
                             0x3c,  // cell 4
                             0x3b,  // cell 5
                             0x3a,  // cell 6
                             0x39,  // cell 7
                             0x38,  // cell 8
                             0x37,  // cell 9
                             0x36,  // cell 10
                             0x35,  // cell 11
                             0x34,  // cell 12
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                             0x33,  // cell 13
                             0x32   // cell 14
#endif
};

#define SMBUS_CELL_COUNT_CHECK_TIMEOUT       15     // check cell count for up to 15 seconds

/*
 * Other potentially useful registers, listed here for future use
 * #define BATTMONITOR_SMBUS_MAXELL_CHARGE_STATUS         0x0d    // relative state of charge
 * #define BATTMONITOR_SMBUS_MAXELL_BATTERY_STATUS        0x16    // battery status register including alarms
 * #define BATTMONITOR_SMBUS_MAXELL_BATTERY_CYCLE_COUNT   0x17    // cycle count
 * #define BATTMONITOR_SMBUS_MAXELL_DESIGN_VOLTAGE        0x19    // design voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_MANUFACTURE_DATE      0x1b    // manufacturer date
 * #define BATTMONITOR_SMBUS_MAXELL_SERIALNUM             0x1c    // serial number register
 * #define BATTMONITOR_SMBUS_MAXELL_HEALTH_STATUS         0x4f    // state of health
 * #define BATTMONITOR_SMBUS_MAXELL_SAFETY_ALERT          0x50    // safety alert
 * #define BATTMONITOR_SMBUS_MAXELL_SAFETY_STATUS         0x51    // safety status
 * #define BATTMONITOR_SMBUS_MAXELL_PF_ALERT              0x52    // safety status
 * #define BATTMONITOR_SMBUS_MAXELL_PF_STATUS             0x53    // safety status
*/

// Constructor
AP_BattMonitor_SMBus_Generic::AP_BattMonitor_SMBus_Generic(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params)
    : AP_BattMonitor_SMBus(mon, mon_state, params, AP_BATTMONITOR_SMBUS_BUS_EXTERNAL)
{}

void AP_BattMonitor_SMBus_Generic::timer()
{
	// read SMBus SpecificationInfo()
    if (!read_specification_info()) {
        return;
    }

    uint16_t data;
    uint32_t tnow = AP_HAL::micros();

    // read voltage (V)
    if (read_word(BATTMONITOR_SMBUS_VOLTAGE, data)) {
        _state.voltage = (float)data * _v_multiplier * 0.001f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // assert that BATTMONITOR_SMBUS_NUM_CELLS_MAX must be no more than smbus_cell_ids
    static_assert(BATTMONITOR_SMBUS_NUM_CELLS_MAX <= ARRAY_SIZE(smbus_cell_ids), "BATTMONITOR_SMBUS_NUM_CELLS_MAX must be no more than smbus_cell_ids");

    // check cell count
    if (!_cell_count_fixed) {
        if (_state.healthy) {
            // when battery first becomes healthy, start check of cell count
            if (_cell_count_check_start_us == 0) {
                _cell_count_check_start_us = tnow;
            }
            if (tnow - _cell_count_check_start_us > (SMBUS_CELL_COUNT_CHECK_TIMEOUT * 1e6)) {
                // give up checking cell count after 15sec of continuous healthy battery reads
                _cell_count_fixed = true;
            }
        } else {
            // if battery becomes unhealthy restart cell count check
            _cell_count_check_start_us = 0;
        }
    }

    // we loop over something limited by
    // BATTMONITOR_SMBUS_NUM_CELLS_MAX but assign into something
    // limited by AP_BATT_MONITOR_CELLS_MAX - so make sure we won't
    // over-write:
    static_assert(BATTMONITOR_SMBUS_NUM_CELLS_MAX <= ARRAY_SIZE(_state.cell_voltages.cells), "BATTMONITOR_SMBUS_NUM_CELLS_MAX must be <= number of cells in state voltages");

    // read cell voltages
    for (uint8_t i = 0; i < (_cell_count_fixed ? _cell_count : BATTMONITOR_SMBUS_NUM_CELLS_MAX); i++) {
        if (read_word(smbus_cell_ids[i], data) && (data > 0) && (data < UINT16_MAX)) {
            _has_cell_voltages = true;
            _state.cell_voltages.cells[i] = data * _v_multiplier;
            _last_cell_update_us[i] = tnow;
            if (!_cell_count_fixed) {
                _cell_count = MAX(_cell_count, i + 1);
            }
        } else if ((tnow - _last_cell_update_us[i]) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
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
        _state.current_amps = -(float)((int16_t)data) * _i_multiplier * 0.001f;
        _state.last_time_micros = tnow;
    }

    read_full_charge_capacity();

    // FIXME: Perform current integration if the remaining capacity can't be requested
    read_remaining_capacity();

    read_temp();

    read_serial_number();

    read_cycle_count();
}

// Read SpecificationInfo(), check if PEC is supported and update voltages and current multipliers
// returns true once SpecificationInfo() is read from the Smart Battery
bool AP_BattMonitor_SMBus_Generic::read_specification_info()
{
    // exit immediately if we have already read SpecificationInfo()
    if (_specification_info_confirmed) {
        return true;
    }

    // specification info
    uint16_t data;
    if (!read_word(BATTMONITOR_SMBUS_SPECIFICATION_INFO, data)) {
        return false;
    }

    // check manufacturer name
    uint8_t buff[AP_BATTMONITOR_SMBUS_READ_BLOCK_MAXIMUM_TRANSFER + 1] {};
    if (read_block(BATTMONITOR_SMBUS_MANUFACTURE_NAME, buff, sizeof(buff))) {
        // Hitachi maxell batteries do not support PEC
        if (strcmp((char*)buff, "Hitachi maxell") == 0) {
            _pec_supported = false;
            _v_multiplier = 1;
            _i_multiplier = 1;
            _specification_info_confirmed = true;
            return true;
        }
    }
    // extract version
    uint8_t version = (data & 0xF0) >> 4;

    // version less than 0011b (i.e. 3) do not support PEC
    _pec_supported = version >= 3;

    // extract and update voltage multiplier
    switch ((data & 0xF00) >> 8) {
    case 1:
        _v_multiplier = 10;
        break;
    case 2:
        _v_multiplier = 100;
        break;
    case 3:
        _v_multiplier = 1000;
        break;
    default:
        _v_multiplier = 1;
        break;
    }

    // extract and update current multiplier
    switch ((data & 0xF000) >> 12) {
    case 1:
        _i_multiplier = 10;
        break;
    case 2:
        _i_multiplier = 100;
        break;
    case 3:
        _i_multiplier = 1000;
        break;
    default:
        _i_multiplier = 1;
        break;
    }
    
	_specification_info_confirmed = true;
	return true;
}

#endif  // AP_BATTERY_SMBUS_GENERIC_ENABLED

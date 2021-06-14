#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_Generic.h"
#include <utility>

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

#define SMBUS_READ_BLOCK_MAXIMUM_TRANSFER    0x20   // A Block Read or Write is allowed to transfer a maximum of 32 data bytes.
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
                                                   AP_BattMonitor_Params &params,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, mon_state, params, std::move(dev))
{}

void AP_BattMonitor_SMBus_Generic::timer()
{
	// check if PEC is supported
    if (!check_pec_support()) {
        return;
    }

    uint16_t data;
    uint32_t tnow = AP_HAL::micros();

    // read voltage (V)
    if (read_word(BATTMONITOR_SMBUS_VOLTAGE, data)) {
        _state.voltage = (float)data / 1000.0f;
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

    // we loop over something limted by
    // BATTMONITOR_SMBUS_NUM_CELLS_MAX but assign into something
    // limited by AP_BATT_MONITOR_CELLS_MAX - so make sure we won't
    // over-write:
    static_assert(BATTMONITOR_SMBUS_NUM_CELLS_MAX <= ARRAY_SIZE(_state.cell_voltages.cells), "BATTMONITOR_SMBUS_NUM_CELLS_MAX must be <= number of cells in state voltages");

    // read cell voltages
    for (uint8_t i = 0; i < (_cell_count_fixed ? _cell_count : BATTMONITOR_SMBUS_NUM_CELLS_MAX); i++) {
        if (read_word(smbus_cell_ids[i], data) && (data > 0) && (data < UINT16_MAX)) {
            _has_cell_voltages = true;
            _state.cell_voltages.cells[i] = data;
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
        _state.current_amps = -(float)((int16_t)data) / 1000.0f;
        _state.last_time_micros = tnow;
    }

    read_full_charge_capacity();

    // FIXME: Perform current integration if the remaining capacity can't be requested
    read_remaining_capacity();

    read_temp();

    read_serial_number();

    read_cycle_count();
}

// read_block - returns number of characters read if successful, zero if unsuccessful
uint8_t AP_BattMonitor_SMBus_Generic::read_block(uint8_t reg, uint8_t* data, bool append_zero) const
{
    // get length
    uint8_t bufflen;
    // read byte (first byte indicates the number of bytes in the block)
    if (!_dev->read_registers(reg, &bufflen, 1)) {
        return 0;
    }

    // sanity check length returned by smbus
    if (bufflen == 0 || bufflen > SMBUS_READ_BLOCK_MAXIMUM_TRANSFER) {
        return 0;
    }

    // buffer to hold results (2 extra byte returned holding length and PEC)
    const uint8_t read_size = bufflen + 1 + (_pec_supported ? 1 : 0);
    uint8_t buff[read_size];

    // read bytes
    if (!_dev->read_registers(reg, buff, read_size)) {
        return 0;
    }

    // check PEC
    if (_pec_supported) {
        uint8_t pec = get_PEC(AP_BATTMONITOR_SMBUS_I2C_ADDR, reg, true, buff, bufflen+1);
        if (pec != buff[bufflen+1]) {
            return 0;
        }
    }

    // copy data (excluding PEC)
    memcpy(data, &buff[1], bufflen);

    // optionally add zero to end
    if (append_zero) {
        data[bufflen] = '\0';
    }

    // return success
    return bufflen;
}

// check if PEC supported with the version value in SpecificationInfo() function
// returns true once PEC is confirmed as working or not working
bool AP_BattMonitor_SMBus_Generic::check_pec_support()
{
    // exit immediately if we have already confirmed pec support
    if (_pec_confirmed) {
        return true;
    }

    // specification info
    uint16_t data;
    if (!read_word(BATTMONITOR_SMBUS_SPECIFICATION_INFO, data)) {
        return false;
    }

    // extract version
    uint8_t version = (data & 0xF0) >> 4;

    // version less than 0011b (i.e. 3) do not support PEC
    if (version < 3) {
        _pec_supported = false;
        _pec_confirmed = true;
        return true;
    }

    // check manufacturer name
    uint8_t buff[SMBUS_READ_BLOCK_MAXIMUM_TRANSFER + 1];
    if (read_block(BATTMONITOR_SMBUS_MANUFACTURE_NAME, buff, true)) {
        // Hitachi maxell batteries do not support PEC
        if (strcmp((char*)buff, "Hitachi maxell") == 0) {
            _pec_supported = false;
            _pec_confirmed = true;
            return true;
        }
    }

    // assume all other batteries support PEC
	_pec_supported = true;
	_pec_confirmed = true;
	return true;
}


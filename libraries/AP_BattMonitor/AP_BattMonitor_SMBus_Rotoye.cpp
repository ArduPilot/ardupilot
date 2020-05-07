#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_Rotoye.h"
#include <utility>

#define BATTMONITOR_SMBUS_Rotoye_NUM_CELLS 10
uint8_t Rotoye_cell_ids[] = { 0x3f,  // cell 1
                              0x3e,  // cell 2
                              0x3d,  // cell 3
                              0x3c,  // cell 4
                              0x3b,  // cell 5
                              0x3a,  // cell 6
                              0x39,  // cell 7
                              0x38,  // cell 8
                              0x37,  // cell 9
                              0x36}; // cell 10

#define SMBUS_READ_BLOCK_MAXIMUM_TRANSFER    0x20   // A Block Read or Write is allowed to transfer a maximum of 32 data bytes.

/*
 * Other potentially useful registers, listed here for future use
 * #define BATTMONITOR_SMBUS_Rotoye_CHARGE_STATUS         0x0d    // relative state of charge
 * #define BATTMONITOR_SMBUS_Rotoye_BATTERY_STATUS        0x16    // battery status register including alarms
 * #define BATTMONITOR_SMBUS_Rotoye_BATTERY_CYCLE_COUNT   0x17    // cycle count
 * #define BATTMONITOR_SMBUS_Rotoye_DESIGN_VOLTAGE        0x19    // design voltage register
 * #define BATTMONITOR_SMBUS_Rotoye_MANUFACTURE_DATE      0x1b    // manufacturer date
 * #define BATTMONITOR_SMBUS_Rotoye_SERIALNUM             0x1c    // serial number register
 * #define BATTMONITOR_SMBUS_Rotoye_HEALTH_STATUS         0x4f    // state of health
 * #define BATTMONITOR_SMBUS_Rotoye_SAFETY_ALERT          0x50    // safety alert
 * #define BATTMONITOR_SMBUS_Rotoye_SAFETY_STATUS         0x51    // safety status
 * #define BATTMONITOR_SMBUS_Rotoye_PF_ALERT              0x52    // safety status
 * #define BATTMONITOR_SMBUS_Rotoye_PF_STATUS             0x53    // safety status
*/

// Constructor
AP_BattMonitor_SMBus_Rotoye::AP_BattMonitor_SMBus_Rotoye(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, mon_state, params, std::move(dev))
{
    _pec_supported = true;
}

void AP_BattMonitor_SMBus_Rotoye::timer()
{

    uint16_t data;
    const uint32_t tnow = AP_HAL::micros();

    // read voltage (V)
    if (read_word(BATTMONITOR_SMBUS_VOLTAGE, data)) {
        _state.voltage = (float)data * 0.001f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // read cell voltages
    for (uint8_t i = 0; i < BATTMONITOR_SMBUS_Rotoye_NUM_CELLS; i++) {
        if (read_word(Rotoye_cell_ids[i], data)) {
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


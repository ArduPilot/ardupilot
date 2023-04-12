#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SMBUS_SOLO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include <utility>

#include "AP_BattMonitor_SMBus_Solo.h"

#define BATTMONITOR_SMBUS_SOLO_CELL_VOLTAGE         0x28    // cell voltage register
#define BATTMONITOR_SMBUS_SOLO_CURRENT              0x2a    // current register
#define BATTMONITOR_SMBUS_SOLO_BUTTON_DEBOUNCE      6       // button held down for 5 intervals will cause a power off event
#define BATTMONITOR_SMBUS_SOLO_NUM_CELLS            4       // solo's battery pack is 4S

/*
 * Other potentially useful registers, listed here for future use
 * #define BATTMONITOR_SMBUS_SOLO_VOLTAGE           0x09    // voltage register
 * #define BATTMONITOR_SMBUS_SOLO_BATTERY_STATUS    0x16    // battery status register including alarms
 * #define BATTMONITOR_SMBUS_SOLO_DESIGN_CAPACITY   0x18    // design capacity register
 * #define BATTMONITOR_SMBUS_SOLO_DESIGN_VOLTAGE    0x19    // design voltage register
 * #define BATTMONITOR_SMBUS_SOLO_SERIALNUM         0x1c    // serial number register
 * #define BATTMONITOR_SMBUS_SOLO_MANUFACTURE_NAME  0x20    // manufacturer name
 * #define BATTMONITOR_SMBUS_SOLO_DEVICE_NAME       0x21    // device name
 * #define BATTMONITOR_SMBUS_SOLO_DEVICE_CHEMISTRY  0x22    // device chemistry
 * #define BATTMONITOR_SMBUS_SOLO_MANUFACTURE_INFO  0x25    // manufacturer info including cell voltage
 */

// Constructor
AP_BattMonitor_SMBus_Solo::AP_BattMonitor_SMBus_Solo(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params)
    : AP_BattMonitor_SMBus(mon, mon_state, params, AP_BATTMONITOR_SMBUS_BUS_INTERNAL)
{
    _pec_supported = true;
}

void AP_BattMonitor_SMBus_Solo::timer()
{
    uint8_t buff[8];
    uint32_t tnow = AP_HAL::micros();


    // read cell voltages
    if (read_block(BATTMONITOR_SMBUS_SOLO_CELL_VOLTAGE, buff, 8)) {
        float pack_voltage_mv = 0.0f;
        for (uint8_t i = 0; i < BATTMONITOR_SMBUS_SOLO_NUM_CELLS; i++) {
            uint16_t cell = buff[(i * 2) + 1] << 8 | buff[i * 2];
            _state.cell_voltages.cells[i] = cell;
            pack_voltage_mv += (float)cell;
        }
        _has_cell_voltages = true;

        // accumulate the pack voltage out of the total of the cells
        // because the Solo's I2C bus is so noisy, it's worth not spending the
        // time and bus bandwidth to request the pack voltage as a seperate
        // transaction
        _state.voltage = pack_voltage_mv * 1e-3f;
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
    if (read_block(BATTMONITOR_SMBUS_SOLO_CURRENT, buff, 4) == 4) {
        _state.current_amps = -(float)((int32_t)((uint32_t)buff[3]<<24 | (uint32_t)buff[2]<<16 | (uint32_t)buff[1]<<8 | (uint32_t)buff[0])) * 0.001f;
        _state.last_time_micros = tnow;
    }

    read_full_charge_capacity();
    read_remaining_capacity();

    // read the button press indicator
    if (read_block(BATTMONITOR_SMBUS_MANUFACTURE_DATA, buff, 6) == 6) {
        bool pressed = (buff[1] >> 3) & 0x01;

        if (_button_press_count >= BATTMONITOR_SMBUS_SOLO_BUTTON_DEBOUNCE) {
            // vehicle will power off, set state flag
            _state.is_powering_off = true;
        } else if (pressed) {
            // battery will power off if the button is held
            _button_press_count++;
        } else {
            // button released, reset counters
            _button_press_count = 0;
        }
    }

    read_temp();

    read_serial_number();

    read_cycle_count();
}

#endif  // AP_BATTERY_SMBUS_SOLO_ENABLED

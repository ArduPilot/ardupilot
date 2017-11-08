#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_SUI.h"
#include <utility>

#define BATTMONITOR_SMBUS_SUI_MANUFACTURE_DATA     0x23    /// manufacturer data
#define BATTMONITOR_SMBUS_SUI_CELL_VOLTAGE         0x28    // cell voltage register

#define BATTMONITOR_SMBUS_SUI_BUTTON_DEBOUNCE      3       // button held down for 3 intervals will cause a power off event

#define BATTMONITOR_SMBUS_ENDURANCE_NUM_CELLS 6

/*
 * Other potentially useful registers, listed here for future use
 * #define BATTMONITOR_SMBUS_SUI_VOLTAGE           0x09    // voltage register
 * #define BATTMONITOR_SMBUS_SUI_BATTERY_STATUS    0x16    // battery status register including alarms
 * #define BATTMONITOR_SMBUS_SOLO_DESIGN_CAPACITY   0x18    // design capacity register
 * #define BATTMONITOR_SMBUS_SOLO_DESIGN_VOLTAGE    0x19    // design voltage register
 * #define BATTMONITOR_SMBUS_SOLO_SERIALNUM         0x1c    // serial number register
 * #define BATTMONITOR_SMBUS_SOLO_MANUFACTURE_NAME  0x20    // manufacturer name
 * #define BATTMONITOR_SMBUS_SOLO_DEVICE_NAME       0x21    // device name
 * #define BATTMONITOR_SMBUS_SOLO_DEVICE_CHEMISTRY  0x22    // device chemistry
 * #define BATTMONITOR_SMBUS_SOLO_MANUFACTURE_INFO  0x25    // manufacturer info including cell voltage
 */

// Constructor
AP_BattMonitor_SMBus_SUI::AP_BattMonitor_SMBus_SUI(AP_BattMonitor &mon,
                    AP_BattMonitor::BattMonitor_State &mon_state,
                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                    uint8_t full_cap_register,
                    uint8_t rem_cap_register,
                    uint8_t temp_register,
                    uint8_t serial_register,
                    uint8_t current_register,
                    uint8_t voltage_register,
                    uint8_t cell_count)
    : AP_BattMonitor_SMBus(mon, mon_state, std::move(dev), full_cap_register, rem_cap_register, temp_register, serial_register),
    _cell_count(cell_count)
{
    _pec_supported = true;
    _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SMBus_SUI::timer, void));
}

void AP_BattMonitor_SMBus_SUI::timer()
{
    uint8_t buff[8];
    uint32_t tnow = AP_HAL::micros();

    // read cell voltages
    if(_cell_count > 0) {
        if (read_block(BATTMONITOR_SMBUS_SUI_CELL_VOLTAGE, buff, 8, false)) {
            float pack_voltage_mv = 0.0f;
            for (uint8_t i = 0; i < _cell_count; i++) {
                uint16_t cell = buff[(i * 2) + 1] << 8 | buff[i * 2];
                _state.cell_voltages.cells[i] = cell;
                pack_voltage_mv += (float)cell;
            }
            _has_cell_voltages = true;

            // accumulate the pack voltage out of the total of the cells
            // because the Solo's I2C bus is so noisy, it's worth not spending the
            // time and bus bandwidth to request the pack voltage as a seperate
            // transaction
            _state.voltage = pack_voltage_mv * 1e-3;
            _state.last_time_micros = tnow;
            _state.healthy = true;
        }
    } else {
        // voltage will be read below
        _state.healthy = true;
        _has_cell_voltages = false;
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
        // do not attempt to ready any more data from battery
        return;
    }

    // read current
    if (read_block(_current_register, buff, 4, false) == 4) {
        _state.current_amps = -(float)((int32_t)((uint32_t)buff[3]<<24 | (uint32_t)buff[2]<<16 | (uint32_t)buff[1]<<8 | (uint32_t)buff[0])) / 1000.0f;
        _state.last_time_micros = tnow;
    }

    // read voltage
    if(_cell_count <= 0) {
        if (read_block(_voltage_register, buff, 2, false) == 2) {
            _state.voltage = (float)((uint32_t)buff[1]<<8 | (uint32_t)buff[0]) / 1000.0f;
            _state.last_time_micros = tnow;
        }
    }

    if(_full_charge_capacity == 0) {
        if(read_block(_full_cap_register, buff, 4, false) == 4) {
            _full_charge_capacity = ((uint32_t)buff[1]<<8 | (uint32_t)buff[0]);
        }
    }

    if(_capacity == 0) {
        _capacity = get_capacity();
    }

    if (_capacity > 0) {
        if(read_block(_rem_cap_register, buff, 4, false) == 4) {
            uint16_t data = ((uint16_t)buff[1]<<8 | (uint16_t)buff[0]);
            _state.current_total_mah = MAX(0, _capacity - data);
        }
    }

    // read the button press indicator
    if (read_block(BATTMONITOR_SMBUS_SUI_MANUFACTURE_DATA, buff, 6, false) == 6) {
        bool pressed = (buff[1] >> 3) & 0x01;

        if (_button_press_count >= BATTMONITOR_SMBUS_SUI_BUTTON_DEBOUNCE) {
            // battery will power off
            _state.is_powering_off = true;

        } else if (pressed) {
            // battery will power off if the button is held
            _button_press_count++;

        } else {
            // button released, reset counters
            _button_press_count = 0;
            _state.is_powering_off = false;
        }
        AP_Notify::flags.powering_off = _state.is_powering_off;
    }

    read_temp();

    if(_serial_register) {
        read_serial_number();
    }
}

// read_block - returns number of characters read if successful, zero if unsuccessful
uint8_t AP_BattMonitor_SMBus_SUI::read_block(uint8_t reg, uint8_t* data, uint8_t max_len, bool append_zero) const
{
    uint8_t buff[max_len+2];    // buffer to hold results (2 extra byte returned holding length and PEC)

    // read bytes
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return 0;
    }

    // get length
    uint8_t bufflen = buff[0];

    // sanity check length returned by smbus
    if (bufflen == 0 || bufflen > max_len) {
        return 0;
    }

    // check PEC
    uint8_t pec = get_PEC(AP_BATTMONITOR_SMBUS_I2C_ADDR, reg, true, buff, bufflen+1);
    if (pec != buff[bufflen+1]) {
        return 0;
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

//
// Xray battery registers
//
#define BATTMONITOR_XRAY_NUM_CELLS 3
#define XRAY_VOLTAGE        0x09
#define XRAY_CURRENT        0x2a
#define XRAY_REMCAP         0x0f
#define XRAY_FULLCAP        0x10
#define XRAY_TEMP           0x08
#define XRAY_SERIAL         0x1C
#define XRAY_MANUF_NAME     0x20
#define XRAY_DEVICE_NAME    0x21
#define XRAY_DEVICE_CHEM    0x22
#define XRAY_MANUF_INFO     0x25

//
// Xray battery
//
AP_BattMonitor_SMBus_Xray::AP_BattMonitor_SMBus_Xray(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus_SUI(mon, mon_state, std::move(dev),
        XRAY_FULLCAP, XRAY_REMCAP, XRAY_TEMP, XRAY_SERIAL, XRAY_CURRENT, XRAY_VOLTAGE, BATTMONITOR_XRAY_NUM_CELLS)
{
}

//
// Endurance battery
//
#define ENDURANCE_VOLTAGE 0x02
#define ENDURANCE_CURRENT 0x03
#define ENDURANCE_FULLCAP 0x11
#define ENDURANCE_REMCAP 0x13
#define ENDURANCE_TEMP 0x04

AP_BattMonitor_SMBus_Endurance::AP_BattMonitor_SMBus_Endurance(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus_SUI(mon, mon_state, std::move(dev), 
        ENDURANCE_FULLCAP, ENDURANCE_REMCAP, ENDURANCE_TEMP, 0, ENDURANCE_CURRENT, ENDURANCE_VOLTAGE, 0)
{
}


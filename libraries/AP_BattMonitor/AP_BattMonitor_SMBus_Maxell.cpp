#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_Maxell.h"
#include <utility>

extern const AP_HAL::HAL& hal;

#include <AP_HAL/AP_HAL.h>


#define BATTMONITOR_SMBUS_MAXELL_VOLTAGE    0x09    // voltage register
#define BATTMONITOR_SMBUS_MAXELL_CURRENT    0x0a    // current register

/*
 * Other potentially useful registers, listed here for future use
 * #define BATTMONITOR_SMBUS_MAXELL_TEMP      0x08    // temperature register
 * #define BATTMONITOR_SMBUS_MAXELL_CHARGE_STATUS         0x0d    // relative state of charge
 * #define BATTMONITOR_SMBUS_MAXELL_BATTERY_STATUS        0x16    // battery status register including alarms
 * #define BATTMONITOR_SMBUS_MAXELL_BATTERY_CYCLE_COUNT   0x17    // cycle count
 * #define BATTMONITOR_SMBUS_MAXELL_DESIGN_VOLTAGE        0x19    // design voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_SPECIFICATION_INFO    0x1a    // specification info
 * #define BATTMONITOR_SMBUS_MAXELL_MANUFACTURE_NAME      0x1b    // manufacturer name
 * #define BATTMONITOR_SMBUS_MAXELL_SERIALNUM             0x1c    // serial number register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE6         0x3a    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE5         0x3b    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE4         0x3c    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE3         0x3d    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE2         0x3e    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_CELL_VOLTAGE1         0x3f    // cell voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_HEALTH_STATUS         0x4f    // state of health
 * #define BATTMONITOR_SMBUS_MAXELL_SAFETY_ALERT          0x50    // safety alert
 * #define BATTMONITOR_SMBUS_MAXELL_SAFETY_STATUS         0x50    // safety status
 * #define BATTMONITOR_SMBUS_MAXELL_PF_ALERT              0x52    // safety status
 * #define BATTMONITOR_SMBUS_MAXELL_PF_STATUS             0x53    // safety status
*/

// Constructor
AP_BattMonitor_SMBus_Maxell::AP_BattMonitor_SMBus_Maxell(AP_BattMonitor &mon, uint8_t instance,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, instance, mon_state)
    , _dev(std::move(dev))
{
    _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SMBus_Maxell::timer, void));
}

/// Read the battery voltage and current.  Should be called at 10hz
void AP_BattMonitor_SMBus_Maxell::read()
{
    // nothing to do - all done in timer()
}

void AP_BattMonitor_SMBus_Maxell::timer()
{
    uint16_t data;
    uint32_t tnow = AP_HAL::micros();

    // read voltage (V)
    if (read_word(BATTMONITOR_SMBUS_MAXELL_VOLTAGE, data)) {
        _state.voltage = (float)data / 1000.0f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
        return;
    }

    // read current (A)
    if (read_word(BATTMONITOR_SMBUS_MAXELL_CURRENT, data)) {
        _state.current_amps = -(float)((int16_t)data) / 1000.0f;
        _state.last_time_micros = tnow;
    }
}

// read word from register
// returns true if read was successful, false if failed
bool AP_BattMonitor_SMBus_Maxell::read_word(uint8_t reg, uint16_t& data) const
{
    uint8_t buff[2];    // buffer to hold results

    // read three bytes and place in last three bytes of buffer
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return false;
    }

    // convert buffer to word
    data = (uint16_t)buff[1]<<8 | (uint16_t)buff[0];

    // return success
    return true;
}

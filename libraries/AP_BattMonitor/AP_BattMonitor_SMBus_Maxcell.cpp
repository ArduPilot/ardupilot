#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_Maxcell.h"
#include <utility>

extern const AP_HAL::HAL& hal;

#include <AP_HAL/AP_HAL.h>

#define BATTMONITOR_SMBUS_TEMP      0x08    // temperature register
#define BATTMONITOR_SMBUS_VOLTAGE   0x09    // voltage register
#define BATTMONITOR_SMBUS_CURRENT               0x0a    // current register
#define BATTMONITOR_SMBUS_CHARGE_STATUS         0x0d    // relative state of charge
#define BATTMONITOR_SMBUS_BATTERY_STATUS        0x16    // battery status register including alarms
#define BATTMONITOR_SMBUS_BATTERY_CYCLE_COUNT   0x17    // cycle count
#define BATTMONITOR_SMBUS_DESIGN_VOLTAGE        0x19    // design voltage register
#define BATTMONITOR_SMBUS_SPECIFICATION_INFO    0x1a    // specification info
#define BATTMONITOR_SMBUS_MANUFACTURE_NAME      0x1b    // manufacturer name
#define BATTMONITOR_SMBUS_SERIALNUM             0x1c    // serial number register
#define BATTMONITOR_SMBUS_CELL_VOLTAGE6         0x3a    // cell voltage register
#define BATTMONITOR_SMBUS_CELL_VOLTAGE5         0x3b    // cell voltage register
#define BATTMONITOR_SMBUS_CELL_VOLTAGE4         0x3c    // cell voltage register
#define BATTMONITOR_SMBUS_CELL_VOLTAGE3         0x3d    // cell voltage register
#define BATTMONITOR_SMBUS_CELL_VOLTAGE2         0x3e    // cell voltage register
#define BATTMONITOR_SMBUS_CELL_VOLTAGE1         0x3f    // cell voltage register
#define BATTMONITOR_SMBUS_HEALTH_STATUS         0x4f    // state of health
#define BATTMONITOR_SMBUS_SAFETY_ALERT          0x50    // safety alert
#define BATTMONITOR_SMBUS_SAFETY_STATUS         0x50    // safety status
#define BATTMONITOR_SMBUS_PF_ALERT              0x52    // safety status
#define BATTMONITOR_SMBUS_PF_STATUS             0x53    // safety status

// Constructor
AP_BattMonitor_SMBus_Maxcell::AP_BattMonitor_SMBus_Maxcell(AP_BattMonitor &mon, uint8_t instance,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus(mon, instance, mon_state)
    , _dev(std::move(dev))
{
    _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_SMBus_Maxcell::timer, void));
}

/// Read the battery voltage and current.  Should be called at 10hz
void AP_BattMonitor_SMBus_Maxcell::read()
{
    // nothing to do - all done in timer()
}

void AP_BattMonitor_SMBus_Maxcell::timer()
{
    uint16_t data;
    uint32_t tnow = AP_HAL::micros();

//    // read temperature (K)
//    if (read_word(BATTMONITOR_SMBUS_TEMP, data, 2)) {
//    	float temperature = (float)data / 10.0f;
//    }
//
//    // read relative state of charge (0-100%)
//    if (read_word(BATTMONITOR_SMBUS_CHARGE_STATUS, data, 1)) {
//    	uint8_t capacity_remaining = data;
//    }

    // read voltage (V)
    if (read_word(BATTMONITOR_SMBUS_VOLTAGE, data, 2)) {
        _state.voltage = (float)data / 1000.0f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // read current (A)
    if (read_word(BATTMONITOR_SMBUS_CURRENT, data, 2)) {
        _state.current_amps = (float)((int16_t)data) / 1000.0f;
        _state.last_time_micros = tnow;
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
    }
}

// read word from register
// returns true if read was successful, false if failed
bool AP_BattMonitor_SMBus_Maxcell::read_word(uint8_t reg, uint16_t& data, uint8_t size) const
{
    uint8_t buff[size];    // buffer to hold results

    // read three bytes and place in last three bytes of buffer
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return false;
    }

    // convert buffer to word
    data = (uint16_t)buff[0];
    for (int i = 1; i < size; i++) {
        data |= (uint16_t)buff[i]<<(8*i);
    }

    // return success
    return true;
}

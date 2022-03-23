#include "AP_BattMonitor_LTC2946.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>

#if HAL_BATTMON_LTC2946_ENABLED

extern const AP_HAL::HAL& hal;

#define REG_CTRLA 0x00
#define REG_CTRLB 0x01
#define REG_STATUS 0x80
#define REG_MFR_ID 0xe7

// first byte of 16 bit ID is stable
#define ID_LTC2946 0x60

#define REG_DELTA 0x14 // 16 bits
#define REG_VIN   0x1e // 16 bits

#define REGA_CONF 0x18 // sense, alternate
#define REGB_CONF 0x01 // auto-reset

void AP_BattMonitor_LTC2946::init(void)
{
    dev = hal.i2c_mgr->get_device(HAL_BATTMON_LTC2946_BUS, HAL_BATTMON_LTC2946_ADDR, 100000, false, 20);
    if (!dev) {
        return;
    }

    uint8_t id = 0;
    WITH_SEMAPHORE(dev->get_semaphore());
    if (!dev->read_registers(REG_MFR_ID, &id, 1) || id != ID_LTC2946) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTC2946: Failed to find device 0x%04x", unsigned(id));
        return;
    }

    if (!dev->write_register(REG_CTRLA, REGA_CONF) ||
        !dev->write_register(REG_CTRLB, REGB_CONF)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTC2946: Failed to configure device");
        return;
    }

    // use datasheet typical values
    voltage_LSB = 102.4 / 4095.0;
    current_LSB = (0.1024/0.0005) / 4095.0;

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "LTC2946: found monitor on bus %u", HAL_BATTMON_LTC2946_BUS);

    if (dev) {
        dev->register_periodic_callback(25000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_LTC2946::timer, void));
    }
}

/// read the battery_voltage and current, should be called at 10hz
void AP_BattMonitor_LTC2946::read(void)
{
    WITH_SEMAPHORE(accumulate.sem);
    _state.healthy = accumulate.count > 0;
    if (!_state.healthy) {
        return;
    }

    _state.voltage = accumulate.volt_sum / accumulate.count;
    _state.current_amps = accumulate.current_sum / accumulate.count;
    accumulate.volt_sum = 0;
    accumulate.current_sum = 0;
    accumulate.count = 0;

    const uint32_t tnow = AP_HAL::micros();
    const uint32_t dt_us = tnow - _state.last_time_micros;
    
    // update total current drawn since startup
    update_consumed(_state, dt_us);

    _state.last_time_micros = tnow;
}

/*
 read word from register
 returns true if read was successful, false if failed
*/
bool AP_BattMonitor_LTC2946::read_word(const uint8_t reg, uint16_t& data) const
{
    // read the appropriate register from the device
    if (!dev->read_registers(reg, (uint8_t *)&data, sizeof(data))) {
        return false;
    }

    // convert byte order
    data = uint16_t(be16toh(uint16_t(data)));

    // return success
    return true;
}

void AP_BattMonitor_LTC2946::timer(void)
{
    uint16_t amps, volts;
    if (!read_word(REG_DELTA, amps) ||
        !read_word(REG_VIN, volts)) {
        return;
    }
    WITH_SEMAPHORE(accumulate.sem);
    // convert 12 bit ADC data
    accumulate.volt_sum += (volts>>4) * voltage_LSB;
    accumulate.current_sum += (amps>>4) * current_LSB;
    accumulate.count++;
}

#endif // HAL_BATTMON_LTC2946_ENABLED

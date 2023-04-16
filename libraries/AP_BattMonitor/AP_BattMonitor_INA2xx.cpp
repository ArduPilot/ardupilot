#include "AP_BattMonitor_config.h"

#if AP_BATTERY_INA2XX_ENABLED

#include <AP_HAL/utility/sparse-endian.h>

#include "AP_BattMonitor_INA2xx.h"

extern const AP_HAL::HAL& hal;

#define REG_CONFIG        0x00
#define REG_SHUNT_VOLTAGE 0x01
#define REG_BUS_VOLTAGE   0x02
#define REG_CURRENT       0x04
#define REG_CALIBRATION   0x05

#define REG_CONFIG_DEFAULT 0x4127
#define REG_CONFIG_RESET   0x8000

// this should become a parameter in future
#define MAX_AMPS 90.0

#ifndef HAL_BATTMON_INA2XX_BUS
#define HAL_BATTMON_INA2XX_BUS 0
#endif
#ifndef HAL_BATTMON_INA2XX_ADDR
#define HAL_BATTMON_INA2XX_ADDR 0
#endif

const AP_Param::GroupInfo AP_BattMonitor_INA2XX::var_info[] = {

    // @Param: I2C_BUS
    // @DisplayName: Battery monitor I2C bus number
    // @Description: Battery monitor I2C bus number
    // @Range: 0 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_BUS", 25, AP_BattMonitor_INA2XX, i2c_bus, HAL_BATTMON_INA2XX_BUS),

    // @Param: I2C_ADDR
    // @DisplayName: Battery monitor I2C address
    // @Description: Battery monitor I2C address
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_ADDR", 26, AP_BattMonitor_INA2XX, i2c_address, HAL_BATTMON_INA2XX_ADDR),

    AP_GROUPEND
};

AP_BattMonitor_INA2XX::AP_BattMonitor_INA2XX(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params)
        : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_BattMonitor_INA2XX::init(void)
{
    dev = hal.i2c_mgr->get_device(i2c_bus, i2c_address, 100000, false, 20);
    if (!dev) {
        return;
    }
    // register now and configure in the timer callbacks
    dev->register_periodic_callback(25000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_INA2XX::timer, void));
}

void AP_BattMonitor_INA2XX::configure(void)
{
    WITH_SEMAPHORE(dev->get_semaphore());

    int16_t config = 0;
    if (!write_word(REG_CONFIG, REG_CONFIG_RESET) ||
        !write_word(REG_CONFIG, REG_CONFIG_DEFAULT) ||
        !read_word(REG_CONFIG, config) ||
        config != REG_CONFIG_DEFAULT) {
        return;
    }

    // configure for MAX_AMPS
    const uint16_t conf = (0x2<<9) | (0x5<<6) | (0x5<<3) | 0x7; // 2ms conv time, 16x sampling
    const float rShunt = 0.0005;
    current_LSB = MAX_AMPS / 32768.0;
    voltage_LSB = 0.00125; // 1.25mV/bit
    const uint16_t cal = uint16_t(0.00512 / (current_LSB * rShunt));
    if (!write_word(REG_CONFIG, REG_CONFIG_RESET) || // reset
        !write_word(REG_CONFIG, conf) ||
        !write_word(REG_CALIBRATION, cal)) {
        return;
    }

    configured = true;
}

/// read the battery_voltage and current, should be called at 10hz
void AP_BattMonitor_INA2XX::read(void)
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
bool AP_BattMonitor_INA2XX::read_word(const uint8_t reg, int16_t& data) const
{
    // read the appropriate register from the device
    if (!dev->read_registers(reg, (uint8_t *)&data, sizeof(data))) {
        return false;
    }

    // convert byte order
    data = int16_t(be16toh(uint16_t(data)));

    return true;
}

/*
  write word to a register, byte swapped
  returns true if write was successful, false if failed
*/
bool AP_BattMonitor_INA2XX::write_word(const uint8_t reg, const uint16_t data) const
{
    const uint8_t b[3] { reg, uint8_t(data >> 8), uint8_t(data&0xff) };
    return dev->transfer(b, sizeof(b), nullptr, 0);
}

void AP_BattMonitor_INA2XX::timer(void)
{
    // allow for power-on after boot
    if (!configured) {
        uint32_t now = AP_HAL::millis();
        if (now - last_configure_ms > 200) {
            // try contacting the device at 5Hz
            last_configure_ms = now;
            configure();
        }
        if (!configured) {
            // waiting for the device to respond
            return;
        }
    }

    int16_t bus_voltage, current;

    if (!read_word(REG_BUS_VOLTAGE, bus_voltage) ||
        !read_word(REG_CURRENT, current)) {
        failed_reads++;
        if (failed_reads > 10) {
            // device has disconnected, we need to reconfigure it
            configured = false;
        }
        return;
    }
    failed_reads = 0;

    WITH_SEMAPHORE(accumulate.sem);
    accumulate.volt_sum += bus_voltage * voltage_LSB;
    accumulate.current_sum += current * current_LSB;
    accumulate.count++;
}

#endif // AP_BATTERY_INA2XX_ENABLED

#include "AP_BattMonitor_INA231.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>

#if HAL_BATTMON_INA231_ENABLED

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

void AP_BattMonitor_INA231::init(void)
{
    dev = hal.i2c_mgr->get_device(HAL_BATTMON_INA231_BUS, HAL_BATTMON_INA231_ADDR, 100000, false, 20);
    if (!dev) {
        return;
    }

    int16_t config = 0;
    WITH_SEMAPHORE(dev->get_semaphore());
    if (!write_word(REG_CONFIG, REG_CONFIG_RESET) ||
        !write_word(REG_CONFIG, REG_CONFIG_DEFAULT) ||
        !read_word(REG_CONFIG, config) ||
        config != REG_CONFIG_DEFAULT) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INA231: Failed to find device 0x%04x", unsigned(config));
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
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INA231: Failed to configure device");
        return;
    }


    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "INA231: found monitor on bus %u", HAL_BATTMON_INA231_BUS);

    if (dev) {
        dev->register_periodic_callback(25000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_INA231::timer, void));
    }
}

/// read the battery_voltage and current, should be called at 10hz
void AP_BattMonitor_INA231::read(void)
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
    const float dt = tnow - _state.last_time_micros;
    
    // update total current drawn since startup
    if (_state.last_time_micros != 0 && dt < 2000000.0) {
        // .0002778 is 1/3600 (conversion to hours)
        const float mah = _state.current_amps * dt * 0.0000002778;
        _state.consumed_mah += mah;
        _state.consumed_wh  += 0.001 * mah * _state.voltage;
    }
    _state.last_time_micros = tnow;
}

/*
 read word from register
 returns true if read was successful, false if failed
*/
bool AP_BattMonitor_INA231::read_word(const uint8_t reg, int16_t& data) const
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
bool AP_BattMonitor_INA231::write_word(const uint8_t reg, const uint16_t data) const
{
    const uint8_t b[3] { reg, uint8_t(data >> 8), uint8_t(data&0xff) };
    return dev->transfer(b, sizeof(b), nullptr, 0);
}

void AP_BattMonitor_INA231::timer(void)
{
    int16_t bus_voltage, current;
    if (!read_word(REG_BUS_VOLTAGE, bus_voltage) ||
        !read_word(REG_CURRENT, current)) {
        return;
    }
    WITH_SEMAPHORE(accumulate.sem);
    accumulate.volt_sum += bus_voltage * voltage_LSB;
    accumulate.current_sum += current * current_LSB;
    accumulate.count++;
}

#endif // HAL_BATTMON_INA231_ENABLED

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_INA239_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>

#include "AP_BattMonitor_INA239.h"

extern const AP_HAL::HAL& hal;

/*
  note that registers are clocked on SPI MSB first, with register
  number in top 6 bits, LSB is read flag
 */
#define REG_CONFIG        0x00
#define REG_ADC_CONFIG    0x01
#define REG_SHUNT_CAL     0x02
#define REG_SHUNT_VOLTAGE 0x04
#define REG_BUS_VOLTAGE   0x05
#define REG_CURRENT       0x07

#define REG_ADC_CONFIG_RESET 0xFB68U

#ifndef HAL_BATTMON_INA239_SHUNT_RESISTANCE
#define HAL_BATTMON_INA239_SHUNT_RESISTANCE 0.0002
#endif

#ifndef HAL_BATTMON_INA239_MAX_CURRENT
#define HAL_BATTMON_INA239_MAX_CURRENT 90
#endif

const AP_Param::GroupInfo AP_BattMonitor_INA239::var_info[] = {

    // @Param: MAX_AMPS
    // @DisplayName: Battery monitor max current
    // @Description: This controls the maximum current the INA239 sensor will work with.
    // @Range: 1 400
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("MAX_AMPS", 27, AP_BattMonitor_INA239, max_amps, HAL_BATTMON_INA239_MAX_CURRENT),

    // @Param: SHUNT
    // @DisplayName: Battery monitor shunt resistor
    // @Description: This sets the shunt resistor used in the device
    // @Range: 0.0001 0.01
    // @Units: Ohm
    // @User: Advanced
    AP_GROUPINFO("SHUNT", 28, AP_BattMonitor_INA239, rShunt, HAL_BATTMON_INA239_SHUNT_RESISTANCE),
    
    AP_GROUPEND
};

AP_BattMonitor_INA239::AP_BattMonitor_INA239(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params)
        : AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_BattMonitor_INA239::init(void)
{
    dev = hal.spi->get_device(AP_BATTERY_INA239_SPI_DEVICE);
    if (!dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "device fail");
        return;
    }
    dev->set_read_flag(0x01);
    // register now and configure in the timer callbacks
    dev->register_periodic_callback(25000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_INA239::timer, void));
}

void AP_BattMonitor_INA239::configure(void)
{
    WITH_SEMAPHORE(dev->get_semaphore());

    int16_t adc_config = 0;

    if (!read_word(REG_ADC_CONFIG, adc_config) ||
        uint16_t(adc_config) != REG_ADC_CONFIG_RESET) {
        return;
    }

    voltage_LSB = 3.125e-3;
    current_LSB = max_amps.get() / 0x8000;
    const int16_t shunt_cal = 819.2 * 1e6 * current_LSB * rShunt.get();
    int16_t shunt_cal2 = 0;

    if (!write_word(REG_SHUNT_CAL, shunt_cal) ||
        !read_word(REG_SHUNT_CAL, shunt_cal2) ||
        shunt_cal != shunt_cal2) {
        return;
    }

    configured = true;
}

/// read the battery_voltage and current, should be called at 10hz
void AP_BattMonitor_INA239::read(void)
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
bool AP_BattMonitor_INA239::read_word(const uint8_t reg, int16_t& data) const
{
    // read the appropriate register from the device
    if (!dev->read_registers(reg<<2, (uint8_t *)&data, sizeof(data))) {
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
bool AP_BattMonitor_INA239::write_word(const uint8_t reg, const uint16_t data) const
{
    const uint8_t b[3] { uint8_t(reg<<2), uint8_t(data >> 8), uint8_t(data&0xff) };
    return dev->transfer(b, sizeof(b), nullptr, 0);
}

void AP_BattMonitor_INA239::timer(void)
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

#endif // AP_BATTERY_INA239_ENABLED

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_INA2XX_ENABLED

/*
  supports INA226, INA228 and INA238 I2C battery monitors
 */

#include <AP_HAL/utility/sparse-endian.h>

#include "AP_BattMonitor_INA2xx.h"

extern const AP_HAL::HAL& hal;


// INA226 specific registers
#define REG_226_CONFIG        0x00
#define  REG_226_CONFIG_DEFAULT 0x4127
#define  REG_226_CONFIG_RESET   0x8000
#define REG_226_BUS_VOLTAGE   0x02
#define REG_226_CURRENT       0x04
#define REG_226_CALIBRATION   0x05
#define REG_226_MANUFACT_ID   0xfe

// INA228 specific registers
#define REG_228_CONFIG        0x00
#define  REG_228_CONFIG_RESET   0x8000
#define REG_228_ADC_CONFIG    0x01
#define REG_228_SHUNT_CAL     0x02
#define REG_228_VBUS          0x05
#define REG_228_CURRENT       0x07
#define REG_228_MANUFACT_ID   0x3e
#define REG_228_DEVICE_ID     0x3f
#define REG_228_DIETEMP       0x06
#define INA_228_TEMP_C_LSB    7.8125e-3

// INA237/INA238 specific registers
#define REG_238_CONFIG        0x00
#define  REG_238_CONFIG_RESET   0x8000
#define REG_238_ADC_CONFIG    0x01
#define REG_238_SHUNT_CAL     0x02
#define REG_238_VBUS          0x05
#define REG_238_CURRENT       0x07
#define REG_238_MANUFACT_ID   0x3e
#define REG_238_DEVICE_ID     0x3f
#define REG_238_DIETEMP       0x06
#define INA_238_TEMP_C_LSB    7.8125e-3 // need to mask bottom 4 bits

// INA231 specific registers
#define REG_231_CONFIG        0x00
#define REG_231_SHUNT_VOLTAGE 0x01
#define REG_231_BUS_VOLTAGE   0x02
#define REG_231_POWER         0x03
#define REG_231_CURRENT       0x04
#define REG_231_CALIBRATION   0x05
#define REG_231_MASK          0x06
#define REG_231_ALERT         0x07

// INA260 specific registers
#define REG_260_CONFIG        0x00
#define REG_260_CURRENT       0x01
#define REG_260_BUS_VOLTAGE   0x02
#define REG_260_POWER         0x03
#define REG_260_MASK          0x06
#define REG_260_ALERT         0x07
#define REG_260_MANUFACT_ID   0xfe
#define REG_260_DIE_ID        0xff

#ifndef DEFAULT_BATTMON_INA2XX_MAX_AMPS
#define DEFAULT_BATTMON_INA2XX_MAX_AMPS 90.0
#endif

#ifndef DEFAULT_BATTMON_INA2XX_SHUNT
#define DEFAULT_BATTMON_INA2XX_SHUNT 0.0005
#endif

#ifndef HAL_BATTMON_INA2XX_BUS
#define HAL_BATTMON_INA2XX_BUS 0
#endif
#ifndef HAL_BATTMON_INA2XX_ADDR
#define HAL_BATTMON_INA2XX_ADDR 0
#endif

// list of addresses to probe if I2C_ADDR is zero
const uint8_t AP_BattMonitor_INA2XX::i2c_probe_addresses[] { 0x41, 0x44, 0x45 };

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
    // @Description: Battery monitor I2C address. If this is zero then probe list of supported addresses
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_ADDR", 26, AP_BattMonitor_INA2XX, i2c_address, HAL_BATTMON_INA2XX_ADDR),

    // @Param: MAX_AMPS
    // @DisplayName: Battery monitor max current
    // @Description: This controls the maximum current the INS2XX sensor will work with.
    // @Range: 1 400
    // @Units: A
    // @User: Advanced
    AP_GROUPINFO("MAX_AMPS", 27, AP_BattMonitor_INA2XX, max_amps, DEFAULT_BATTMON_INA2XX_MAX_AMPS),

    // @Param: SHUNT
    // @DisplayName: Battery monitor shunt resistor
    // @Description: This sets the shunt resistor used in the device
    // @Range: 0.0001 0.01
    // @Units: Ohm
    // @User: Advanced
    AP_GROUPINFO("SHUNT", 28, AP_BattMonitor_INA2XX, rShunt, DEFAULT_BATTMON_INA2XX_SHUNT),

    // CHECK/UPDATE INDEX TABLE IN AP_BattMonitor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

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
    dev = hal.i2c_mgr->get_device_ptr(i2c_bus, i2c_address, 100000, false, 20);
    if (!dev) {
        return;
    }
    // register now and configure in the timer callbacks
    dev->register_periodic_callback(25000, FUNCTOR_BIND_MEMBER(&AP_BattMonitor_INA2XX::timer, void));
}

bool AP_BattMonitor_INA2XX::configure(DevType dtype)
{
    switch (dtype) {
    case DevType::UNKNOWN:
        return false;

    case DevType::INA226: {
        // configure for MAX_AMPS
        const uint16_t conf = (0x2<<9) | (0x5<<6) | (0x5<<3) | 0x7; // 2ms conv time, 16x sampling
        current_LSB = max_amps / 32768.0;
        voltage_LSB = 0.00125; // 1.25mV/bit
        const uint16_t cal = uint16_t(0.00512 / (current_LSB * rShunt));
        if (write_word(REG_226_CONFIG, REG_226_CONFIG_RESET) && // reset
            write_word(REG_226_CONFIG, conf) &&
            write_word(REG_226_CALIBRATION, cal)) {
            dev_type = dtype;
            return true;
        }
        break;
    }

    case DevType::INA228: {
        // configure for MAX_AMPS
        voltage_LSB = 195.3125e-6; // 195.3125 uV/LSB
        current_LSB = max_amps / (1U<<19);
        const uint16_t shunt_cal = uint16_t(13107.2e6 * current_LSB * rShunt) & 0x7FFF;
        if (write_word(REG_228_CONFIG, REG_228_CONFIG_RESET) && // reset
            write_word(REG_228_CONFIG, 0) &&
            write_word(REG_228_SHUNT_CAL, shunt_cal)) {
            dev_type = dtype;
            return true;
        }
        break;
    }

    case DevType::INA238: {
        // configure for MAX_AMPS
        voltage_LSB = 3.125e-3; // 3.125mV/LSB
        current_LSB = max_amps / (1U<<15);
        const uint16_t shunt_cal = uint16_t(819.2e6 * current_LSB * rShunt) & 0x7FFF;
        if (write_word(REG_238_CONFIG, REG_238_CONFIG_RESET) && // reset
            write_word(REG_238_CONFIG, 0) &&
            write_word(REG_238_SHUNT_CAL, shunt_cal)) {
            dev_type = dtype;
            return true;
        }
        break;
    }

    case DevType::INA231: {
        // no configuration needed
        voltage_LSB = 1.25e-3;
        current_LSB = max_amps / (1U<<15);
        const uint16_t cal = 0.00512 / (current_LSB * rShunt);
        if (write_word(REG_231_CALIBRATION, cal)) {
            dev_type = dtype;
            return true;
        }
        break;
    }

    case DevType::INA260: {
        // Reset device
        if (!write_word(REG_260_CONFIG, 0x8000)) {
            return false;
        }

        // Set longest conversion time and no averaging
        // This is 8.244ms for both voltage and current
        // So both should have new readings after 16.488ms
        // This is a new reading at 60Hz, we read at 40Hz
        // Continuous voltage and current measurement
        if (!write_word(REG_260_CONFIG, 0x01ff)) {
            return false;
        }

        // Configuration OK
        // Set (not save) unused shunt and max amps parameters to reflect specs
        // 2 milliohms internal shunt
        rShunt.set_and_default(0.002);

        // Rated for 30 amps for 5 seconds
        max_amps.set_and_default(30.0);

        // Set detected type
        dev_type = dtype;
        return true;
    }

    }
    return false;
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
  read 16 bit word from register
  returns true if read was successful, false if failed
*/
bool AP_BattMonitor_INA2XX::read_word16(const uint8_t reg, int16_t& data) const
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
  read 24 bit signed value from register
  returns true if read was successful, false if failed
*/
bool AP_BattMonitor_INA2XX::read_word24(const uint8_t reg, int32_t& data) const
{
    // read the appropriate register from the device
    uint8_t d[3];
    if (!dev->read_registers(reg, d, sizeof(d))) {
        return false;
    }
    // 24 bit 2s complement data. Shift into upper 24 bits of int32_t then divide by 256
    // to cope with negative numbers properly
    data = d[0]<<24 | d[1]<<16 | d[2] << 8;
    data = data / 256;

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

/*
  detect device type. This may happen well after power on if battery is
  not plugged in yet
*/
bool AP_BattMonitor_INA2XX::detect_device(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_detect_ms < 200) {
        // don't flood the bus
        return false;
    }
    last_detect_ms = now;
    int16_t id;

    WITH_SEMAPHORE(dev->get_semaphore());

    if (i2c_address.get() == 0) {
        // Cycle through probe address list
        dev->set_address(i2c_probe_addresses[i2c_probe_next]);
        i2c_probe_next = (i2c_probe_next+1) % sizeof(i2c_probe_addresses);

    } else {
        // User provided address
        dev->set_address(i2c_address.get());
    }

    if (read_word16(REG_228_MANUFACT_ID, id) && id == 0x5449 &&
        read_word16(REG_228_DEVICE_ID, id) && (id&0xFFF0) == 0x2280) {
        has_temp = true;
        return configure(DevType::INA228);
    }
    if (read_word16(REG_238_MANUFACT_ID, id) && id == 0x5449 &&
        read_word16(REG_238_DEVICE_ID, id) && (id&0xFFF0) == 0x2380) {
        has_temp = true;
        return configure(DevType::INA238);
    }
    if (read_word16(REG_260_MANUFACT_ID, id) && id == 0x5449 &&
        read_word16(REG_260_DIE_ID, id) && (id&0xFFF0) == 0x2270) {
        return configure(DevType::INA260);
    }
    if (read_word16(REG_226_MANUFACT_ID, id) && id == 0x5449 &&
        write_word(REG_226_CONFIG, REG_226_CONFIG_RESET) &&
        write_word(REG_226_CONFIG, REG_226_CONFIG_DEFAULT) &&
        read_word16(REG_226_CONFIG, id) &&
        id == REG_226_CONFIG_DEFAULT) {
        return configure(DevType::INA226);
    }
    if (read_word16(REG_231_CONFIG, id) && id == 0x4127) {
        // no manufacturer ID for 231
        return configure(DevType::INA231);
    }

    return false;
}


void AP_BattMonitor_INA2XX::timer(void)
{
    if (failed_reads > 10) {
        // device has disconnected, we need to reconfigure it
        dev_type = DevType::UNKNOWN;
    }

    if (dev_type == DevType::UNKNOWN) {
        if (!detect_device()) {
            return;
        }

        // Reset failed reads after successful detection
        failed_reads = 0;
    }

    float voltage = 0, current = 0;

    switch (dev_type) {
    case DevType::UNKNOWN:
        return;

    case DevType::INA226: {
        int16_t bus_voltage16, current16;
        if (!read_word16(REG_226_BUS_VOLTAGE, bus_voltage16) ||
            !read_word16(REG_226_CURRENT, current16)) {
            failed_reads++;
            return;
        }
        voltage = bus_voltage16 * voltage_LSB;
        current = current16 * current_LSB;
        break;
    }

    case DevType::INA228: {
        int32_t bus_voltage24, current24;
        int16_t temp16;
        if (!read_word24(REG_228_VBUS, bus_voltage24) ||
            !read_word24(REG_228_CURRENT, current24) ||
            !read_word16(REG_228_DIETEMP, temp16)) {
            failed_reads++;
            return;
        }
        voltage = (bus_voltage24>>4) * voltage_LSB;
        current = (current24>>4) * current_LSB;
        temperature = temp16 * INA_228_TEMP_C_LSB;
        break;
    }

    case DevType::INA238: {
        int16_t bus_voltage16, current16, temp16;
        if (!read_word16(REG_238_VBUS, bus_voltage16) ||
            !read_word16(REG_238_CURRENT, current16) ||
            !read_word16(REG_238_DIETEMP, temp16)) {
            failed_reads++;
            return;
        }
        voltage = bus_voltage16 * voltage_LSB;
        current = current16 * current_LSB;
        temperature = (temp16&0xFFF0) * INA_238_TEMP_C_LSB;
        break;
    }

    case DevType::INA231: {
        int16_t bus_voltage16, current16;
        if (!read_word16(REG_231_SHUNT_VOLTAGE, bus_voltage16) ||
            !read_word16(REG_231_CURRENT, current16)) {
            failed_reads++;
            return;
        }
        voltage = bus_voltage16 * voltage_LSB;
        current = current16 * current_LSB;
        break;
    }

    case DevType::INA260: {
        int16_t current16, bus_voltage16;
        if (!read_word16(REG_260_BUS_VOLTAGE, bus_voltage16) ||
            !read_word16(REG_260_CURRENT, current16)) {
            failed_reads++;
            return;
        }
        voltage = bus_voltage16 * 0.00125;
        current = current16 * 0.00125;
        break;
    }
    }

    failed_reads = 0;

    WITH_SEMAPHORE(accumulate.sem);
    accumulate.volt_sum += voltage;
    accumulate.current_sum += current;
    accumulate.count++;
}

/*
  get last temperature
 */
bool AP_BattMonitor_INA2XX::get_temperature(float &temp) const
{
    temp = temperature;
    return has_temp;
}

#endif // AP_BATTERY_INA2XX_ENABLED

/*
   TMP117 High-Accuracy Digital Temperature Sensor driver

   TMP117 datasheet: https://www.ti.com/lit/ds/snosd82d/snosd82d.pdf

   - I2C interface, up to 400 kHz (Fast Mode)
   - 16-bit signed result, 7.8125 m°C/LSB resolution
   - Default: continuous conversion, 8 averages, 1s cycle
   - Device ID register 0x0F reads 0x0117
*/

#include "AP_TemperatureSensor_config.h"

#if AP_TEMPERATURE_SENSOR_TMP117_ENABLED

#include "AP_TemperatureSensor_TMP117.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#define TMP117_I2C_ADDR_DEFAULT     0x48    // ADD0 = GND

#define TMP117_REG_TEMP_RESULT      0x00    // 16-bit temperature result (read-only)
#define TMP117_REG_CONFIGURATION    0x01    // 16-bit configuration register
#define TMP117_REG_DEVICE_ID        0x0F    // 16-bit device ID register

#define TMP117_DEVICE_ID            0x0117  // expected device ID value
#define TMP117_TEMP_RESOLUTION      0.0078125f  // °C per LSB
#define TMP117_TEMP_RESET_VALUE     0x8000  // -256°C, indicates no conversion yet

void AP_TemperatureSensor_TMP117::init()
{
    _params.bus_address.set_default(TMP117_I2C_ADDR_DEFAULT);

    _dev = hal.i2c_mgr->get_device_ptr(_params.bus, _params.bus_address);
    if (!_dev) {
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    // verify device ID
    uint8_t cmd = TMP117_REG_DEVICE_ID;
    uint8_t val[2];
    if (!_dev->transfer(&cmd, 1, val, 2)) {
        return;
    }
    const uint16_t dev_id = (uint16_t(val[0]) << 8) | val[1];
    if ((dev_id & 0x0FFF) != (TMP117_DEVICE_ID & 0x0FFF)) {
        // DID[11:0] must match 0x117, upper nibble is revision
        return;
    }

    // factory defaults are fine: CC mode, 8 averages, 1s cycle (config = 0x0220)
    // no configuration write needed

    _dev->register_periodic_callback(50 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_TMP117::_timer, void));
}

void AP_TemperatureSensor_TMP117::_timer()
{
    uint8_t cmd = TMP117_REG_TEMP_RESULT;
    uint8_t val[2];

    if (!_dev->transfer(&cmd, 1, val, 2)) {
        return;
    }

    const uint16_t raw = (uint16_t(val[0]) << 8) | val[1];

    // 0x8000 = reset value (-256°C), means no conversion has completed yet
    if (raw == TMP117_TEMP_RESET_VALUE) {
        return;
    }

    // signed 16-bit two's complement, 7.8125 m°C per LSB
    const float temp_deg = int16_t(raw) * TMP117_TEMP_RESOLUTION;
    set_temperature(temp_deg);
}

#endif  // AP_TEMPERATURE_SENSOR_TMP117_ENABLED

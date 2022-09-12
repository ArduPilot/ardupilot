#include "AP_TemperatureSensor_TSYS01.h"

#if AP_TEMPERATURE_SENSOR_TSYS01_ENABLE
#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

static const uint8_t TSYS01_CMD_RESET       = 0x1E;
static const uint8_t TSYS01_CMD_READ_PROM   = 0xA0;
static const uint8_t TSYS01_CMD_CONVERT     = 0x40;
static const uint8_t TSYS01_CMD_READ_ADC    = 0x00;

void AP_TemperatureSensor_TSYS01::init()
{
    constexpr char name[] = "TSYS01";

    // I2C Address: Default to using TSYS01_ADDR_CSB0 & Check I2C Address is Correct
    uint8_t addr = _params._i2c_address;
    if ((addr != TSYS01_ADDR_CSB0) && (addr != TSYS01_ADDR_CSB1)) {
        printf("%s, wrong I2C addr", name);
        return;
    }

    _dev = std::move(hal.i2c_mgr->get_device(_params._i2c_bus, _params._i2c_address));
    if (!_dev) {
        printf("%s device is null!", name);
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    if (!_reset()) {
        printf("%s reset failed", name);
        return;
    }

    hal.scheduler->delay(4);

    if (!_read_prom()) {
        printf("%s prom read failed", name);
        return;
    }

    _convert();

    // lower retries for run
    _dev->set_retries(3);

    /* Request 20Hz update */
    // Max conversion time is 9.04 ms
    _dev->register_periodic_callback(50 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_TSYS01::_timer, void));
}

bool AP_TemperatureSensor_TSYS01::_reset() const
{
    return _dev->transfer(&TSYS01_CMD_RESET, 1, nullptr, 0);
}

// Register map
// prom word	Address
//      0       0xA0 -> unused
//      1       0xA2 -> _k[4]
//      2       0xA4 -> _k[3]
//      3       0xA6 -> _k[2]
//      4       0xA8 -> _k[1]
//      5       0xAA -> _k[0]
//      6       0xAC -> unused
//      7       0xAE -> unused
bool AP_TemperatureSensor_TSYS01::_read_prom()
{
    bool success = false;
    for (int i = 0; i < 5; i++) {
        // Read only the prom values that we use
        _k[i] = _read_prom_word(5-i);
        success |= _k[i] != 0;
    }
    return success;
}

// Borrowed from MS Baro driver
uint16_t AP_TemperatureSensor_TSYS01::_read_prom_word(uint8_t word) const
{
    const uint8_t reg = TSYS01_CMD_READ_PROM + (word << 1);
    uint8_t val[2];
    if (!_dev->transfer(&reg, 1, val, 2)) {
        return 0;
    }
    return (val[0] << 8) | val[1];
}

bool AP_TemperatureSensor_TSYS01::_convert() const
{
    return _dev->transfer(&TSYS01_CMD_CONVERT, 1, nullptr, 0);
}

uint32_t AP_TemperatureSensor_TSYS01::_read_adc() const
{
    uint8_t val[3];
    if (!_dev->transfer(&TSYS01_CMD_READ_ADC, 1, val, 3)) {
        return 0;
    }
    return (val[0] << 16) | (val[1] << 8) | val[2];
}

void AP_TemperatureSensor_TSYS01::_timer(void)
{
    const uint32_t adc = _read_adc();
    _state.healthy = adc != 0;

    if (_state.healthy) {
        _calculate(adc);
    } else {
        _state.temperature = 0;
    }

    //printf("\nTemperature: %.2lf C", _state.temperature;

    _convert();
}

void AP_TemperatureSensor_TSYS01::_calculate(uint32_t adc)
{
    const float adc16 = adc/256;
    _state.temperature =
        -2   * _k[4] * powf(10, -21) * powf(adc16, 4) +
        4    * _k[3] * powf(10, -16) * powf(adc16, 3) +
        -2   * _k[2] * powf(10, -11) * powf(adc16, 2) +
        1    * _k[1] * powf(10, -6)  * adc16 +
        -1.5 * _k[0] * powf(10, -2);
}

#endif // AP_TEMPERATURE_SENSOR_TSYS01_ENABLE

#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include "AP_TemperatureSensor_HASAN.h"

extern const AP_HAL::HAL &hal;

void AP_TemperatureSensor_HASAN::init() {
    _dev = std::move(hal.i2c_mgr->get_device(1 /* bus */, HASAN_ADDR));

    if (!_dev) {
        hal.console->printf("Device is null!\n");
        return;
    }

    _dev->set_retries(10);
}

// void AP_TemperatureSensor_HASAN::update() {
//     // This could be where you read temperature and update state
//     float temperature = read_temperature();
//     // update state here with temperature
// }

float AP_TemperatureSensor_HASAN::read_temperature() {
    if (!_dev->transfer(nullptr, 0, _buffer, sizeof(_buffer))) {
        hal.console->printf("Reading temperature failed");
        return 0.0f;
    }

    // Convert 4 bytes in _buffer to temperature
    // Here, I am assuming the data is in floating point format
    float temp = 0;
    memcpy(&temp, &_buffer, sizeof(temp));
    return temp;
}

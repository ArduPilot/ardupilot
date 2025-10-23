#include "AP_ExternalSensor.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define ESP32_I2C_ADDR 0x42

void AP_ExternalSensor::read_esp32_sensor()
{
    uint8_t buf[8] = {0};

    if (!_i2c_mgr) return;

    // Reads 8 bytes from ESP32
    if (_i2c_mgr->read(ESP32_I2C_ADDR, 0, buf, sizeof(buf)) == 0) {
        float temperature = 0;
        float pitch = 0;
        memcpy(&temperature, &buf[0], 4);
        memcpy(&pitch, &buf[4], 4);

        // Send to GCS as NAMED_VALUE_FLOAT
        mavlink_msg_named_value_float_send(MAVLINK_COMM_0, AP_HAL::millis(), "TEMP_C", temperature);
        mavlink_msg_named_value_float_send(MAVLINK_COMM_0, AP_HAL::millis(), "PITCH_D", pitch);
    }
}

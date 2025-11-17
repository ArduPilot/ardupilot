#include "AP_Airspeed_PitotHoneywell.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/boards/CubeOrangePlus/board.h>

#if AP_AIRSPEED_ENABLED

extern const AP_HAL::HAL &hal;

#define HONEYWELL_I2C_ADDR 0x28
#define HONEYWELL_SAMPLES_HZ 20
#define HONEYWELL_CALLBACK_US (1000000UL / HONEYWELL_SAMPLES_HZ)
#define HONEYWELL_I2C_REG 0x00

// Constructor
AP_Airspeed_PitotHoneywell::AP_Airspeed_PitotHoneywell(AP_Airspeed &_frontend, uint8_t _instance, int8_t bus, uint8_t address)
    : AP_Airspeed_Backend(_frontend, _instance),
      _address(address)
{
    _dev = hal.i2c_mgr->get_device(bus, address);
    _last_q = 0.0f;
    _last_temperature = 0.0f;
    _healthy = false;
}

// Initialize the sensor
bool AP_Airspeed_PitotHoneywell::init()
{
    if (!_dev) {
        _healthy = false;
        return false;
    }

    // Register periodic callback at ~20 Hz
    _dev->register_periodic_callback(
        HONEYWELL_CALLBACK_US,
        FUNCTOR_BIND_MEMBER(&AP_Airspeed_PitotHoneywell::timer, void)
    );

    _healthy = true;
    return true;
}

// Timer callback: read sensor, convert to Pa
void AP_Airspeed_PitotHoneywell::timer()
{
    if (!_dev) {
        _healthy = false;
        return;
    }

    uint8_t buf[2] = {0, 0};
    if (!_dev->read_registers(HONEYWELL_I2C_REG, buf, 2)) {
        _healthy = false;
        return;
    }

    // Combine MSB and LSB
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];

    // Extract 14-bit counts (top 2 bits are status)
    uint16_t counts = (raw >> 2) & 0x3FFF;

    // Convert to differential pressure in Pa
    float readValPitot = (float)counts;
    float q_calc = (22.5f * readValPitot / 16384.0f - 10.0f - 1.25f) * 248.84f;
    float q = constrain_float(q_calc, 0.0f, 2488.4f);

    WITH_SEMAPHORE(sem) {
        _last_q = q;
        _healthy = true;
    }
}

// Return latest differential pressure
bool AP_Airspeed_PitotHoneywell::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem) {
        if (!_healthy) return false;
        pressure = _last_q;
    }
    return true;
}

// Temperature not available
bool AP_Airspeed_PitotHoneywell::get_temperature(float &temperature)
{
    temperature = 0.0f;
    return false;
}

#endif // AP_AIRSPEED_ENABLED

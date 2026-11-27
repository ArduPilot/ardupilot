#include "AP_AngleSensor_config.h"

#if AP_ANGLESENSOR_AS5600_ENABLED

// AS5600 angle sensor
// base on code by Cole Mero

#include "AS5600.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

#include <AP_HAL/utility/sparse-endian.h>

#include <GCS_MAVLink/GCS.h>

static constexpr uint8_t REG_ZMCO { 0x00 };
static constexpr uint8_t REG_ZPOS_HI { 0x01 };
static constexpr uint8_t REG_ZPOS_LO { 0x02 };
static constexpr uint8_t REG_MPOS_HI { 0x03 };
static constexpr uint8_t REG_MPOS_LO { 0x04 };
static constexpr uint8_t REG_MANG_HI { 0x05 };
static constexpr uint8_t REG_MANG_LO { 0x06 };
static constexpr uint8_t REG_CONF_HI { 0x07 };
static constexpr uint8_t REG_CONF_LO { 0x08 };
static constexpr uint8_t REG_RAW_ANG_HI { 0x0C };
static constexpr uint8_t REG_RAW_ANG_LO { 0x0D };
static constexpr uint8_t REG_ANG_HI { 0x0E };
static constexpr uint8_t REG_ANG_LO { 0x0F };
static constexpr uint8_t REG_STAT { 0x0B };
static constexpr uint8_t REG_AGC { 0x1A };
static constexpr uint8_t REG_MAG_HI { 0x1B };
static constexpr uint8_t REG_MAG_LO { 0x1C };
static constexpr uint8_t REG_BURN { 0xFF };

extern const AP_HAL::HAL &hal;

void AP_AngleSensor_AS5600::timer(void)
{
    uint16_t angle;
    {
        WITH_SEMAPHORE(dev->get_semaphore());
        dev->read_registers(REG_RAW_ANG_HI, (uint8_t*)&angle, 2);
    }
    {
        WITH_SEMAPHORE(readings.sem);
        readings.angle = be16toh(angle);
    }
}

void AP_AngleSensor_AS5600::init(void)
{
    dev = hal.i2c_mgr->get_device(bus, address);

    if (!dev) {
        return;
    }

    // insert some register reads here to try to fingerprint the device

    WITH_SEMAPHORE(dev->get_semaphore());
    dev->register_periodic_callback(25000, FUNCTOR_BIND_MEMBER(&AP_AngleSensor_AS5600::timer, void));
}

void AP_AngleSensor_AS5600::update()
{
    {
        WITH_SEMAPHORE(readings.sem);
        state.angle = readings.angle;
    }

    const uint32_t tnow = AP_HAL::millis();
    state.last_reading_ms = tnow;

    AP::logger().Write("AoAR", "TimeUS,Angle", "QH", AP_HAL::micros64(), state.angle);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "angle=%u", state.angle);
}

#endif

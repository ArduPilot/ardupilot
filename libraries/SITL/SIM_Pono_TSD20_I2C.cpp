#include "SIM_Pono_TSD20_I2C.h"
#include "SITL/SIM_Aircraft.h"
#include <AP_HAL/AP_HAL.h>

#if AP_SIM_PONO_TSD20_I2C_ENABLED

using namespace SITL;

// Register addresses matching the real sensor
static const uint8_t REG_DIST_HIGH = 0x00;
static const uint8_t REG_DIST_LOW = 0x01;
static const uint8_t REG_LASER = 0x02;
static const uint8_t REG_ID = 0x03;

static const uint8_t TSD20_ID = 0x4A;

// Limits matching the real sensor
static const uint16_t MIN_DIST_MM = 50;
static const uint16_t MAX_DIST_MM = 20000;
static const uint16_t UPDATE_RATE_HZ = 200;

void Pono_TSD20_I2C::update(const class Aircraft &aircraft)
{
    // rate limit updates to match the real sensor
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_update_ms < (1000U / UPDATE_RATE_HZ)) {
        return;
    }
    last_update_ms = now_ms;

    // only sample while the laser is on.  rangefinder_range() draws from the
    // shared SITL random stream (sonar noise), so sampling a device which is
    // not being used would shift the noise sequence of every other simulated
    // sensor on the vehicle
    if (!laser_on) {
        rangefinder_range = 0;
        return;
    }

    rangefinder_range = aircraft.rangefinder_range();
}

int Pono_TSD20_I2C::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    /*
     * single write: register selection, or a register write such as
     * enabling the laser
     */
    if (data->nmsgs == 1) {
        auto &msg = data->msgs[0];
        if (msg.flags & I2C_M_RD) {
            return -1;
        }
        reg = msg.buf[0];
        if (msg.len == 2 && reg == REG_LASER) {
            laser_on = (msg.buf[1] == 1);
        }
        return 0;
    }

    // combined register select and read, as used by read_registers()
    if ((data->nmsgs != 2) || !(data->msgs[1].flags & I2C_M_RD)) {
        return -1;
    }
    reg = data->msgs[0].buf[0];

    // the sensor only measures with the laser enabled; readings outside
    // the 50mm to 20m window are reported as 0, which the driver rejects
    uint16_t dist_mm = 0;
    if (laser_on &&
        rangefinder_range >= (MIN_DIST_MM * 0.001f) &&
        rangefinder_range <= (MAX_DIST_MM * 0.001f)) {
        dist_mm = rangefinder_range * 1000.0f;
    }

    const uint8_t count = data->msgs[1].len;
    for (uint8_t i = 0; i < count; i++) {
        const uint8_t curr_reg = reg + i;
        switch (curr_reg) {
        case REG_DIST_HIGH:
            data->msgs[1].buf[i] = (dist_mm >> 8) & 0xFF;
            break;
        case REG_DIST_LOW:
            data->msgs[1].buf[i] = dist_mm & 0xFF;
            break;
        case REG_LASER:
            data->msgs[1].buf[i] = laser_on ? 1 : 0;
            break;
        case REG_ID:
            data->msgs[1].buf[i] = TSD20_ID;
            break;
        default:
            return -1;
        }
    }

    return 0;
}

#endif  // AP_SIM_PONO_TSD20_I2C_ENABLED

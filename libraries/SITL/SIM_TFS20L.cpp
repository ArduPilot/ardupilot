#include "SIM_TFS20L.h"
#include "SITL/SIM_Aircraft.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <cmath>

#if AP_SIM_TFS20L_ENABLED

using namespace SITL;

// Register addresses matching real sensor
static const uint8_t REG_DIST_LOW = 0x00;
static const uint8_t REG_DIST_HIGH = 0x01;
static const uint8_t REG_AMP_LOW = 0x02;
static const uint8_t REG_AMP_HIGH = 0x03;
static const uint8_t REG_VERSION_MAJOR = 0x0C;

// Limits matching real sensor
static const uint16_t MAX_DIST_CM = 2000;  // 20m
static const uint16_t MIN_DIST_CM = 1;     // 1cm
static const uint16_t UPDATE_RATE_HZ = 20; // Sensor update rate

void TFS20L::update(const class Aircraft &aircraft)  
{
    // Rate limit updates to match real sensor (20Hz)
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_update_ms < (1000U / UPDATE_RATE_HZ)) {
        return;
    }
    last_update_ms = now_ms;

    // Get range and convert to cm for checking limits
    rangefinder_range = aircraft.rangefinder_range();
    
    // Check for invalid readings first (infinite or negative)
    // Also check before casting to uint16_t to avoid overflow (>655m would wrap)
    if (rangefinder_range <= 0 || rangefinder_range > (MAX_DIST_CM * 0.01f)) {
        rangefinder_range = 0;  // Will trigger out of range
        strength = 0;          // Low strength for invalid reading
        return;
    }
    
    const uint16_t range_cm = rangefinder_range * 100.0f;
    
    // Apply real sensor limits (should always be in range due to check above)
    if (range_cm < MIN_DIST_CM) {
        rangefinder_range = 0;  // Will trigger out of range
        strength = 0;          // Low strength for invalid reading
    } else {
        // Simulate decreasing signal strength with distance
        const uint32_t str = 65535 - (range_cm * 20);
        strength = str < MIN_STRENGTH ? MIN_STRENGTH : (str > 65535 ? 65535 : str);
    }
}

int TFS20L::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 1) {
        // Register selection
        reg = data->msgs[0].buf[0];
        return 0;
    }

    if ((data->nmsgs != 2) || !(data->msgs[1].flags & I2C_M_RD)) {
        return -1;
    }

    const uint16_t dist_cm = rangefinder_range * 100.0f;

    // Handle register reads
    switch (reg) {
        case REG_DIST_LOW:
        case REG_DIST_HIGH:
        case REG_AMP_LOW:
        case REG_AMP_HIGH: {
            const uint8_t count = data->msgs[1].len;
            if (reg + count > REG_AMP_HIGH + 1) {
                return -1;
            }
            // Fill in sequential registers starting at reg
            for (uint8_t i = 0; i < count; i++) {
                const uint8_t curr_reg = reg + i;
                if (curr_reg == REG_DIST_LOW) {
                    data->msgs[1].buf[i] = dist_cm & 0xFF;
                } else if (curr_reg == REG_DIST_HIGH) {
                    data->msgs[1].buf[i] = (dist_cm >> 8) & 0xFF;
                } else if (curr_reg == REG_AMP_LOW) {
                    data->msgs[1].buf[i] = strength & 0xFF;
                } else if (curr_reg == REG_AMP_HIGH) {
                    data->msgs[1].buf[i] = (strength >> 8) & 0xFF;
                }
            }
            return 0;
        }
        case REG_VERSION_MAJOR:
            // Return version 1.0.0
            data->msgs[1].buf[0] = 1;
            return 0;
    }
    return -1;
}

#endif // AP_SIM_TFS20L_ENABLED
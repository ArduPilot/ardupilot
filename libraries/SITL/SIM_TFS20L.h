#pragma once

/*
  Simulator for the Benewake TFS20L rangefinder

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter --speedup=1

param set RNGFND1_TYPE 45  # TFS20L
param set RNGFND1_ADDR 0x10
graph RANGEFINDER.distance
graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance
reboot

arm throttle
rc 3 1600
*/

#include "SIM_config.h"

#if AP_SIM_TFS20L_ENABLED

#include <cstdint>
#include "SIM_I2CDevice.h"

namespace SITL {

class TFS20L : public I2CDevice {
public:
    void init() override {
        rangefinder_range = 0;
        strength = MIN_STRENGTH;
        reg = 0;
        last_update_ms = 0;
    }

    void update(const class Aircraft &aircraft) override;

    // Used for testing
    void update(float distance_m) {
        rangefinder_range = distance_m;
        strength = MIN_STRENGTH;
    }

#ifdef SITL_I2C_TEST
    // Read distance directly for testing
    void read_distance(uint8_t *buf) {
        uint16_t dist_cm = rangefinder_range * 100.0f;
        buf[0] = dist_cm & 0xFF;         // Low byte
        buf[1] = (dist_cm >> 8) & 0xFF;  // High byte
    }
#endif

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:
    float rangefinder_range;         // Range in meters
    uint16_t strength{MIN_STRENGTH}; // Signal strength
    uint8_t reg{0};                  // Current register for I2C operations
    uint32_t last_update_ms;         // Last time sensor was updated
    static const uint16_t MIN_STRENGTH = 100;
};

} // namespace SITL

#endif  // AP_SIM_TFS20L_ENABLED
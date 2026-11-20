#pragma once

/*
  Simulator for the Pono TSD20 I2C-connected rangefinder

  ./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter --speedup=1

  param set RNGFND1_TYPE 49
  param set RNGFND1_ADDR 0x52
  graph RANGEFINDER.distance
  graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance
  reboot

  arm throttle
  rc 3 1600
*/

#include "SIM_config.h"

#if AP_SIM_PONO_TSD20_I2C_ENABLED

#include <cstdint>
#include "SIM_I2CDevice.h"

namespace SITL {

class Pono_TSD20_I2C : public I2CDevice {
public:
    void init() override {
        rangefinder_range = 0;
        reg = 0;
        laser_on = false;
        last_update_ms = 0;
    }

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:
    float rangefinder_range;        // range in metres
    uint8_t reg;                    // current register for I2C operations
    bool laser_on;                  // laser state, register 0x02
    uint32_t last_update_ms;        // last time sensor was updated
};

} // namespace SITL

#endif  // AP_SIM_PONO_TSD20_I2C_ENABLED

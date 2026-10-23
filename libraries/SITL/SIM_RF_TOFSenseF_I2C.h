#pragma once

/*
  Simulator for the TOFSenseF I2C-connected rangefinder

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter --speedup=1

param set RNGFND1_TYPE 40
# param set RNGFND1_ADDR 0x10
param ftp
param set RNGFND1_MAX 10  # ????
graph DISTANCE_SENSOR[0].current_distance*0.01
graph GLOBAL_POSITION_INT.relative_alt/1000-DISTANCE_SENSOR[0].current_distance*0.01
reboot

arm throttle
rc 3 1600
*/

#include "SIM_config.h"

#if AP_SIM_RF_TOFSENSEF_I2C_ENABLED

#include "SIM_I2CDevice.h"

namespace SITL {

class TOFSenseF_I2C : public I2CDevice
{
public:

    using I2CDevice::I2CDevice;

    void update(const class Aircraft &aircraft) override;

private:

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;
    int take_reading(I2C::i2c_rdwr_ioctl_data *&data);
    int read_data(I2C::i2c_rdwr_ioctl_data *&data);

    uint32_t last_update_ms;

    float range;

    bool reading_requested;
};

} // namespace SITL

#endif  // AP_SIM_RF_TOFSENSEF_I2C_ENABLED

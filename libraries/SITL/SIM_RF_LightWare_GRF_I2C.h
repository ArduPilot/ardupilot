#pragma once

/*
   Simulator for the LightWare GRF rangefinder on I2C.

   Usage:
     ./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter --speedup=1

     param set RNGFND1_TYPE 48      # LightWare-GRF-I2C
     param set RNGFND1_ADDR 102     # 0x66
     reboot
*/

#include "SIM_config.h"

#if AP_SIM_RF_LIGHTWARE_GRF_I2C_ENABLED

#include "SIM_I2CDevice.h"

namespace SITL {

class LightWareGRF_I2C : public I2CDevice
{
public:
    using I2CDevice::I2CDevice;

    void update(const class Aircraft &aircraft) override;

private:
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    // Handle an incoming write. The first byte is the register we're
    // selecting; any remaining bytes are data for that register. Returns
    // the register id so the caller can respond to a follow-up read.
    uint8_t apply_write(const uint8_t *buf, uint16_t len);

    // Fill buf with the bytes this register would return on the real sensor.
    uint16_t read_register(uint8_t reg, uint8_t *buf, uint16_t buf_max);

    // Sensor state mirrored from writes the autopilot sends us.
    uint32_t distance_output_mask = (1U << 0) | (1U << 2); // first_raw + first_strength
    uint8_t  update_rate_hz = 5;

    // Latest simulated range from the aircraft model (metres).
    float range_m;

    // Tracks the most-recent register select for split write+read flows.
    uint8_t last_register;
};

} // namespace SITL

#endif  // AP_SIM_RF_LIGHTWARE_GRF_I2C_ENABLED

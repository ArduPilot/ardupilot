#pragma once

/*
  Simulator for the Benewake TFMiniPlus i2c-connected rangefinder

  Currently we accept writes to the rate register but do not act upon it
  Output must be enabled before a reading is requested from the device
  We do understand and act on the unit-type configuration - so if the driver doesn't set the units we do report strange altitudes

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter --speedup=1

param set RNGFND1_TYPE 25
param set RNGFND1_ADDR 0x10
param ftp
param set RNGFND1_MAX 10  # ????
graph DISTANCE_SENSOR[0].current_distance*0.01
graph GLOBAL_POSITION_INT.relative_alt/1000-DISTANCE_SENSOR[0].current_distance*0.01
reboot

arm throttle
rc 3 1600
*/

#include "SIM_config.h"

#if AP_SIM_RF_BENEWAKE_TFMINIPLUS_ENABLED

#include "SIM_I2CDevice.h"

namespace SITL {

class Benewake_TFMiniPlus : public I2CDevice
{
public:

    using I2CDevice::I2CDevice;

    void init() override;

    void update(const class Aircraft &aircraft) override {
        const uint32_t now_ms = AP_HAL::millis();
        if (last_update_ms - now_ms < 50) {
            return;
        }
        last_update_ms = now_ms;
        range = aircraft.rangefinder_range();
    }

private:

    enum class DataTypeRequested {
        NONE = 0,
        FW_VERSION = 1,
        READING = 2,
    };
    DataTypeRequested data_type_requested;  // defaults to NONE

    enum class Command {
        FW_VERSION = 0x01,
        SYSTEM_RESET = 0x04,
        OUTPUT_FORMAT_CM = 0x05,
        ENABLE_DATA_OUTPUT = 0x07,
        SET_FRAME_RATE = 0x03,
        SAVE_SETTINGS = 0x11,
        READ_MEASUREMENT = 0x00,
    };

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    float range;
    uint32_t last_update_ms;

    struct Config {
        uint16_t frame_rate;
        bool output_format_is_cm;
        bool data_output_enabled;
    };
    Config running_config;
    Config new_config;

    void reset() { running_config = new_config; }
};

} // namespace SITL

#endif  // AP_SIM_RF_BENEWAKE_TFMINIPLUS_ENABLED

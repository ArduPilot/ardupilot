#pragma once

#include "SIM_I2CDevice.h"

namespace SITL {

class SN_GCJA5 : public I2CDevice
{
public:

    SN_GCJA5() :
        I2CDevice() {
        // _status.sensor_status = 3;
        // _status.pd_status = 1;
        // _status.ld_operational_status = 0;
        // _status.fan_status = 0;
        _status.byte = 77;
    }

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:
    uint32_t cmd_take_reading_received_ms;

    union PACKED {
        uint8_t byte;
        struct {
            uint8_t sensor_status : 2;
            uint8_t pd_status : 2;
            uint8_t ld_operational_status : 2;
            uint8_t fan_status : 2;
        };
    } _status;


};

} // namespace SITL

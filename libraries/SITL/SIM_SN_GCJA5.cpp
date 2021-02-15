#include "SIM_SN_GCJA5.h"

#include <SITL/SITL.h>
#include <SITL/SIM_Aircraft.h>

#include <stdio.h>
int SITL::SN_GCJA5::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs == 2) {
        // return sensor state here - don't update the state.
        if (data->msgs[0].flags != 0) {
            AP_HAL::panic("Unexpected flags");
        }
        if (data->msgs[1].flags != I2C_M_RD) {
            AP_HAL::panic("Unexpected flags");
        }
        data->msgs[1].buf[0] = _status.byte;
        return 0;
    }

    if (data->nmsgs == 1) {
        AP_HAL::panic("No writable registers");
    }

    return -1;
};

void SITL::SN_GCJA5::update(const Aircraft &aircraft)
{
    // called rapidly; update the sensor state here
    // gcs().send_text(MAV_SEVERITY_INFO, "Called");
}

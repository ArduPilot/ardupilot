/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Simulated i2c buses and devices
*/


#include <GCS_MAVLink/GCS.h>
#include <SITL/SITL.h>

#include "SIM_I2C.h"
#include "SIM_ToshibaLED.h"
#include "SIM_MaxSonarI2CXL.h"

#include <signal.h>

using namespace SITL;

enum class IOCtlType {
    RDWR = 0,
};

class IgnoredI2CDevice : public I2CDevice
{
public:
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override {
        return -1;
    }
};
static IgnoredI2CDevice ignored;

static ToshibaLED toshibaled;
static MaxSonarI2CXL maxsonari2cxl;

struct i2c_device_at_address {
    uint8_t bus;
    uint8_t addr;
    I2CDevice &device;
} i2c_devices[] {
    { 1, 0x55, toshibaled },
    { 1, 0x38, ignored }, // NCP5623
    { 1, 0x39, ignored }, // NCP5623C
    { 1, 0x40, ignored }, // KellerLD
    { 1, 0x70, maxsonari2cxl },
    { 1, 0x76, ignored }, // MS56XX
};

void I2C::update(const class Aircraft &aircraft)
{
    for (auto daa : i2c_devices) {
        daa.device.update(aircraft);
    }
}

int I2C::ioctl_rdwr(i2c_rdwr_ioctl_data *data)
{
    int ret = 0;
    for (uint8_t i=0; i<data->nmsgs; i++) {
        const i2c_msg &msg = data->msgs[i];
        const uint8_t addr = msg.addr;
        bool handled = false;
        for (auto dev_at_address : i2c_devices) {
            // where's the bus check?!
            if (dev_at_address.addr == addr) {
                ret = dev_at_address.device.rdwr(data);
                handled = true;
            }
        }
        if (!handled) {
            ::fprintf(stderr, "Unhandled i2c message: addr=0x%x flags=%u len=%u\n", msg.addr, msg.flags, msg.len);
            return -1;  // ?!
        }
    }
    return ret;
}

int I2C::ioctl(uint8_t ioctl_type, void *data)
{
    switch ((IOCtlType) ioctl_type) {
    case IOCtlType::RDWR:
        return ioctl_rdwr((i2c_rdwr_ioctl_data*)data);
    }
    return -1;
}

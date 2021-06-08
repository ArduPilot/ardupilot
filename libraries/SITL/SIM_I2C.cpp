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
#include "SIM_BattMonitor_SMBus_Maxell.h"
#include "SIM_BattMonitor_SMBus_Rotoye.h"
#include "SIM_Airspeed_DLVR.h"
#include "SIM_Temperature_TSYS01.h"
#include "SIM_ICM40609.h"

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
static MaxSonarI2CXL maxsonari2cxl_2;
static Maxell maxell;
static Rotoye rotoye;
static Airspeed_DLVR airspeed_dlvr;
static TSYS01 tsys01;
static ICM40609 icm40609;

struct i2c_device_at_address {
    uint8_t bus;
    uint8_t addr;
    I2CDevice &device;
} i2c_devices[] {
    { 0, 0x70, maxsonari2cxl },
    { 0, 0x71, maxsonari2cxl_2 },
    { 1, 0x01, icm40609 },
    { 1, 0x55, toshibaled },
    { 1, 0x38, ignored }, // NCP5623
    { 1, 0x39, ignored }, // NCP5623C
    { 1, 0x40, ignored }, // KellerLD
    { 1, 0x76, ignored }, // MS56XX
    { 1, 0x77, tsys01 },
    { 1, 0x0B, rotoye },
    { 2, 0x0B, maxell },
    { 2, 0x28, airspeed_dlvr },
};

void I2C::init()
{
    for (auto &i : i2c_devices) {
        i.device.init();
    }

    // sanity check the i2c_devices structure to ensure we don't have
    // two devices at the same address on the same bus:
    for (uint8_t i=0; i<ARRAY_SIZE(i2c_devices)-1; i++) {
        const auto &dev_i = i2c_devices[i];
        for (uint8_t j=i+1; j<ARRAY_SIZE(i2c_devices); j++) {
            const auto &dev_j = i2c_devices[j];
            if (dev_i.bus == dev_j.bus &&
                dev_i.addr == dev_j.addr) {
                AP_HAL::panic("Two devices at the same address on the same bus");
            }
        }
    }
}

void I2C::update(const class Aircraft &aircraft)
{
    for (auto daa : i2c_devices) {
        daa.device.update(aircraft);
    }
}

int I2C::ioctl_rdwr(i2c_rdwr_ioctl_data *data)
{
    const uint8_t addr = data->msgs[0].addr;
    const uint8_t bus = data->msgs[0].bus;
    for (auto &dev_at_address : i2c_devices) {
        if (dev_at_address.addr != addr) {
            continue;
        }
        if (dev_at_address.bus != bus) {
            continue;
        }
        return dev_at_address.device.rdwr(data);
    }
//            ::fprintf(stderr, "Unhandled i2c message: bus=%u addr=0x%02x flags=%u len=%u\n", msg.bus, msg.addr, msg.flags, msg.len);
    return -1;  // ?!
}

int I2C::ioctl(uint8_t ioctl_type, void *data)
{
    switch ((IOCtlType) ioctl_type) {
    case IOCtlType::RDWR:
        return ioctl_rdwr((i2c_rdwr_ioctl_data*)data);
    }
    return -1;
}

/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>

#include "Util.h"

namespace VRBRAIN {

uint8_t VRBRAIN::VRBRAIN_I2C::instance;

/*
  implement wrapper for VRBRAIN I2C driver
 */
bool VRBRAIN_I2C::do_transfer(uint8_t address, const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{
    if (!init_done) {
        init_done = true;
        // we do late init() so we can setup the device paths
        snprintf(devname, sizeof(devname), "AP_I2C_%u", instance);
        snprintf(devpath, sizeof(devpath), "/dev/api2c%u", instance);
        init_ok = (init() == OK);
        if (init_ok) {
            instance++;
        }
    }
    if (!init_ok) {
        return false;
    }
    set_address(address);
    bool ret = (transfer(send, send_len, recv, recv_len) == OK);
    return ret;
}
    
I2CDevice::I2CDevice(uint8_t bus, uint8_t address) :
    _bus(bus),
    _address(address)
{
}
    
I2CDevice::~I2CDevice()
{
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    return _bus.do_transfer(_address, send, send_len, recv, recv_len);
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    return false;
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address)
{
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address));
    return dev;
}

}

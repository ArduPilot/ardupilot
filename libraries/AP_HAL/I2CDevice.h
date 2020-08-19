/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
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
#pragma once

#include <inttypes.h>
#include <vector>

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

namespace AP_HAL {

class I2CDevice : public Device {
public:
    I2CDevice() : Device(BUS_TYPE_I2C) { }

    virtual ~I2CDevice() { }

    /* Device implementation */

    /* See Device::set_speed() */
    virtual bool set_speed(Device::Speed speed) override = 0;

    /* See Device::transfer() */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    /*
     * Read location from device multiple times, advancing the buffer each
     * time
     */
    virtual bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                         uint32_t recv_len, uint8_t times) = 0;

    /* See Device::get_semaphore() */
    virtual Semaphore *get_semaphore() override = 0;

    /* See Device::register_periodic_callback() */
    virtual Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, Device::PeriodicCb) override = 0;

    /* See Device::adjust_periodic_callback() */
    virtual bool adjust_periodic_callback(
        Device::PeriodicHandle h, uint32_t period_usec) override = 0;

    /*
     * Force I2C transfers to be split between send and receive parts, with a
     * stop condition between them. Setting this allows to conveniently
     * continue using the read_* and transfer() methods on those devices.
     *
     * Some platforms may have transfers always split, in which case
     * this method is not needed.
     */
    virtual void set_split_transfers(bool set) {};
};

class I2CDeviceManager {
public:
    /* Get a device handle */
    virtual OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address,
                                                 uint32_t bus_clock=400000,
                                                 bool use_smbus = false,
                                                 uint32_t timeout_ms=4) = 0;
    /*
     * Get device by looking up the I2C bus on the buses from @devpaths.
     *
     * Each string in @devpaths are possible locations for the bus. How the
     * strings are implemented are HAL-specific. On Linux this is the info
     * returned by 'udevadm info -q path /dev/i2c-X'. The first I2C bus
     * matching a prefix in @devpaths is used to create a I2CDevice object.
     */
    virtual OwnPtr<I2CDevice> get_device(std::vector<const char *> devpaths,
                                         uint8_t address) {
        // Not implemented
        return nullptr;
    }

    /*
      get mask of bus numbers for all configured I2C buses
     */
    virtual uint32_t get_bus_mask(void) const { return 0x0F; }

    /*
      get mask of bus numbers for all configured external I2C buses
     */
    virtual uint32_t get_bus_mask_external(void) const { return 0x0F; }

    /*
      get mask of bus numbers for all configured internal I2C buses
     */
    virtual uint32_t get_bus_mask_internal(void) const { return 0x01; }
};

/*
  convenient macros for iterating over I2C bus numbers
 */
#define FOREACH_I2C_MASK(i,mask) for (uint32_t _bmask=mask, i=0; i<32; i++) if ((1U<<i)&_bmask)
#define FOREACH_I2C_EXTERNAL(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask_external())
#define FOREACH_I2C_INTERNAL(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask_internal())
#define FOREACH_I2C(i) FOREACH_I2C_MASK(i,hal.i2c_mgr->get_bus_mask())

}

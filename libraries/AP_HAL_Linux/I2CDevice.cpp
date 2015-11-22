/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#include "I2CDevice.h"

#include <algorithm>
#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <AP_HAL/AP_HAL.h>

#include "Util.h"

namespace Linux {

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

/* Private struct to maintain for each bus */
class I2CBus {
public:
    ~I2CBus()
    {
        if (fd >= 0) {
            ::close(fd);
        }
    }

    int open(uint8_t n)
    {
        char path[sizeof("/dev/i2c-XXX")];
        int r;

        if (fd >= 0) {
            return -EBUSY;
        }

        // TODO: open the bus
        bus = n;

        return fd;
    }

    Semaphore sem;
    int fd = -1;
    uint8_t bus;

    uint8_t ref;
};

I2CDevice::~I2CDevice()
{
    // Unregister itself from the I2CDeviceManager
    I2CDeviceManager::from(hal.i2c_mgr)->_unregister(_bus);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    // TODO: implement I2C transfer
    return false;
}

AP_HAL::Semaphore *I2CDevice::get_semaphore()
{
    return &_bus.sem;
}

int I2CDevice::get_fd()
{
    return _bus.fd;
}

I2CDeviceManager::I2CDeviceManager()
{
    /* Reserve space up-front for 4 buses */
    _buses.reserve(4);
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(std::vector<const char *> devpaths, uint8_t address)
{
    // implement device search on sysfs
    return nullptr;
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address)
{
    for (uint8_t i = 0, n = _buses.size(); i < n; i++) {
        if (_buses[i]->bus == bus) {
            return _create_device(*_buses[i], address);
        }
    }

    /* Bus not found for this device, create a new one */
    AP_HAL::OwnPtr<I2CBus> b{new I2CBus()};
    if (!b) {
        return nullptr;
    }

    if (b->open(bus) < 0) {
        return nullptr;
    }

    auto dev = _create_device(*b, address);
    if (!dev) {
        return nullptr;
    }

    _buses.push_back(b.leak());

    return dev;
}

/* Create a new device increasing the bus reference */
AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::_create_device(I2CBus &b, uint8_t address) const
{
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(b, address));
    if (!dev) {
        return nullptr;
    }
    b.ref++;
    return dev;
}

void I2CDeviceManager::_unregister(I2CBus &b)
{
    assert(b.ref > 0);

    if (--b.ref > 0) {
        return;
    }

    for (auto it = _buses.begin(); it != _buses.end(); it++) {
        if ((*it)->bus == b.bus) {
            _buses.erase(it);
            delete &b;
            break;
        }
    }
}

}

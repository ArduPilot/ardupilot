/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
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
#include "SPIDevice.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "Util.h"

namespace Linux {

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

/* Private struct to maintain for each bus */
class SPIBus {
public:
    ~SPIBus()
    {
        if (fd >= 0) {
            ::close(fd);
        }
    }

    int open(uint16_t bus_, uint16_t kernel_cs_)
    {
        char path[sizeof("/dev/spidevXXXXX.XXXXX")];

        if (fd > 0) {
            return -EBUSY;
        }

        snprintf(path, sizeof(path), "/dev/spidev%u.%u", bus_, kernel_cs_);
        fd = ::open(path, O_RDWR | O_CLOEXEC);
        if (fd < 0) {
            AP_HAL::panic("SPI: unable to open SPI bus %s: %s",
                          path, strerror(errno));
        }

        bus = bus_;
        kernel_cs = kernel_cs_;

        return fd;
    }

    Semaphore sem;
    int fd = -1;
    uint16_t bus;
    uint16_t kernel_cs;
    uint8_t ref;
};

SPIDevice::SPIDevice(SPIBus &bus, SPIDeviceDriver &device_desc)
    : _bus(bus)
    , _desc(device_desc)
{
    _desc.init();
    _cs_release();
}

SPIDevice::~SPIDevice()
{
    // Unregister itself from the SPIDeviceManager
    SPIDeviceManager::from(hal.spi)->_unregister(_bus);
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    switch (speed) {
    case AP_HAL::Device::SPEED_HIGH:
        _desc._speed = _desc._highspeed;
        break;
    case AP_HAL::Device::SPEED_LOW:
        _desc._speed = _desc._lowspeed;
        break;
    }

    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    struct spi_ioc_transfer msgs[2] = { };
    unsigned nmsgs = 0;

    assert(_bus.fd >= 0);

    if (send && send_len != 0) {
        msgs[nmsgs].tx_buf = (uint64_t) send;
        msgs[nmsgs].rx_buf = 0;
        msgs[nmsgs].len = send_len;
        msgs[nmsgs].speed_hz = _desc._speed;
        msgs[nmsgs].delay_usecs = 0;
        msgs[nmsgs].bits_per_word = _desc._bitsPerWord;
        msgs[nmsgs].cs_change = 0;
        nmsgs++;
    }

    if (recv && recv_len != 0) {
        msgs[nmsgs].tx_buf = 0;
        msgs[nmsgs].rx_buf = (uint64_t) recv;
        msgs[nmsgs].len = recv_len;
        msgs[nmsgs].speed_hz = _desc._speed;
        msgs[nmsgs].delay_usecs = 0;
        msgs[nmsgs].bits_per_word = _desc._bitsPerWord;
        msgs[nmsgs].cs_change = 0;
        nmsgs++;
    }

    if (!nmsgs) {
        return false;
    }

    int r = ioctl(_bus.fd, SPI_IOC_WR_MODE, &_desc._mode);
    if (r < 0) {
        hal.console->printf("SPIDevice: error on setting mode fd=%d (%s)\n",
                            _bus.fd, strerror(errno));
        return false;
    }

    _cs_assert();
    r = ioctl(_bus.fd, SPI_IOC_MESSAGE(nmsgs), &msgs);
    _cs_release();

    if (r == -1) {
        hal.console->printf("SPIDevice: error transferring data fd=%d (%s)\n",
                            _bus.fd, strerror(errno));
        return false;
    }

    return true;
}

void SPIDevice::_cs_assert()
{
    if (_desc._cs_pin == SPI_CS_KERNEL) {
        return;
    }

    _desc._cs->write(0);
}

void SPIDevice::_cs_release()
{
    if (_desc._cs_pin == SPI_CS_KERNEL) {
        return;
    }

    _desc._cs->write(1);
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &_bus.sem;
}

int SPIDevice::get_fd()
{
    return _bus.fd;
}

AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(const char *name)
{
    SPIDeviceDriver *desc = nullptr;

    /* Find the bus description in the table */
    for (uint8_t i = 0; i < _n_device_desc; i++) {
        if (!strcmp(_device[i]._name, name)) {
            desc = &_device[i];
            break;
        }
    }

    if (!desc) {
        AP_HAL::panic("SPI: invalid device name");
    }

    return get_device(*desc);
}

AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(SPIDeviceDriver &desc)
{
    /* Find if bus is already open */
    for (uint8_t i = 0, n = _buses.size(); i < n; i++) {
        if (_buses[i]->bus == desc._bus &&
            _buses[i]->kernel_cs == desc._subdev) {
            return _create_device(*_buses[i], desc);
        }
    }

    /* Bus not found for this device, create a new one */
    AP_HAL::OwnPtr<SPIBus> b{new SPIBus()};
    if (!b) {
        return nullptr;
    }

    if (b->open(desc._bus, desc._subdev) < 0) {
        return nullptr;
    }

    auto dev = _create_device(*b, desc);
    if (!dev) {
        return nullptr;
    }

    _buses.push_back(b.leak());

    return dev;
}

/* Create a new device increasing the bus reference */
AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::_create_device(SPIBus &b, SPIDeviceDriver &desc) const
{
    auto dev = AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice(b, desc));
    if (!dev) {
        return nullptr;
    }
    b.ref++;
    return dev;
}

void SPIDeviceManager::_unregister(SPIBus &b)
{
    if (b.ref == 0 || --b.ref > 0) {
        return;
    }

    for (auto it = _buses.begin(); it != _buses.end(); it++) {
        if ((*it)->bus == b.bus && (*it)->kernel_cs == b.kernel_cs) {
            _buses.erase(it);
            delete &b;
            break;
        }
    }
}

}

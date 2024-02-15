/*
 * Copyright (C) 2021  Peter Barker. All rights reserved.
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

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

#include <SITL/SITL.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

/*
 * SPIBus
 */
class SPIBus {
    friend class SPIDeviceManager;
public:

    SPIBus(uint8_t _bus) :
        bus{_bus}
        { }

    uint8_t bus;
    Semaphore sem;
    int ioctl(uint8_t cs_pin, uint8_t ioctl_number, void *data) {
        return _ioctl(cs_pin, ioctl_number, data);
    }
    // void _timer_tick(); // in lieu of a thread-per-bus
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb);

    class SPIBus *next;

private:
    int _ioctl(uint8_t cs_pin, uint8_t ioctl_number, void *data);

    struct callback_info {
        struct callback_info *next;
        AP_HAL::Device::PeriodicCb cb;
        uint32_t period_usec;
        uint64_t next_usec;
    } *callbacks;

    static uint8_t spi_buscount;

};

uint8_t SPIBus::spi_buscount;

// FIXME: do the whole subdev dance here and get an actual FD to do an ioctl on?
int SPIBus::_ioctl(uint8_t cs_pin, uint8_t ioctl_number, void *data)
{
    SITL::SIM *sitl = AP::sitl();
    return sitl->spi_ioctl(bus, cs_pin, ioctl_number, data);
}

AP_HAL::Device::PeriodicHandle SPIBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    // mostly swiped from ChibiOS:
    SPIBus::callback_info *callback = new SPIBus::callback_info;
    if (callback == nullptr) {
        return nullptr;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    // add to linked list of callbacks on thread
    callback->next = callbacks;
    callbacks = callback;
    return callback;
}

// void SPIBus::_timer_tick()
// {
//     const uint64_t now = AP_HAL::micros64();
//     for (struct callback_info *ci = callbacks; ci != nullptr; ci = ci->next) {
//         if (ci->next_usec < now) {
//             WITH_SEMAPHORE(sem);
//             ci->cb();
//             ci->next_usec += ci->period_usec;
//         }
//     }
// }

/*
 * SPIDeviceManager
 */
SPIBus *SPIDeviceManager::buses;

SPIDeviceManager::SPIDeviceManager()
{
}

static const struct SPIDriverInfo {
    // SPIDriver *driver;
    uint8_t busid; // used for device IDs in parameters
    // uint8_t dma_channel_rx;
    // uint8_t dma_channel_tx;
} spi_devices[] = {
    { 0, },
};

// name, bus, cs_pin
SPIDesc SPIDeviceManager::device_table[] = {
    { "ramtron", 0, 0 },
    { "dataflash", 1, 0}
};

AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(const char *name)
{
    // this was swiped from AP_HAL_ChibiOS

    /* Find the bus description in the table */
    uint8_t i;
    for (i = 0; i<ARRAY_SIZE(device_table); i++) {
        if (strcmp(device_table[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_table)) {
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(nullptr);
    }

    SPIDesc &desc = device_table[i];

    // find the bus
    SPIBus *busp;
    for (busp = buses; busp; busp = (SPIBus *)busp->next) {
        if (busp->bus == desc.bus) {
            break;
        }
    }
    if (busp == nullptr) {
        // create a new one
        busp = new SPIBus(desc.bus);
        if (busp == nullptr) {
            return nullptr;
        }
        busp->next = buses;
        busp->bus = desc.bus;

        buses = busp;
    }

    return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice(*busp, desc));
}

// void SPIDeviceManager::_timer_tick()
// {
//     for (auto &bus : buses) {
//         bus._timer_tick();
//     }
// }

/*
 * SPIDevice
 */

SPIDevice::SPIDevice(SPIBus &_bus, SPIDesc &_device_desc)
    : bus(_bus)
    , device_desc(_device_desc)
{
    set_device_bus(spi_devices[_bus.bus].busid);
    set_device_address(_device_desc.cs_pin);
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &bus.sem;
}

/*
  low level transfer function - swiped from the Linux HAL
 */
// this is copied into the SITL SPI layer
struct spi_ioc_transfer {
	uint64_t tx_buf;
	uint64_t rx_buf;

	uint32_t len;
	// uint32_t speed_hz;

	// uint16_t delay_usecs;
	// uint8_t bits_per_word;
	// uint8_t cs_change;
};

#define SPI_TRANSACTION_1LONG 17
#define SPI_TRANSACTION_2LONG 18

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    bus.sem.check_owner();

    struct spi_ioc_transfer msgs[2] = { };
    unsigned nmsgs = 0;

    if (send && send_len != 0) {
        msgs[nmsgs].tx_buf = (uint64_t) send;
        msgs[nmsgs].rx_buf = 0;
        msgs[nmsgs].len = send_len;
        nmsgs++;
    }

    if (recv && recv_len != 0) {
        msgs[nmsgs].tx_buf = 0;
        msgs[nmsgs].rx_buf = (uint64_t) recv;
        msgs[nmsgs].len = recv_len;
        nmsgs++;
    }

    if (!nmsgs) {
        return false;
    }

    uint8_t ioctl_number;
    switch (nmsgs) {
    case 1:
        ioctl_number = SPI_TRANSACTION_1LONG;
        break;
    case 2:
        ioctl_number = SPI_TRANSACTION_2LONG;
        break;
    default:
        abort();
    }
    const int r = bus.ioctl(device_desc.cs_pin, ioctl_number, &msgs);

    if (r == -1) {
        hal.console->printf("SPIDevice: error transferring data\n");
        return false;
    }

    return true;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb);
}

#endif // HAL_BOARD_SITL

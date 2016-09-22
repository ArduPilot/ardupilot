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
#include <linux/i2c-dev.h>
/*
 * linux/i2c-dev.h is a kernel header, but some distros rename it to
 * linux/i2c-dev.h.kernel when i2c-tools is installed.  The header provided by
 * i2c-tools is old/broken and contains some symbols defined in
 * linux/i2c.h. The i2c.h will be only included if a well-known symbol is not
 * defined. This is a workaround while distros propagate the real fix like
 * http://lists.opensuse.org/archive/opensuse-commit/2015-10/msg00918.html (or
 * do like Archlinux that installs only the kernel header).
 */
#ifndef I2C_SMBUS_BLOCK_MAX
#include <linux/i2c.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "PollerThread.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include "Thread.h"
#include "Util.h"

/* Workaround broken header from i2c-tools */
#ifndef I2C_RDRW_IOCTL_MAX_MSGS
#define I2C_RDRW_IOCTL_MAX_MSGS 42
#endif

namespace Linux {

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

/*
 * TODO: move to Util or other upper class to be used by others
 *
 * Return pointer to the next char if @s starts with @prefix, otherwise
 * returns nullptr.
 */
static inline char *startswith(const char *s, const char *prefix)
{
    size_t len = strlen(prefix);
    if (strncmp(s, prefix, len) == 0) {
        return (char *) s + len;
    }
    return nullptr;
}

/* Private struct to maintain for each bus */
class I2CBus : public TimerPollable::WrapperCb {
public:
    ~I2CBus();

    /*
     * TimerPollable::WrapperCb methods to take
     * and release semaphore while calling the callback
     */
    void start_cb() override;
    void end_cb() override;

    int open(uint8_t n);

    PollerThread thread;
    Semaphore sem;
    int fd = -1;
    uint8_t bus;
    uint8_t ref;
};

I2CBus::~I2CBus()
{
    if (fd >= 0) {
        ::close(fd);
    }
}

void I2CBus::start_cb()
{
    sem.take(HAL_SEMAPHORE_BLOCK_FOREVER);
}

void I2CBus::end_cb()
{
    sem.give();
}

int I2CBus::open(uint8_t n)
{
    char path[sizeof("/dev/i2c-XXX")];
    int r;

    if (fd >= 0) {
        return -EBUSY;
    }

    r = snprintf(path, sizeof(path), "/dev/i2c-%u", n);
    if (r < 0 || r >= (int)sizeof(path)) {
        return -EINVAL;
    }

    fd = ::open(path, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        return -errno;
    }

    bus = n;

    return fd;
}

I2CDevice::~I2CDevice()
{
    // Unregister itself from the I2CDeviceManager
    I2CDeviceManager::from(hal.i2c_mgr)->_unregister(_bus);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    struct i2c_msg msgs[2] = { };
    unsigned nmsgs = 0;

    assert(_bus.fd >= 0);

    if (send && send_len != 0) {
        msgs[nmsgs].addr = _address;
        msgs[nmsgs].flags = 0;
        msgs[nmsgs].buf = const_cast<uint8_t*>(send);
        msgs[nmsgs].len = send_len;
        nmsgs++;
    }

    if (recv && recv_len != 0) {
        msgs[nmsgs].addr = _address;
        msgs[nmsgs].flags = I2C_M_RD;
        msgs[nmsgs].buf = recv;
        msgs[nmsgs].len = recv_len;
        nmsgs++;
    }

    /* interpret it as an input error if nothing has to be done */
    if (!nmsgs) {
        return false;
    }

    struct i2c_rdwr_ioctl_data i2c_data = { };

    i2c_data.msgs = msgs;
    i2c_data.nmsgs = nmsgs;

    int r;
    unsigned retries = _retries;
    do {
        r = ::ioctl(_bus.fd, I2C_RDWR, &i2c_data);
    } while (r == -1 && retries-- > 0);

    return r != -1;
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    const uint8_t max_times = I2C_RDRW_IOCTL_MAX_MSGS / 2;

    first_reg |= _read_flag;

    while (times > 0) {
        uint8_t n = MIN(times, max_times);
        struct i2c_msg msgs[2 * n];
        struct i2c_rdwr_ioctl_data i2c_data = { };

        memset(msgs, 0, 2 * n * sizeof(*msgs));

        i2c_data.msgs = msgs;
        i2c_data.nmsgs = 2 * n;

        for (uint8_t i = 0; i < i2c_data.nmsgs; i += 2) {
            msgs[i].addr = _address;
            msgs[i].flags = 0;
            msgs[i].buf = &first_reg;
            msgs[i].len = 1;
            msgs[i + 1].addr = _address;
            msgs[i + 1].flags = I2C_M_RD;
            msgs[i + 1].buf = recv;
            msgs[i + 1].len = recv_len;

            recv += recv_len;
        };

        int r;
        unsigned retries = _retries;
        do {
            r = ::ioctl(_bus.fd, I2C_RDWR, &i2c_data);
        } while (r == -1 && retries-- > 0);

        if (r == -1) {
            return false;
        }

        times -= n;
    }

    return true;
}

AP_HAL::Semaphore *I2CDevice::get_semaphore()
{
    return &_bus.sem;
}

AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    TimerPollable *p = _bus.thread.add_timer(cb, &_bus, period_usec);
    if (!p) {
        AP_HAL::panic("Could not create periodic callback");
    }

    if (!_bus.thread.is_started()) {
        char name[16];
        snprintf(name, sizeof(name), "ap-i2c-%u", _bus.bus);

        _bus.thread.set_stack_size(AP_LINUX_SENSORS_STACK_SIZE);
        _bus.thread.start(name, AP_LINUX_SENSORS_SCHED_POLICY,
                          AP_LINUX_SENSORS_SCHED_PRIO);
    }

    return static_cast<AP_HAL::Device::PeriodicHandle>(p);
}

bool I2CDevice::adjust_periodic_callback(
    AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return _bus.thread.adjust_timer(static_cast<TimerPollable*>(h), period_usec);
}

I2CDeviceManager::I2CDeviceManager()
{
    /* Reserve space up-front for 4 buses */
    _buses.reserve(4);
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(std::vector<const char *> devpaths, uint8_t address)
{
    const char *dirname = "/sys/class/i2c-dev/";
    struct dirent *de = nullptr;
    DIR *d;

    d = opendir(dirname);
    if (!d) {
        AP_HAL::panic("Could not get list of I2C buses");
    }

    for (de = readdir(d); de; de = readdir(d)) {
        char *str_device, *abs_str_device;
        const char *p;

        if (strcmp(de->d_name, ".") == 0 || strcmp(de->d_name, "..") == 0) {
            continue;
        }

        if (asprintf(&str_device, "%s/%s", dirname, de->d_name) < 0) {
            continue;
        }

        abs_str_device = realpath(str_device, nullptr);
        if (!abs_str_device || !(p = startswith(abs_str_device, "/sys/devices/"))) {
            free(abs_str_device);
            free(str_device);
            continue;
        }

        auto t = std::find_if(std::begin(devpaths), std::end(devpaths),
                              [p](const char *prefix) {
                                  return startswith(p, prefix) != nullptr;
                              });

        free(abs_str_device);
        free(str_device);

        if (t != std::end(devpaths)) {
            unsigned int n;

            /* Found the bus, try to create the device now */
            if (sscanf(de->d_name, "i2c-%u", &n) != 1) {
                AP_HAL::panic("I2CDevice: can't parse %s", de->d_name);
            }
            if (n > UINT8_MAX) {
                AP_HAL::panic("I2CDevice: bus with number n=%u higher than %u",
                              n, UINT8_MAX);
            }

            closedir(d);
            return get_device(n, address);
        }
    }

    /* not found */
    closedir(d);
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

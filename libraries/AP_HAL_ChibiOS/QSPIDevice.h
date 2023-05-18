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
 *
 * Code by 
 *         Andy Piper
 *         Siddharth Bharat Purohit, Cubepilot Pty. Ltd.
 */

#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/QSPIDevice.h>
#include "AP_HAL_ChibiOS.h"

#if HAL_USE_WSPI == TRUE && defined(HAL_QSPI_DEVICE_LIST)

#if !defined(HAL_BOOTLOADER_BUILD)
#include "Semaphores.h"
#endif
#include "hwdef/common/stm32_util.h"

#include "Scheduler.h"
#include "Device.h"

namespace ChibiOS
{

struct QSPIDesc {
    QSPIDesc(const char *_name, uint8_t _bus,
             uint32_t _mode, uint32_t _speed,
             uint8_t _size_pow2, uint8_t _ncs_clk_shift)
        : name(_name), bus(_bus), mode(_mode), speed(_speed),
          size_pow2(_size_pow2), ncs_clk_delay(_ncs_clk_shift)
    {
    }

    const char *name; // name of the device
    uint8_t bus; // qspi bus being used
    uint8_t device; // device id
    uint32_t mode; // clock mode
    uint32_t speed; // clock speed
    uint8_t size_pow2; // size as power of 2
    uint8_t ncs_clk_delay; // number of clk cycles to wait while transitioning NCS

};

class QSPIBus : public DeviceBus
{
public:
    QSPIBus(uint8_t _bus) :
        DeviceBus(APM_SPI_PRIORITY, true),
        bus(_bus) {}

    uint8_t bus;
    WSPIConfig wspicfg;
    bool qspi_started;
};

class QSPIDevice : public AP_HAL::QSPIDevice
{
public:
    static QSPIDevice *from(AP_HAL::QSPIDevice *dev)
    {
        return static_cast<QSPIDevice*>(dev);
    }

    QSPIDevice(QSPIBus &_bus, QSPIDesc &_device_desc) :
        bus(_bus),
        device_desc(_device_desc)
    {}

    bool set_speed(Speed speed) override
    {
        return true;
    }

    PeriodicHandle register_periodic_callback(uint32_t period_usec, PeriodicCb) override
    {
        return nullptr;
    }
    bool adjust_periodic_callback(PeriodicHandle h, uint32_t period_usec) override
    {
        return false;
    }

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    void set_cmd_header(const CommandHeader& cmd_hdr) override;

    AP_HAL::Semaphore* get_semaphore() override
    {
#if !defined(HAL_BOOTLOADER_BUILD)
        // if asking for invalid bus number use bus 0 semaphore
        return &bus.semaphore;
#else
        return nullptr;
#endif
    }

    bool acquire_bus(bool acquire);

    // Enters Memory mapped or eXecution In Place or 0-4-4 mode
    bool enter_xip_mode(void** map_ptr) override;
    bool exit_xip_mode() override;

private:
    QSPIBus &bus;
    QSPIDesc &device_desc;
    wspi_command_t mode;
};

class QSPIDeviceManager : public AP_HAL::QSPIDeviceManager
{
public:
    friend class QSPIDevice;

    static QSPIDeviceManager *from(AP_HAL::QSPIDeviceManager *qspi_mgr)
    {
        return static_cast<QSPIDeviceManager*>(qspi_mgr);
    }

    AP_HAL::OwnPtr<AP_HAL::QSPIDevice> get_device(const char *name) override;
private:
    static QSPIDesc device_table[];
    QSPIBus *buses;
};

}

#endif // #if HAL_USE_WSPI == TRUE && defined(HAL_QSPI_DEVICE_LIST)

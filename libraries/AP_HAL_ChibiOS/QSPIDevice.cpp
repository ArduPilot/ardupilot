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

#include <hal.h>
#include "QSPIDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_InternalError/AP_InternalError.h>
#include "Util.h"
#include "Scheduler.h"
#include <stdio.h>

#if HAL_USE_WSPI == TRUE && defined(HAL_QSPI_DEVICE_LIST)

using namespace ChibiOS;
extern const AP_HAL::HAL& hal;

static const struct QSPIDriverInfo {
    WSPIDriver *driver;
    uint8_t busid; // used for device IDs in parameters
} qspi_devices[] = { HAL_QSPI_BUS_LIST };

#define QSPIDEV_MODE1 0
#define QSPIDEV_MODE3 STM32_DCR_CK_MODE

// device list comes from hwdef.dat
QSPIDesc QSPIDeviceManager::device_table[] = { HAL_QSPI_DEVICE_LIST };

// Check clock sanity during runtime
#if (STM32_QSPICLK < HAL_QSPI1_CLK)
#error "Flash speed must not be greater than QSPI Clock"
#endif
#if (STM32_QSPICLK % HAL_QSPI1_CLK)
#warning "QSPI clock not an integer multiple of flash speed"
#endif

#if defined(STM32_WSPI_USE_QUADSPI2) && STM32_WSPI_USE_QUADSPI2
#if (STM32_QSPICLK < HAL_QSPI2_CLK)
#error "Flash speed must not be greater than QSPI Clock"
#endif
#if (STM32_QSPICLK % HAL_QSPI2_CLK)
#warning "QSPI clock not an integer multiple of flash speed"
#endif
#endif

bool QSPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len)
{
    if (!acquire_bus(true)) {
        return false;
    }
    if (!bus.bouncebuffer_setup(send, send_len, recv, recv_len)) {
        acquire_bus(false);
        return false;
    }
    bool ret = true;
    if (send_len == 0 && recv_len == 0) {
        // This is just a command
        ret = !wspiCommand(qspi_devices[device_desc.bus].driver, &mode);
    } else if (send_len > 0 && recv == 0) {
        // This is a send cmd
        ret = !wspiSend(qspi_devices[device_desc.bus].driver, &mode, send_len, send);
    } else if (send_len == 0 && recv_len >= 1) {
        // This is a receive cmd,
        // we only consume first byte of send
        ret = !wspiReceive(qspi_devices[device_desc.bus].driver, &mode, recv_len, recv);
    } else {
        // Can't handle this transaction type
        ret = false;
    }
    bus.bouncebuffer_finish(send, recv, recv_len);
    acquire_bus(false);
    return ret;
}

void QSPIDevice::set_cmd_header(const CommandHeader& cmd_hdr)
{
    mode.cmd   = cmd_hdr.cmd;
    mode.cfg   = cmd_hdr.cfg;
    mode.addr  = cmd_hdr.addr;
    mode.alt   = cmd_hdr.alt;
    mode.dummy = cmd_hdr.dummy;
}


bool QSPIDevice::acquire_bus(bool acquire)
{
    if (!bus.semaphore.check_owner()) {
        return false;
    }
    if (acquire) {
        wspiAcquireBus(qspi_devices[device_desc.bus].driver);
        if (qspi_devices[device_desc.bus].driver->config != &bus.wspicfg) {
            // Initialise and Start WSPI driver
            bus.wspicfg.end_cb = nullptr;
            bus.wspicfg.error_cb = nullptr;
            bus.wspicfg.dcr = STM32_DCR_FSIZE(device_desc.size_pow2) |
                              STM32_DCR_CSHT(device_desc.ncs_clk_delay - 1)   |
                              device_desc.mode;
            wspiStart(qspi_devices[device_desc.bus].driver, &bus.wspicfg);
        }
    } else {
        wspiReleaseBus(qspi_devices[device_desc.bus].driver);
    }
    return true;
}


// Enters Memory mapped or eXecution In Place or 0-4-4 mode
bool QSPIDevice::enter_xip_mode(void** map_ptr)
{
    if (!acquire_bus(true)) {
        return false;
    }
    wspiMapFlash(qspi_devices[device_desc.bus].driver, &mode, (uint8_t**)map_ptr);
    acquire_bus(false);
    return true;
}

bool QSPIDevice::exit_xip_mode()
{
    if (!acquire_bus(true)) {
        return false;
    }
    wspiUnmapFlash(qspi_devices[device_desc.bus].driver);
    acquire_bus(false);
    return true;
}

/*
  return a SPIDevice given a string device name
 */
AP_HAL::OwnPtr<AP_HAL::QSPIDevice>
QSPIDeviceManager::get_device(const char *name)
{
    /* Find the bus description in the table */
    uint8_t i;
    for (i = 0; i<ARRAY_SIZE(device_table); i++) {
        if (strcmp(device_table[i].name, name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_table)) {
        return AP_HAL::OwnPtr<AP_HAL::QSPIDevice>(nullptr);
    }

    QSPIDesc &desc = device_table[i];

    // find the bus
    QSPIBus *busp;
    for (busp = buses; busp; busp = (QSPIBus *)busp->next) {
        if (busp->bus == desc.bus) {
            break;
        }
    }
    if (busp == nullptr) {
        // create a new one
        busp = new QSPIBus(desc.bus);
        if (busp == nullptr) {
            return nullptr;
        }
        busp->next = buses;
        busp->bus = desc.bus;

        buses = busp;
    }

    return AP_HAL::OwnPtr<AP_HAL::QSPIDevice>(new QSPIDevice(*busp, desc));
}

#endif // #if HAL_USE_QSPI == TRUE && defined(HAL_QPI_DEVICE_LIST)

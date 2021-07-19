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
 * Code by Andy Piper and Siddharth Bharat Purohit
 */
#pragma once

#include <inttypes.h>
#include <vector>

#include "AP_HAL_Namespace.h"
#include "Device.h"
#include "utility/OwnPtr.h"

#ifndef HAL_USE_QSPI_DEFAULT_CFG
#define HAL_USE_QSPI_DEFAULT_CFG 1
#endif

namespace AP_HAL
{

// Underlying HAL implementation can override these
#if HAL_USE_QSPI_DEFAULT_CFG
namespace QSPI
{
constexpr uint32_t CFG_CMD_MODE_MASK        =   (3LU << 8LU);
constexpr uint32_t CFG_CMD_MODE_NONE        =   (0LU << 8LU);
constexpr uint32_t CFG_CMD_MODE_ONE_LINE    =   (1LU << 8LU);
constexpr uint32_t CFG_CMD_MODE_TWO_LINES   =   (2LU << 8LU);
constexpr uint32_t CFG_CMD_MODE_FOUR_LINES  =   (3LU << 8LU);

constexpr uint32_t CFG_CMD_SIZE_MASK     =  0LU;
constexpr uint32_t CFG_CMD_SIZE_8        =  0LU;

constexpr uint32_t CFG_ADDR_MODE_MASK        =  (3LU << 10LU);
constexpr uint32_t CFG_ADDR_MODE_NONE        =  (0LU << 10LU);
constexpr uint32_t CFG_ADDR_MODE_ONE_LINE    =  (1LU << 10LU);
constexpr uint32_t CFG_ADDR_MODE_TWO_LINES   =  (2LU << 10LU);
constexpr uint32_t CFG_ADDR_MODE_FOUR_LINES  =  (3LU << 10LU);


constexpr uint32_t CFG_ADDR_SIZE_MASK   =  (3LU << 12LU);
constexpr uint32_t CFG_ADDR_SIZE_8      =  (0LU << 12LU);
constexpr uint32_t CFG_ADDR_SIZE_16     =  (1LU << 12LU);
constexpr uint32_t CFG_ADDR_SIZE_24     =  (2LU << 12LU);
constexpr uint32_t CFG_ADDR_SIZE_32     =  (3LU << 12LU);

constexpr uint32_t CFG_ALT_MODE_MASK        =  (3LU << 14LU);
constexpr uint32_t CFG_ALT_MODE_NONE        =  (0LU << 14LU);
constexpr uint32_t CFG_ALT_MODE_ONE_LINE    =  (1LU << 14LU);
constexpr uint32_t CFG_ALT_MODE_TWO_LINES   =  (2LU << 14LU);
constexpr uint32_t CFG_ALT_MODE_FOUR_LINES  =  (3LU << 14LU);

constexpr uint32_t CFG_ALT_DDR              =  (1LU << 31LU);

constexpr uint32_t CFG_ALT_SIZE_MASK        =  (3LU << 16LU);
constexpr uint32_t CFG_ALT_SIZE_8           =  (0LU << 16LU);
constexpr uint32_t CFG_ALT_SIZE_16          =  (1LU << 16LU);
constexpr uint32_t CFG_ALT_SIZE_24          =  (2LU << 16LU);
constexpr uint32_t CFG_ALT_SIZE_32          =  (3LU << 16LU);

constexpr uint32_t CFG_DATA_MODE_MASK       =  (3LU << 24LU);
constexpr uint32_t CFG_DATA_MODE_NONE       =  (0LU << 24LU);
constexpr uint32_t CFG_DATA_MODE_ONE_LINE   =  (1LU << 24LU);
constexpr uint32_t CFG_DATA_MODE_TWO_LINES  =  (2LU << 24LU);
constexpr uint32_t CFG_DATA_MODE_FOUR_LINES =  (3LU << 24LU);

constexpr uint32_t CFG_DATA_DDR             =  (1LU << 31LU);

constexpr uint32_t CFG_SIOO                 =  (1LU << 28LU);
}
#endif //#if HAL_USE_QSPI_DEFAULT_CFG

class QSPIDevice : public Device
{
public:

    QSPIDevice() : Device(BUS_TYPE_QSPI) { }

    /* See AP_HAL::Device::transfer() */
    virtual bool transfer(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t recv_len) override = 0;

    // Set command header for upcomming transfer call(s)
    virtual void set_cmd_header(const CommandHeader& cmd_hdr) override = 0;

    virtual AP_HAL::Semaphore* get_semaphore() override = 0;

protected:
    uint32_t _trx_flags;
};

class QSPIDeviceManager
{
public:
    virtual OwnPtr<QSPIDevice> get_device(const char *name)
    {
        return nullptr;
    }

    /* Return the number of QSPI devices currently registered. */
    virtual uint8_t get_count() const
    {
        return 0;
    }

    /* Get qspi device name at @idx */
    virtual const char *get_device_name(uint8_t idx) const
    {
        return nullptr;
    }
};

}

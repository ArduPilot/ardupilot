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

#include <AP_HAL/HAL.h>
#include <AP_HAL/WSPIDevice.h>
#include <AP_HAL/utility/OwnPtr.h>

#ifndef HAL_USE_WSPI_DEFAULT_CFG
#define HAL_USE_WSPI_DEFAULT_CFG 1
#endif

namespace Empty
{

class WSPIDevice : public AP_HAL::WSPIDevice
{
public:

    WSPIDevice() { }

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override
    {
        return false;
    }

    // Set command header for upcomming transfer call(s)
    void set_cmd_header(const CommandHeader& cmd_hdr) override
    {
        return;
    }

    AP_HAL::Semaphore* get_semaphore() override
    {
        return nullptr;
    }
};

class WSPIDeviceManager : public AP_HAL::WSPIDeviceManager
{
};

}

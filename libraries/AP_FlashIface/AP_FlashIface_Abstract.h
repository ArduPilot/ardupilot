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
/*
    Implements Common frontend methods for Flash Interface Driver
*/
#pragma once

#include <AP_HAL/AP_HAL.h>

class AP_FlashIface
{

public:
    virtual bool init() = 0;

    /**
     * @details Read data from flash chip.
     *
     * @param[in] offset        address offset from where to start the read
     * @param[out] data         data to be read from the device
     * @param[in] size          size of the data to be read
     * @return                  The operation status.
     * @retval false            if the operation failed.
     * @retval true             if the operation succeeded.
     *
     */
    virtual bool read(uint32_t offset, uint8_t* data, uint32_t size) = 0;

    /**
     * @details Gets number bytes that can be written in one go (page size).
     *
     * @return                  page size in bytes.
     *
     */
    virtual uint32_t get_page_size() const = 0;

    /**
     * @details Gets number pages, each page can written in one go
     *
     * @return                  Number of pages.
     *
     */
    virtual uint32_t get_page_count() const = 0;

    /**
     * @details Sends command to start programming a page of the chip.
     *
     * @param[in]  page         Page number to be written to
     * @param[in]  data         data to be written
     * @param[out] delay_us     Time to wait until next is_device_busy call
     * @param[out] timeout_us   Time after which the erase should be timedout,
     *                          should be reset at every call.
     * @return                  The operation status.
     * @retval false            if the operation failed.
     * @retval true             if the operation succeeded.
     *
     */
    virtual bool start_program_page(uint32_t page, const uint8_t *data, uint32_t &delay_us, uint32_t &timeout_us) = 0;

    /**
     * @details Tries to program as much as possible starting from the offset
     *          until size. User needs to call this as many times as needed
     *          taking already programmed bytes into account.
     *
     * @param[in] offset        address offset for program
     * @param[in] data          data to be programmed
     * @param[in] size          size desired to be programmed
     * @param[out] programming  number of bytes programming, taking care of the limits
     * @param[out] delay_us     Time to wait until program typically finishes
     * @param[out] timeout_us   Time by which current program should have timedout.
     * @return                  The operation status.
     * @retval false            if the operation failed.
     * @retval true             if the operation succeeded.
     *
     */
    virtual bool start_program_offset(uint32_t offset, const uint8_t* data, uint32_t size, uint32_t &programming,
                                      uint32_t &delay_us, uint32_t &timeout_us)
    {
        return false;
    }

    /**
     * @details Gets number bytes that can erased in one go(sector size)
     *
     * @return                  Sector size in bytes.
     *
     */
    virtual uint32_t get_sector_size() const = 0;


    /**
     * @details Gets number of sectors, each sector can be erased in one go
     *
     * @return                  Number of sectors.
     *
     */
    virtual uint32_t get_sector_count() const = 0;

    /**
     * @details Sends command to erase the entire chips.
     *
     * @param[out] delay_ms     Time to wait until next is_device_busy call
     * @param[out] timeout_ms   Time by which the erase should have timedout
     *
     * @return                  The operation status.
     * @retval false            if the operation failed.
     * @retval true             if the operation succeeded.
     *
     */
    virtual bool start_mass_erase(uint32_t &delay_ms, uint32_t &timeout_ms) = 0;

    /**
     * @details Sends command to erase a sector of the chip.
     *
     * @param[in]  sector       Sector number to be erased
     * @param[out] delay_ms     Time to wait until next is_device_busy call
     * @param[out] timeout_ms   Time by which the erase should have timedout
     *
     * @return                  The operation status.
     * @retval false            if the operation failed.
     * @retval true             if the operation succeeded.
     *
     */
    virtual bool start_sector_erase(uint32_t sector, uint32_t &delay_ms, uint32_t &timeout_ms) = 0;

    /**
     * @details Tries to erase as much as possible starting from the offset
     *          until size. User needs to call this as many times as needed
     *          taking already erased bytes into account, until desired erase
     *          has taken place
     *
     * @param[in] offset        address offset for erase
     * @param[in] size          size desired to be erased
     * @param[out] erasing      number of bytes erasing
     * @param[out] delay_ms     Time to wait until next is_device_busy call
     * @param[out] timeout_ms   Time by which the erase should have timedout
     *
     * @return                  The operation status.
     * @retval false            if the operation failed.
     * @retval true             if the operation succeeded.
     *
     */
    virtual bool start_erase_offset(uint32_t offset, uint32_t size, uint32_t &erasing,
                                    uint32_t &delay_ms, uint32_t &timeout_ms)
    {
        return false;
    }

    /**
     * @details Check if the device is busy.
     *
     * @return                  device busy with last op.
     *
     * @retval false            if the device is ready.
     * @retval true             if the device is busy.
     *
     */
    virtual bool is_device_busy() = 0;


    /**
     * @details Check if selected sector is erased.
     *
     * @param[in] sector        sector for which to check erase
     * @return                  The operation status.
     * @retval false            if the operation failed.
     * @retval true             if the operation succeeded.
     *
     */
    virtual bool verify_sector_erase(uint32_t sector) = 0;

    /**
     * @details minimum number of bytes that can be erased
     *
     * @return                  Number of bytes.
     *
     */
    virtual uint32_t min_erase_size() const = 0;

    /**
     * @details Starts execution in place mode
     *
     * @return                  if successfully entered XIP mode.
     *
     * @retval false            the device failed to enter XIP mode.
     * @retval true             the device has entered XIP mode.
     *
     */
    virtual bool start_xip_mode(void** addr) { return false; }

    virtual bool stop_xip_mode() { return false; }
};

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
    Implements Common Flash Interface Driver based on
    Open Standard Published by JEDEC
*/

#include "AP_FlashIface_Abstract.h"


class AP_FlashIface_JEDEC : public AP_FlashIface
{
public:
    bool init() override;

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
    bool read(uint32_t offset, uint8_t *data, uint32_t size) override;


    /**
     * @details Gets number bytes that can be written in one go (page size).
     *
     * @return                  page size in bytes.
     *
     */
    uint32_t get_page_size() const override
    {
        return _desc.page_size;
    }

    /**
     * @details Gets number pages, each page can written in one go
     *
     * @return                  Number of pages.
     *
     */
    uint32_t get_page_count() const override
    {
        return _desc.page_count;
    }

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
    bool start_program_page(uint32_t page, const uint8_t *data, uint32_t &delay_us, uint32_t &timeout_us) override;

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
    bool start_program_offset(uint32_t offset, const uint8_t* data, uint32_t size, uint32_t &programming,
                              uint32_t &delay_us, uint32_t &timeout_us) override;

    // Erase Methods

    /**
     * @details Sends command to erase the entire chip.
     *
     * @param[out] delay_ms     Time to wait until next is_device_busy call
     * @param[out] timeout_ms   Time by which the erase should have timedout
     *
     * @return                  The operation status.
     * @retval false            if the operation failed.
     * @retval true             if the operation succeeded.
     *
     */
    bool start_mass_erase(uint32_t &delay_ms, uint32_t &timeout_ms) override;

    /**
     * @details Gets number bytes that can erased in one go(sector size)
     *
     * @return                  Sector size in bytes.
     *
     */
    uint32_t get_sector_size() const override
    {
        return _desc.sector_size;
    }

    /**
     * @details Gets number of sectors, each sector can be erased in one go
     *
     * @return                  Number of sectors.
     *
     */
    uint32_t get_sector_count() const override
    {
        return _desc.sector_count;
    }

    /**
     * @details minimum number of bytes that can be erased
     *
     * @return                  Number of bytes.
     *
     */
    uint32_t min_erase_size() const override
    {
        return _desc.min_erase_size;
    }


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
    bool start_sector_erase(uint32_t sector, uint32_t &delay_ms, uint32_t &timeout_ms) override;

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
    bool start_erase_offset(uint32_t offset, uint32_t size, uint32_t &erasing,
                            uint32_t &delay_ms, uint32_t &timeout_ms) override;


    /**
     * @details Check if selected sector is erased.
     *
     * @param[in] sector        sector for which to check erase
     * @return                  The operation status.
     * @retval false            if the operation failed.
     * @retval true             if the operation succeeded.
     *
     */
    bool verify_sector_erase(uint32_t sector) override;


    /**
     * @details Check if the device is busy.
     *
     * @return                  device busy with last op.
     *
     * @retval false            if the device is ready.
     * @retval true             if the device is busy.
     *
     */
    bool is_device_busy() override;


    /**
     * @details Starts execution in place mode
     *
     * @return                  if successfully entered XIP mode.
     *
     * @retval false            the device failed to enter XIP mode.
     * @retval true             the device has entered XIP mode.
     *
     */
    bool start_xip_mode(void** addr) override;

    bool stop_xip_mode() override;
protected:
    void reset_device();

    // Does initial configuration to bring up and setup chip
    bool detect_device();

    // Configures device to normal working state, currently 4-4-4 QSPI
    bool configure_device();

    // Enables commands that modify flash data or settings
    bool write_enable();

    // Disables commands that modify flash data or settings
    bool write_disable();

    // wait for the chip to be ready for the next instruction
    void wait_ready();

    // Read modify write register
    bool modify_reg(uint8_t read_ins, uint8_t write_ins,
                    uint8_t mask, uint8_t va_list);

    // reads a register value of chip using instruction
    bool read_reg(uint8_t read_ins, uint8_t &read_val);

    // sends instruction to write a register value in the chip
    bool write_reg(uint8_t read_ins, uint8_t write_val);

    // Sends QSPI command without data
    bool send_cmd(uint8_t ins);

    // Is device in quad spi mode
    bool _quad_spi_mode;

    AP_HAL::OwnPtr<AP_HAL::QSPIDevice> _dev;

    enum xip_entry_methods {
        XIP_ENTRY_METHOD_1,
        XIP_ENTRY_METHOD_2,
        XIP_ENTRY_METHOD_3
    };

    // Device description extracted from SFDP
    struct device_desc {
        uint16_t param_rev; //parameter revision
        uint8_t param_table_len; // size of parameter table
        uint32_t param_table_pointer; // location of parameter table
        uint32_t flash_size; // size of flash in bytes
        uint32_t page_size; // maximum size that can be written in one transaction
        uint32_t page_count; // number of pages each of page size
        uint32_t sector_size; // maximum number of bytes that can be erased outside of mass erase
        uint32_t sector_count; // number of sectors
        uint32_t min_erase_size; // minimum amount of bytes that can be erased
        struct {
            uint8_t ins; // instruction for the erase
            uint32_t size; // number of bytes that will be erased
            uint32_t delay_ms; // typical time this command will finish
            uint32_t timeout_ms; // time after which the erase cmd caller should time
        } erase_type[4];
        uint32_t mass_erase_delay_ms; // typical time taken while mass erase
        uint32_t mass_erase_timeout_ms; // time after which mass erase cmd caller should timeout
        uint8_t write_enable_ins; // instruction to allow enabling modification of register and data
        uint32_t page_prog_delay_us; // time taken to write a page worth of data to flash
        uint32_t page_prog_timeout_us; // time after which the page program caller should timeout
        uint8_t fast_read_ins; // instruction to do fast read, i.e. read any number of bytes in single trx
        uint8_t fast_read_dummy_cycles; // number of dummy cycles after which the chip will respond with data
        uint8_t quad_mode_ins; // instruction to enter 4-4-4 mode
        uint8_t quad_mode_enable;
        bool quad_mode_rmw_seq; // use Read modify write sequence to enter 4-4-4 mode supported or not
        uint8_t status_read_ins; // read status of the chip, gets us if busy writing/erasing
        bool legacy_status_polling; // check if legacy status polling supported or not
        bool is_xip_supported; // is execution in place or 0-4-4 mode supported
        uint8_t fast_read_mode_clocks;
        xip_entry_methods entry_method;
    } _desc;

    uint8_t _dev_list_idx;
    bool initialised;
    bool write_enable_called;
};


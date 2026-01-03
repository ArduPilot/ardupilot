/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_Filesystem_FlashMemory_LittleFS.h"
#include <AP_HAL_RP/NAND_PIO_Driver.h>

#if AP_FILESYSTEM_RP2350_ENABLED

class AP_Filesystem_RP2350 : public AP_Filesystem_FlashMemory_LittleFS
{
public:
    int _flashmem_read(lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size) override;
    int _flashmem_prog(lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size) override;
    int _flashmem_erase(lfs_block_t block) override;
    int _flashmem_sync() override;

private:
    // PIO device that handles the NAND flash memory
    RP::NAND_PIO_Driver* dev;

    bool init_flash() override WARN_IF_UNUSED;
    bool write_enable() override WARN_IF_UNUSED;
    bool is_busy() override;
    bool mount_filesystem() override;
    uint32_t find_block_size_and_count() override;
};

#endif  // #if AP_FILESYSTEM_RP2350_ENABLED
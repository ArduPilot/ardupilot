/*
   Please contribute your ideas! See http://dev.ardupilot.org for details

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
/*
  a class to allow for FLASH to be used as a memory backed storage
  backend for any HAL. The basic methodology is to use a log based
  storage system over two flash sectors. Key design elements:

  - erase of sectors only called on init, as erase will lock the flash
    and prevent code execution

  - write using log based system

  - read requires scan of all log elements. This is expected to be called rarely

  - assumes flash that erases to 0xFF and where writing can only clear
    bits, not set them

  - assumes flash sectors are much bigger than storage size. If they
    aren't then caller can aggregate multiple sectors. Designed for
    128k flash sectors with 16k storage size.

  - assumes two flash sectors are available
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

/*
  The StorageManager holds the layout of non-volatile storeage
 */
class AP_FlashStorage {
private:
    static const uint8_t block_size = 8;
    static const uint16_t num_blocks = 2048;
    static const uint8_t max_write = 64;

public:
    // caller provided function to write to a flash sector
    FUNCTOR_TYPEDEF(FlashWrite, bool, uint8_t , uint32_t , const uint8_t *, uint16_t );

    // caller provided function to read from a flash sector. Only called on init()
    FUNCTOR_TYPEDEF(FlashRead, bool, uint8_t , uint32_t , uint8_t *, uint16_t );
    
    // caller provided function to erase a flash sector. Only called from init()
    FUNCTOR_TYPEDEF(FlashErase, bool, uint8_t );

    // caller provided function to indicate if erasing is allowed
    FUNCTOR_TYPEDEF(FlashEraseOK, bool);
    
    // constructor. 
    AP_FlashStorage(uint8_t *mem_buffer,        // buffer of storage_size bytes
                    uint32_t flash_sector_size, // size of each flash sector in bytes
                    FlashWrite flash_write,     // function to write to flash
                    FlashRead flash_read,       // function to read from flash
                    FlashErase flash_erase,     // function to erase flash
                    FlashEraseOK flash_erase_ok); // function to check if erasing allowed

    // initialise storage, filling mem_buffer with current contents
    bool init(void);

    // switch full sector - should only be called when safe to have CPU
    // offline for considerable periods as an erase will be needed
    bool switch_full_sector(void);
    
    // write some data to storage from mem_buffer
    bool write(uint16_t offset, uint16_t length);

    // fixed storage size
    static const uint16_t storage_size = block_size * num_blocks;
    
private:
    uint8_t *mem_buffer;
    const uint32_t flash_sector_size;
    FlashWrite flash_write;
    FlashRead flash_read;
    FlashErase flash_erase;
    FlashEraseOK flash_erase_ok;

    uint8_t current_sector;
    uint32_t write_offset;
    uint32_t reserved_space;
    bool write_error;

    // 24 bit signature
    static const uint32_t signature = 0x51685B;

    // 8 bit sector states
    enum SectorState {
        SECTOR_STATE_AVAILABLE = 0xFF,
        SECTOR_STATE_IN_USE    = 0xFE,
        SECTOR_STATE_FULL      = 0xFC
    };

    // header in first word of each sector
    struct sector_header {
        uint32_t state:8;
        uint32_t signature:24;
    };


    enum BlockState {
        BLOCK_STATE_AVAILABLE = 0x3,
        BLOCK_STATE_WRITING   = 0x1,
        BLOCK_STATE_VALID     = 0x0
    };
    
    // header of each block of data
    struct block_header {
        uint16_t state:2;
        uint16_t block_num:11;
        uint16_t num_blocks_minus_one:3;
    };

    // amount of space needed to write full storage
    static const uint32_t reserve_size = (storage_size / max_write) * (sizeof(block_header) + max_write) + max_write;
        
    // load data from a sector
    bool load_sector(uint8_t sector);

    // erase a sector and write header
    bool erase_sector(uint8_t sector);

    // erase all sectors and reset
    bool erase_all(void);

    // write all of mem_buffer to current sector
    bool write_all(void);

    // return true if all bytes are zero
    bool all_zero(uint16_t ofs, uint16_t size);

    // switch to next sector for writing
    bool switch_sectors(void);
};

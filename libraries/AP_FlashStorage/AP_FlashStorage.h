/*
   Please contribute your ideas! See https://ardupilot.org/dev for details

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
  we support 4 different types of flash which have different restrictions
 */
#define AP_FLASHSTORAGE_TYPE_F1  1 // F1 and F3
#define AP_FLASHSTORAGE_TYPE_F4  2 // F4 and F7
#define AP_FLASHSTORAGE_TYPE_H7  3 // H7
#define AP_FLASHSTORAGE_TYPE_G4  4 // G4

#ifndef AP_FLASHSTORAGE_TYPE
#if defined(STM32F1) || defined(STM32F3)
/*
  the STM32F1 and STM32F3 can't change individual bits from 1 to 0
  unless all bits in the 16 bit word are 1
 */
#define AP_FLASHSTORAGE_TYPE AP_FLASHSTORAGE_TYPE_F1
#elif defined(STM32H7)
/*
  STM32H7 can only write in 32 byte chunks, and must only write when all bits are 1
 */
#define AP_FLASHSTORAGE_TYPE AP_FLASHSTORAGE_TYPE_H7
#elif defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
/*
  STM32G4 can only write in 8 byte chunks, and must only write when all bits are 1
 */
#define AP_FLASHSTORAGE_TYPE AP_FLASHSTORAGE_TYPE_G4
#else // F4, F7
/*
  STM32F4 and STM32F7 can update bits from 1 to 0
 */
#define AP_FLASHSTORAGE_TYPE AP_FLASHSTORAGE_TYPE_F4
#endif
#endif

/*
  The StorageManager holds the layout of non-volatile storage
 */
class AP_FlashStorage {
private:
#if AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_H7
    // need to write in 32 byte chunks, with 2 byte header
    static const uint8_t block_size = 30;
    static const uint8_t max_write = block_size;
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_G4
    // write in 8 byte chunks, with 2 byte header
    static const uint8_t block_size = 6;
    static const uint8_t max_write = block_size;
#else
    static const uint8_t block_size = 8;
    static const uint8_t max_write = 64;
#endif
    static const uint16_t num_blocks = (HAL_STORAGE_SIZE+(block_size-1)) / block_size;

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

    // erase sectors and re-initialise
    bool erase(void) WARN_IF_UNUSED {
        return erase_all();
    }
    
    // re-initialise storage, using current mem_buffer
    bool re_initialise(void) WARN_IF_UNUSED;
    
    // switch full sector - should only be called when safe to have CPU
    // offline for considerable periods as an erase will be needed
    bool switch_full_sector(void) WARN_IF_UNUSED;

    // write some data to storage from mem_buffer
    bool write(uint16_t offset, uint16_t length) WARN_IF_UNUSED;

    // fixed storage size
    static const uint16_t storage_size = HAL_STORAGE_SIZE;
    
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
#if AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_F4
    static const uint32_t signature = 0x51685B;
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_F1
    static const uint32_t signature = 0x51;
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_H7
    static const uint32_t signature = 0x51685B62;
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_G4
    static const uint32_t signature = 0x1586B562;
#else
#error "Unknown AP_FLASHSTORAGE_TYPE"
#endif

    // sector states, representation depends on storage type
    enum SectorState {
        SECTOR_STATE_AVAILABLE = 1,
        SECTOR_STATE_IN_USE    = 2,
        SECTOR_STATE_FULL      = 3,
        SECTOR_STATE_INVALID   = 4
    };

    // header in first word of each sector
    struct sector_header {
#if AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_F4
        uint32_t state1:8;
        uint32_t signature1:24;
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_F1
        uint32_t state1:32;
        uint32_t signature1:16;
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_H7
        // needs to be 96 bytes on H7 to support 3 states
        uint32_t state1;
        uint32_t signature1;
        uint32_t pad1[6];
        uint32_t state2;
        uint32_t signature2;
        uint32_t pad2[6];
        uint32_t state3;
        uint32_t signature3;
        uint32_t pad3[6];
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_G4
        // needs to be 24 bytes on G4 to support 3 states
        uint32_t state1;
        uint32_t signature1;
        uint32_t state2;
        uint32_t signature2;
        uint32_t state3;
        uint32_t signature3;
#endif
        bool signature_ok(void) const;
        SectorState get_state() const;
        void set_state(SectorState state);
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
    bool load_sector(uint8_t sector) WARN_IF_UNUSED;

    // erase a sector and write header
    bool erase_sector(uint8_t sector, bool mark_available) WARN_IF_UNUSED;

    // erase all sectors and reset
    bool erase_all() WARN_IF_UNUSED;

    // write all of mem_buffer to current sector
    bool write_all() WARN_IF_UNUSED;

    // return true if all bytes are zero
    bool all_zero(uint16_t ofs, uint16_t size) WARN_IF_UNUSED;

    // switch to next sector for writing
    bool switch_sectors(void) WARN_IF_UNUSED;

    // _switch_full_sector is protected by switch_full_sector to avoid
    // an infinite recursion problem; switch_full_sector calls
    // write() which can call switch_full_sector.  This has been seen
    // in practice.
    bool protected_switch_full_sector(void) WARN_IF_UNUSED;
    bool in_switch_full_sector;
};

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

#include <AP_HAL/AP_HAL.h>
#include <AP_FlashStorage/AP_FlashStorage.h>
#include <stdio.h>

#define FLASHSTORAGE_DEBUG 0

#if FLASHSTORAGE_DEBUG
#define debug(fmt, args...)  do { printf(fmt, ##args); } while(0)
#else
#define debug(fmt, args...)  do { } while(0)
#endif

// constructor.
AP_FlashStorage::AP_FlashStorage(uint8_t *_mem_buffer,
                                 uint32_t _flash_sector_size,
                                 FlashWrite _flash_write,
                                 FlashRead _flash_read,
                                 FlashErase _flash_erase,
                                 FlashEraseOK _flash_erase_ok) :
    mem_buffer(_mem_buffer),
    flash_sector_size(_flash_sector_size),
    flash_write(_flash_write),
    flash_read(_flash_read),
    flash_erase(_flash_erase),
    flash_erase_ok(_flash_erase_ok) {}

// initialise storage
bool AP_FlashStorage::init(void)
{
    debug("running init()\n");
    
    // start with empty memory buffer
    memset(mem_buffer, 0, storage_size);

    // find state of sectors
    struct sector_header header[2];

    // read headers and possibly initialise if bad signature
    for (uint8_t i=0; i<2; i++) {
        if (!flash_read(i, 0, (uint8_t *)&header[i], sizeof(header[i]))) {
            return false;
        }
        bool bad_header = (header[i].signature != signature);
        enum SectorState state = (enum SectorState)header[i].state;
        if (state != SECTOR_STATE_AVAILABLE &&
            state != SECTOR_STATE_IN_USE &&
            state != SECTOR_STATE_FULL) {
            bad_header = true;
        }

        // initialise if bad header
        if (bad_header) {
            return erase_all();
        }
    }

    // work out the first sector to read from using sector states
    enum SectorState states[2] {(enum SectorState)header[0].state, (enum SectorState)header[1].state};
    uint8_t first_sector;

    if (states[0] == states[1]) {
        if (states[0] != SECTOR_STATE_AVAILABLE) {
            return erase_all();
        }
        first_sector = 0;
    } else if (states[0] == SECTOR_STATE_FULL) {
        first_sector = 0;
    } else if (states[1] == SECTOR_STATE_FULL) {
        first_sector = 1;
    } else if (states[0] == SECTOR_STATE_IN_USE) {
        first_sector = 0;
    } else if (states[1] == SECTOR_STATE_IN_USE) {
        first_sector = 1;
    } else {
        // doesn't matter which is first
        first_sector = 0;
    }

    // load data from any current sectors
    for (uint8_t i=0; i<2; i++) {
        uint8_t sector = (first_sector + i) & 1;
        if (states[sector] == SECTOR_STATE_IN_USE ||
            states[sector] == SECTOR_STATE_FULL) {
            if (!load_sector(sector)) {
                return erase_all();
            }
        }
    }

    // clear any write error
    write_error = false;
    reserved_space = 0;
    
    // if the first sector is full then write out all data so we can erase it
    if (states[first_sector] == SECTOR_STATE_FULL) {
        current_sector = first_sector ^ 1;
        if (!write_all()) {
            return erase_all();
        }
    }

    // erase any sectors marked full
    for (uint8_t i=0; i<2; i++) {
        if (states[i] == SECTOR_STATE_FULL) {
            if (!erase_sector(i)) {
                return false;
            }
        }
    }

    reserved_space = 0;
    
    // ready to use
    return true;
}



// switch full sector - should only be called when safe to have CPU
// offline for considerable periods as an erase will be needed
bool AP_FlashStorage::switch_full_sector(void)
{
    debug("running switch_full_sector()\n");
    
    // clear any write error
    write_error = false;
    reserved_space = 0;
    
    if (!write_all()) {
        return false;
    }

    if (!erase_sector(current_sector ^ 1)) {
        return false;
    }

    return switch_sectors();
}

// write some data to virtual EEPROM
bool AP_FlashStorage::write(uint16_t offset, uint16_t length)
{
    if (write_error) {
        return false;
    }
    //debug("write at %u for %u write_offset=%u\n", offset, length, write_offset);
    
    while (length > 0) {
        uint8_t n = max_write;
        if (length < n) {
            n = length;
        }

        if (write_offset > flash_sector_size - (sizeof(struct block_header) + max_write + reserved_space)) {
            if (!switch_sectors()) {
                if (!flash_erase_ok()) {
                    return false;
                }
                if (!switch_full_sector()) {
                    return false;                    
                }
            }
        }
        
        struct block_header header;
        header.state = BLOCK_STATE_WRITING;
        header.block_num = offset / block_size;
        header.num_blocks_minus_one = ((n + (block_size - 1)) / block_size)-1;
        uint16_t block_ofs = header.block_num*block_size;
        uint16_t block_nbytes = (header.num_blocks_minus_one+1)*block_size;
        
        if (!flash_write(current_sector, write_offset, (uint8_t*)&header, sizeof(header))) {
            return false;
        }
        if (!flash_write(current_sector, write_offset+sizeof(header), &mem_buffer[block_ofs], block_nbytes)) {
            return false;
        }
        header.state = BLOCK_STATE_VALID;
        if (!flash_write(current_sector, write_offset, (uint8_t*)&header, sizeof(header))) {
            return false;
        }
        write_offset += sizeof(header) + block_nbytes;

        uint8_t n2 = block_nbytes - (offset % block_size);
        //debug("write_block at %u for %u n2=%u\n", block_ofs, block_nbytes, n2);
        if (n2 > length) {
            break;
        }
        offset += n2;
        length -= n2;
    }
    
    // handle wrap to next sector
    // write data
    // write header word
    return true;
}

/*
  load all data from a flash sector into mem_buffer
 */
bool AP_FlashStorage::load_sector(uint8_t sector)
{
    uint32_t ofs = sizeof(sector_header);
    while (ofs < flash_sector_size - sizeof(struct block_header)) {
        struct block_header header;
        if (!flash_read(sector, ofs, (uint8_t *)&header, sizeof(header))) {
            return false;
        }
        enum BlockState state = (enum BlockState)header.state;

        switch (state) {
        case BLOCK_STATE_AVAILABLE:
            // we've reached the end
            write_offset = ofs;
            return true;

        case BLOCK_STATE_WRITING: {
            /*
              we were interrupted while writing a block. We can't
              re-use the data in this block as it may have some bits
              that are not set to 1, so by flash rules can't be set to
              an arbitrary value. So we skip over this block, leaving
              a gap. The gap size is limited to (7+1)*8=64 bytes. That
              gap won't be recovered until we next do an erase of this
              sector
             */
            uint16_t block_nbytes = (header.num_blocks_minus_one+1)*block_size;
            ofs += block_nbytes + sizeof(header);
            break;
        }
            
        case BLOCK_STATE_VALID: {
            uint16_t block_nbytes = (header.num_blocks_minus_one+1)*block_size;
            uint16_t block_ofs = header.block_num*block_size;
            if (block_ofs + block_nbytes > storage_size) {
                // the data is invalid (out of range)
                return false;
            }
            if (!flash_read(sector, ofs+sizeof(header), &mem_buffer[block_ofs], block_nbytes)) {
                return false;
            }
            //debug("read at %u for %u\n", block_ofs, block_nbytes);
            ofs += block_nbytes + sizeof(header);
            break;
        }
        default:
            // invalid state
            return false;
        }
    }
    write_offset = ofs;
    return true;
}

/*
  erase one sector
 */
bool AP_FlashStorage::erase_sector(uint8_t sector)
{
    if (!flash_erase(sector)) {
        return false;
    }

    struct sector_header header;
    header.signature = signature;
    header.state = SECTOR_STATE_AVAILABLE;
    return flash_write(sector, 0, (const uint8_t *)&header, sizeof(header));
}

/*
  erase both sectors
 */
bool AP_FlashStorage::erase_all(void)
{
    write_error = false;

    current_sector = 0;
    write_offset = sizeof(struct sector_header);
    
    if (!erase_sector(0) || !erase_sector(1)) {
        return false;
    }
    
    // mark current sector as in-use
    struct sector_header header;
    header.signature = signature;
    header.state = SECTOR_STATE_IN_USE;
    return flash_write(current_sector, 0, (const uint8_t *)&header, sizeof(header));    
}

/*
  write all of mem_buffer to current sector
 */
bool AP_FlashStorage::write_all(void)
{
    debug("write_all to sector %u at %u with reserved_space=%u\n",
           current_sector, write_offset, reserved_space);
    for (uint16_t ofs=0; ofs<storage_size; ofs += max_write) {
        if (!all_zero(ofs, max_write)) {
            if (!write(ofs, max_write)) {
                return false;
            }
        }
    }
    return true;
}

// return true if all bytes are zero
bool AP_FlashStorage::all_zero(uint16_t ofs, uint16_t size)
{
    while (size--) {
        if (mem_buffer[ofs++] != 0) {
            return false;
        }
    }
    return true;
}

// switch to next sector for writing
bool AP_FlashStorage::switch_sectors(void)
{
    if (reserved_space != 0) {
        // other sector is already full
        debug("both sectors are full\n");
        return false;
    }

    struct sector_header header;
    header.signature = signature;

    uint8_t new_sector = current_sector ^ 1;
    debug("switching to sector %u\n", new_sector);
    
    // check sector is available
    if (!flash_read(new_sector, 0, (uint8_t *)&header, sizeof(header))) {
        return false;
    }
    if (header.signature != signature) {
        write_error = true;
        return false;
    }
    if (SECTOR_STATE_AVAILABLE != (enum SectorState)header.state) {
        write_error = true;
        debug("both sectors full\n");
        return false;
    }

    // mark current sector as full. This needs to be done before we
    // mark the new sector as in-use so that a power failure between
    // the two steps doesn't leave us with an erase on the
    // reboot. Thanks to night-ghost for spotting this.
    header.state = SECTOR_STATE_FULL;
    if (!flash_write(current_sector, 0, (const uint8_t *)&header, sizeof(header))) {
        return false;
    }

    // mark new sector as in-use
    header.state = SECTOR_STATE_IN_USE;
    if (!flash_write(new_sector, 0, (const uint8_t *)&header, sizeof(header))) {
        return false;
    }

    // switch sectors
    current_sector = new_sector;
        
    // we need to reserve some space in next sector to ensure we can successfully do a
    // full write out on init()
    reserved_space = reserve_size;
    
    write_offset = sizeof(header);
    return true;    
}

/*
  re-initialise, using current mem_buffer
 */
bool AP_FlashStorage::re_initialise(void)
{
    if (!flash_erase_ok()) {
        return false;
    }
    if (!erase_all()) {
        return false;        
    }
    return write_all();
}

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
/*
  Flymaple port by Mike McCauley
  Partly based on EEPROM.*, flash_stm* copied from AeroQuad_v3.2
  This uses n*2*2k pages of FLASH ROM to emulate an EEPROM
  This storage is retained after power down, and survives reloading of firmware
  All multi-byte accesses are reduced to single byte access so that can span EEPROM block boundaries
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include <string.h>
#include "Storage.h"
#include "FlymapleWirish.h"
#include "utility/EEPROM.h"

using namespace AP_HAL_FLYMAPLE_NS;

extern const AP_HAL::HAL& hal;

// The EEPROM class uses 2x2k FLASH ROM pages to emulate 1k of EEPROM.
// AP needs at last 4k of EEPROM, so we define multiple EEPROMClass, one for each
// 1k of EEPROM address space

// This is the number of 1k EEPROM blocks we need
const uint8_t num_eeprom_blocks = 4;

// This is the addres of the LAST 2k FLASH ROM page to be used to implement the EEPROM
// FLASH ROM page use grows downwards from here
// It is in fact the lat page in the available FLASH ROM
const uint32_t last_flash_page = 0x0807F800;

// This is the size of each FLASH ROM page
const uint32 pageSize  = 0x800;

// This defines the base addresses of the 2 FLASH ROM pages that will be used to emulate EEPROM
// These are the last 2 2k pages in the FLASH ROM address space on the RET6 used by Flymaple
// This will effectively provide a total of 1kb of emulated EEPROM storage
const uint32 pageBase0 = 0x0807F000;
const uint32 pageBase1 = 0x0807F800;

static EEPROMClass eeprom[num_eeprom_blocks];

FLYMAPLEStorage::FLYMAPLEStorage()
{}

void FLYMAPLEStorage::init(void*)
{
    for (int i = 0; i < num_eeprom_blocks; i++)
    {
	uint16 result = eeprom[i].init(last_flash_page - (2*i * pageSize), 
				       last_flash_page - (((2*i)+1) * pageSize), 
				       pageSize);
	if (result != EEPROM_OK)
	    hal.console->printf("FLYMAPLEStorage::init eeprom.init[%d] failed: %x\n", i, result);
    }
}

uint8_t FLYMAPLEStorage::read_byte(uint16_t loc){
//hal.console->printf("read_byte %d\n", loc);
    uint16_t eeprom_index = loc >> 10;
    uint16_t eeprom_offset = loc & 0x3ff;
    if (eeprom_index >= num_eeprom_blocks)
    {
	hal.console->printf("FLYMAPLEStorage::read_byte loc %d out of range\n", loc);
	return 0xff; // What else?
    }

    // 'bytes' are packed 2 per word
    // Read existing dataword and change upper or lower byte
    uint16_t data = eeprom[eeprom_index].read(eeprom_offset >> 1);
    if (eeprom_offset & 1)
	return data >> 8; // Odd, upper byte
    else
	return data & 0xff; // Even lower byte
}

void FLYMAPLEStorage::read_block(void* dst, uint16_t src, size_t n) {
//    hal.console->printf("read_block %d %d\n", src, n);
    // Treat as a block of bytes
    for (size_t i = 0; i < n; i++)
	((uint8_t*)dst)[i] = read_byte(src+i);
}

void FLYMAPLEStorage::write_byte(uint16_t loc, uint8_t value)
{
//    hal.console->printf("write_byte %d, %d\n", loc, value);

    uint16_t eeprom_index = loc >> 10;
    uint16_t eeprom_offset = loc & 0x3ff;
    if (eeprom_index >= num_eeprom_blocks)
    {
	hal.console->printf("FLYMAPLEStorage::write_byte loc %d out of range\n", loc);
	return;
    }
    
    // 'bytes' are packed 2 per word
    // Read existing data word and change upper or lower byte
    uint16_t data = eeprom[eeprom_index].read(eeprom_offset >> 1);
    if (eeprom_offset & 1)
	data = (data & 0x00ff) | (value << 8); // Odd, upper byte
    else
	data = (data & 0xff00) | value; // Even, lower byte
    eeprom[eeprom_index].write(eeprom_offset >> 1, data);
}

void FLYMAPLEStorage::write_block(uint16_t loc, const void* src, size_t n)
{
//    hal.console->printf("write_block %d, %d\n", loc, n);
    // Treat as a block of bytes
    for (size_t i = 0; i < n; i++)
	write_byte(loc+i, ((uint8_t*)src)[i]);
}

#endif

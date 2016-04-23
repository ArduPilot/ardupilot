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
  driver modified for VRbrain board
  Partly based on EEPROM.*, flash_stm* copied from AeroQuad_v3.2
  This uses 2*16k pages of FLASH ROM to emulate an EEPROM
  This storage is retained after power down, and survives reloading of firmware
  All multi-byte accesses are reduced to single byte access so that can span EEPROM block boundaries
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI

#include <string.h>
#include "Storage.h"
#include "EEPROM.h"

using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

// The EEPROM class uses 2x16k FLASH ROM pages to emulate 4k of EEPROM.

// This is the addres of the 16k FLASH ROM page to be used to implement the EEPROM
// FLASH ROM page use grows downwards from here
// It is in fact the lat page in the available FLASH ROM
const uint32_t last_flash_page = 0x0800c000;

// This is the size of each FLASH ROM page
const uint32 pageSize  = 0x1000;

// This defines the base addresses of the 2 FLASH ROM pages that will be used to emulate EEPROM
// These are the 2 16k pages in the FLASH ROM address space on the STM32F4 used by VRBrain
// This will effectively provide a total of 4kb of emulated EEPROM storage
const uint32 pageBase0 = 0x08008000; // Page2
const uint32 pageBase1 = 0x0800c000; // Page3

static EEPROMClass eeprom;

REVOMINIStorage::REVOMINIStorage()
{}

void REVOMINIStorage::init()
{
    eeprom.init(0x0800c000, 0x08008000, pageSize);
}

uint8_t REVOMINIStorage::read_byte(uint16_t loc){

    // 'bytes' are packed 2 per word
    // Read existing dataword and change upper or lower byte
    //uint16_t data = eeprom.read(eeprom_offset);
    uint16_t data = eeprom.read(loc >> 1);
    if (loc & 1)
	return data >> 8; // Odd, upper byte
    else
	return data & 0xff; // Even lower byte
}

uint16_t REVOMINIStorage::read_word(uint16_t loc){
    uint16_t value;
    read_block(&value, loc, sizeof(value));
    return value;
}

uint32_t REVOMINIStorage::read_dword(uint16_t loc){
    uint32_t value;
    read_block(&value, loc, sizeof(value));
    return value;
}

void REVOMINIStorage::read_block(void* dst, uint16_t src, size_t n) {
    // Treat as a block of bytes
    for (size_t i = 0; i < n; i++)
	((uint8_t*)dst)[i] = read_byte(src+i);
}

void REVOMINIStorage::write_byte(uint16_t loc, uint8_t value)
{
    // 'bytes' are packed 2 per word
    // Read existing data word and change upper or lower byte
    uint16_t data = eeprom.read(loc >> 1);

    if (loc & 1)
	data = (data & 0x00ff) | (value << 8); // Odd, upper byte
    else
	data = (data & 0xff00) | value; // Even, lower byte
    eeprom.write(loc >> 1, data);
}

void REVOMINIStorage::write_word(uint16_t loc, uint16_t value)
{
    write_block(loc, &value, sizeof(value));
}

void REVOMINIStorage::write_dword(uint16_t loc, uint32_t value)
{
    write_block(loc, &value, sizeof(value));
}

void REVOMINIStorage::write_block(uint16_t loc, const void* src, size_t n)
{
    // Treat as a block of bytes
    for (size_t i = 0; i < n; i++)
	write_byte(loc+i, ((uint8_t*)src)[i]);
}

void REVOMINIStorage::format_eeprom(void)
{
    eeprom.format();
}

#endif

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
 */
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include <string.h>
#include "Storage.h"
#include "FlymapleWirish.h"
// EEPROM.*, flash_stm* copied from AeroQuad_v3.2
#include "utility/EEPROM.h"

using namespace AP_HAL_FLYMAPLE_NS;

static EEPROMClass eeprom;

FLYMAPLEStorage::FLYMAPLEStorage()
{}

void FLYMAPLEStorage::init(void*)
{
    eeprom.init(0x801F000, 0x801F800, 0x800);
}

uint8_t FLYMAPLEStorage::read_byte(uint16_t loc){
    // 'bytes' are packed 2 per word
    // Read existing dataword and change upper or lower byte
    uint16_t data = eeprom.read(loc >> 1);
    if (loc & 1)
	return data >> 8;
    else
	return data & 0xff;
}

uint16_t FLYMAPLEStorage::read_word(uint16_t loc){
    return eeprom.read(loc);
}

uint32_t FLYMAPLEStorage::read_dword(uint16_t loc){
    loc <<= 1;
    uint32_t data = eeprom.read(loc);
    data |= (eeprom.read(loc+1) << 16); // second word is the high word
    return data;
}

void FLYMAPLEStorage::read_block(void* dst, uint16_t src, size_t n) {
    // Treat as a block of bytes
    for (size_t i = 0; i < n; i++)
	((uint8_t*)dst)[i] = read_byte(src+i);
}

void FLYMAPLEStorage::write_byte(uint16_t loc, uint8_t value)
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

void FLYMAPLEStorage::write_word(uint16_t loc, uint16_t value)
{
    eeprom.write(loc, value);
}

void FLYMAPLEStorage::write_dword(uint16_t loc, uint32_t value)
{
    loc <<= 1;
    eeprom.write(loc, value & 0xffff);
    eeprom.write(loc+1, value >> 16);
}

void FLYMAPLEStorage::write_block(uint16_t loc, const void* src, size_t n)
{
    // Treat as a block of bytes
    for (size_t i = 0; i < n; i++)
	write_byte(loc+i, ((uint8_t*)src)[i]);
}

#endif

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include <string.h>
#include "Storage.h"
#include "utility/EEPROM.h"

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

// The EEPROM class uses 2x2k FLASH ROM pages to emulate 1k of EEPROM.
// AP needs at last 4k of EEPROM, so we define multiple EEPROMClass, one for each
// 1k of EEPROM address space

// This is the number of 1k EEPROM blocks we need
const uint8_t num_eeprom_blocks = 4;

// This is the address of the LAST 2k FLASH ROM page to be used to implement the EEPROM
// FLASH ROM page use grows downwards from here
// It is in fact the last page in the available FLASH ROM
const uint32_t last_flash_page = 0x0803F800;

// This is the size of each FLASH ROM page
const uint32_t pageSize  = 0x800;

// This defines the base addresses of the 2 FLASH ROM pages that will be used to emulate EEPROM
// These are the last 2 2k pages in the FLASH ROM address space on the STM32F37xRC used by YUNEEC
// This will effectively provide a total of 1kb of emulated EEPROM storage
const uint32_t pageBase0 = 0x0803F000;
const uint32_t pageBase1 = 0x0803F800;

static EEPROMClass eeprom[num_eeprom_blocks];

YUNEECStorage::YUNEECStorage()
{}

void YUNEECStorage::init(void*)
{
    for (int i = 0; i < num_eeprom_blocks; i++) {
    	uint16_t result = eeprom[i].init(last_flash_page - (2*i * pageSize),
				       	   last_flash_page - (((2*i)+1) * pageSize),
				       	   pageSize);
    	if (result != EEPROM_OK)
    		hal.console->printf("YUNEECStorage::init eeprom.init[%d] failed: %x\n", i, result);
    }
}

uint8_t YUNEECStorage::read_byte(uint16_t loc){
//hal.console->printf("read_byte %d\n", loc);
    uint16_t eeprom_index = loc >> 10;
    uint16_t eeprom_offset = loc & 0x3ff;
    if (eeprom_index >= num_eeprom_blocks) {
		hal.console->printf("YUNEECStorage::read_byte loc %d out of range\n", loc);
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

uint16_t YUNEECStorage::read_word(uint16_t loc){
//hal.console->printf("read_word %d\n", loc);
    uint16_t value;
    read_block(&value, loc, sizeof(value));
    return value;
}

uint32_t YUNEECStorage::read_dword(uint16_t loc){
//hal.console->printf("read_dword %d\n", loc);
    uint32_t value;
    read_block(&value, loc, sizeof(value));
    return value;
}

void YUNEECStorage::read_block(void* dst, uint16_t src, size_t n) {
//    hal.console->printf("read_block %d %d\n", src, n);
    // Treat as a block of bytes
    for (size_t i = 0; i < n; i++)
    	((uint8_t*)dst)[i] = read_byte(src+i);
}

void YUNEECStorage::write_byte(uint16_t loc, uint8_t value)
{
//    hal.console->printf("write_byte %d, %d\n", loc, value);

    uint16_t eeprom_index = loc >> 10;
    uint16_t eeprom_offset = loc & 0x3ff;
    if (eeprom_index >= num_eeprom_blocks) {
		hal.console->printf("YUNEECStorage::write_byte loc %d out of range\n", loc);
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

void YUNEECStorage::write_word(uint16_t loc, uint16_t value)
{
//    hal.console->printf("write_word %d, %d\n", loc, value);
    write_block(loc, &value, sizeof(value));
}

void YUNEECStorage::write_dword(uint16_t loc, uint32_t value)
{
//    hal.console->printf("write_dword %d, %d\n", loc, value);
    write_block(loc, &value, sizeof(value));
}

void YUNEECStorage::write_block(uint16_t loc, const void* src, size_t n)
{
//    hal.console->printf("write_block %d, %d\n", loc, n);
    // Treat as a block of bytes
    for (size_t i = 0; i < n; i++)
    	write_byte(loc+i, ((uint8_t*)src)[i]);
}

#endif

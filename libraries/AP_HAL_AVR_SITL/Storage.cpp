#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "Storage.h"
using namespace AVR_SITL;

void SITLEEPROMStorage::_eeprom_open(void)
{
	if (_eeprom_fd == -1) {
		_eeprom_fd = open("eeprom.bin", O_RDWR|O_CREAT, 0777);
		assert(ftruncate(_eeprom_fd, 4096) == 0);
	}
}

uint8_t SITLEEPROMStorage::read_byte(uint16_t loc) 
{
	uint8_t value;
	assert(loc < 4096);
	_eeprom_open();
	assert(pread(_eeprom_fd, &value, 1, loc) == 1);
	return value;
}

uint16_t SITLEEPROMStorage::read_word(uint16_t loc) 
{
	uint16_t value;
	assert(loc < 4096);
	_eeprom_open();
	assert(pread(_eeprom_fd, &value, 2, loc) == 2);
	return value;
}

uint32_t SITLEEPROMStorage::read_dword(uint16_t loc) 
{
	uint32_t value;
	assert(loc < 4096);
	_eeprom_open();
	assert(pread(_eeprom_fd, &value, 4, loc) == 4);
	return value;
}

void SITLEEPROMStorage::read_block(void *dst, uint16_t src, size_t n) 
{
	assert(src < 4096 && src + n < 4096);
	_eeprom_open();
	assert(pread(_eeprom_fd, dst, n, src) == (ssize_t)n);
}

void SITLEEPROMStorage::write_byte(uint16_t loc, uint8_t value) 
{
	assert(loc < 4096);
	_eeprom_open();
	assert(pwrite(_eeprom_fd, &value, 1, loc) == 1);
}

void SITLEEPROMStorage::write_word(uint16_t loc, uint16_t value) 
{
	assert(loc < 4096);
	_eeprom_open();
	assert(pwrite(_eeprom_fd, &value, 2, loc) == 2);
}

void SITLEEPROMStorage::write_dword(uint16_t loc, uint32_t value) 
{
	assert(loc < 4096);
	_eeprom_open();
	assert(pwrite(_eeprom_fd, &value, 4, loc) == 4);
}

void SITLEEPROMStorage::write_block(uint16_t dst, void *src, size_t n) 
{
	assert(dst < 4096);
	_eeprom_open();
	assert(pwrite(_eeprom_fd, src, n, dst) == (ssize_t)n);
}

#endif

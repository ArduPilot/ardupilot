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
		assert(ftruncate(_eeprom_fd, HAL_STORAGE_SIZE) == 0);
	}
}

void SITLEEPROMStorage::read_block(void *dst, uint16_t src, size_t n) 
{
	assert(src < HAL_STORAGE_SIZE && src + n <= HAL_STORAGE_SIZE);
	_eeprom_open();
	assert(pread(_eeprom_fd, dst, n, src) == (ssize_t)n);
}

void SITLEEPROMStorage::write_block(uint16_t dst, const void *src, size_t n) 
{
	assert(dst < HAL_STORAGE_SIZE);
	_eeprom_open();
	assert(pwrite(_eeprom_fd, src, n, dst) == (ssize_t)n);
}

#endif

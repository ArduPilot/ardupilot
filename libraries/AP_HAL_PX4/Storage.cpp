#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "Storage.h"
using namespace PX4;

#define EEPROM_SIZE 4096
#define EEPROM_FILE "/fs/microsd/apm-eeprom"

extern const AP_HAL::HAL& hal;

/* NuttX doesn't have pread/pwrite */
static ssize_t pread(int fd, void *buf, size_t count, off_t ofs)
{
	lseek(fd, ofs, SEEK_SET);
	return read(fd, buf, count);
}

static ssize_t pwrite(int fd, const void *buf, size_t count, off_t ofs)
{
	lseek(fd, ofs, SEEK_SET);
	return write(fd, buf, count);
}

void PX4EEPROMStorage::_eeprom_open(void)
{
	if (_eeprom_fd == -1) {
		_eeprom_fd = open(EEPROM_FILE, O_RDWR|O_CREAT, 0777);
		if (_eeprom_fd == -1) {
			hal.scheduler->panic("Failed to open " EEPROM_FILE);
		}
		if (lseek(_eeprom_fd, 0, SEEK_END) < EEPROM_SIZE) {
			char c = 0;
			assert(pwrite(_eeprom_fd, &c, 1, EEPROM_SIZE-1) == 1);
		}
	}
}

uint8_t PX4EEPROMStorage::read_byte(uint16_t loc) 
{
	uint8_t value;
	_eeprom_open();
	assert(pread(_eeprom_fd, &value, 1, loc) == 1);
	return value;
}

uint16_t PX4EEPROMStorage::read_word(uint16_t loc) 
{
	uint16_t value;
	assert(loc < EEPROM_SIZE);
	_eeprom_open();
	assert(pread(_eeprom_fd, &value, 2, loc) == 2);
	return value;
}

uint32_t PX4EEPROMStorage::read_dword(uint16_t loc) 
{
	uint32_t value;
	assert(loc < EEPROM_SIZE);
	_eeprom_open();
	assert(pread(_eeprom_fd, &value, 4, loc) == 4);
	return value;
}

void PX4EEPROMStorage::read_block(void *dst, uint16_t src, size_t n) 
{
	assert(src < EEPROM_SIZE && src + n < EEPROM_SIZE);
	_eeprom_open();
	assert(pread(_eeprom_fd, dst, n, src) == (ssize_t)n);
}

void PX4EEPROMStorage::write_byte(uint16_t loc, uint8_t value) 
{
	assert(loc < EEPROM_SIZE);
	_eeprom_open();
	assert(pwrite(_eeprom_fd, &value, 1, loc) == 1);
}

void PX4EEPROMStorage::write_word(uint16_t loc, uint16_t value) 
{
	assert(loc < EEPROM_SIZE);
	_eeprom_open();
	assert(pwrite(_eeprom_fd, &value, 2, loc) == 2);
}

void PX4EEPROMStorage::write_dword(uint16_t loc, uint32_t value) 
{
	assert(loc < EEPROM_SIZE);
	_eeprom_open();
	assert(pwrite(_eeprom_fd, &value, 4, loc) == 4);
}

void PX4EEPROMStorage::write_block(uint16_t dst, void *src, size_t n) 
{
	assert(dst < EEPROM_SIZE);
	_eeprom_open();
	assert(pwrite(_eeprom_fd, src, n, dst) == (ssize_t)n);
}

#endif

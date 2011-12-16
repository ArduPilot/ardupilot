#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <stdint.h>

static int eeprom_fd;

static void eeprom_open(void)
{
	if (eeprom_fd == 0) {
		eeprom_fd = open("eeprom.bin", O_RDWR|O_CREAT, 0777);
		ftruncate(eeprom_fd, 4096);
	}
}

void eeprom_write_byte(uint8_t *p, uint8_t value)
{
	intptr_t ofs = (intptr_t)p;
	assert(ofs < 4096);
	eeprom_open();
	pwrite(eeprom_fd, &value, 1, ofs);
}


void eeprom_write_word(uint16_t *p, uint16_t value)
{
	intptr_t ofs = (intptr_t)p;
	assert(ofs < 4096);
	eeprom_open();
	pwrite(eeprom_fd, &value, 2, ofs);
}

void eeprom_write_dword(uint32_t *p, uint32_t value)
{
	intptr_t ofs = (intptr_t)p;
	assert(ofs < 4096);
	eeprom_open();
	pwrite(eeprom_fd, &value, 4, ofs);
}

uint8_t eeprom_read_byte(const uint8_t *p)
{
	intptr_t ofs = (intptr_t)p;
	uint8_t value;
	assert(ofs < 4096);
	eeprom_open();
	pread(eeprom_fd, &value, 1, ofs);
	return value;
}

uint16_t eeprom_read_word(const uint16_t *p)
{
	intptr_t ofs = (intptr_t)p;
	uint16_t value;
	assert(ofs < 4096);
	eeprom_open();
	pread(eeprom_fd, &value, 2, ofs);
	return value;
}

uint32_t eeprom_read_dword(const uint32_t *p)
{
	intptr_t ofs = (intptr_t)p;
	uint32_t value;
	assert(ofs < 4096);
	eeprom_open();
	pread(eeprom_fd, &value, 4, ofs);
	return value;
}

void eeprom_read_block(void *buf, void *ptr, uint8_t size)
{
	intptr_t ofs = (intptr_t)ptr;
	assert(ofs < 4096);
	eeprom_open();
	pread(eeprom_fd, buf, size, ofs);
}

void eeprom_write_block(const void *buf, void *ptr, uint8_t size)
{
	intptr_t ofs = (intptr_t)ptr;
	assert(ofs < 4096);
	eeprom_open();
	pwrite(eeprom_fd, buf, size, ofs);
}

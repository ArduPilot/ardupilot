#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "Storage.h"
using namespace HALSITL;

void EEPROMStorage::_eeprom_open(void)
{
    if (_eeprom_fd == -1) {
        _eeprom_fd = open("eeprom.bin", O_RDWR|O_CREAT|O_CLOEXEC, 0777);
        assert(ftruncate(_eeprom_fd, HAL_STORAGE_SIZE) == 0);
    }
}

void EEPROMStorage::read_block(void *dst, uint16_t src, size_t n)
{
    assert(src < HAL_STORAGE_SIZE && src + n <= HAL_STORAGE_SIZE);
    _eeprom_open();
    assert(pread(_eeprom_fd, dst, n, src) == (ssize_t)n);
}

void EEPROMStorage::write_block(uint16_t dst, const void *src, size_t n)
{
    assert(dst < HAL_STORAGE_SIZE);
    _eeprom_open();
    assert(pwrite(_eeprom_fd, src, n, dst) == (ssize_t)n);
}

#endif

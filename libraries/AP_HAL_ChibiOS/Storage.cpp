/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "Storage.h"
#include "hwdef/common/flash.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include "sdcard.h"
#include <stdio.h>

using namespace ChibiOS;

#ifndef HAL_USE_EMPTY_STORAGE

extern const AP_HAL::HAL& hal;

#ifndef HAL_STORAGE_FILE
// using SKETCHNAME allows the one microSD to be used
// for multiple vehicle types
#define HAL_STORAGE_FILE "/APM/" SKETCHNAME ".stg"
#endif

#ifndef HAL_STORAGE_BACKUP_FILE
// location of backup file
#define HAL_STORAGE_BACKUP_FILE "/APM/" SKETCHNAME ".bak"
#endif

#define STORAGE_FLASH_RETRIES 5

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }

#ifdef USE_POSIX
    // if we have failed filesystem init don't try again
    if (log_fd == -1) {
        return;
    }
#endif

    _dirty_mask.clearall();

#if HAL_WITH_RAMTRON
    using_fram = fram.init();
    if (using_fram) {
        if (!fram.read(0, _buffer, CH_STORAGE_SIZE)) {
            return;
        }
        _save_backup();
        _initialised = true;
        return;
    }
    // allow for FMUv3 with no FRAM chip, fall through to flash storage
#endif

#ifdef STORAGE_FLASH_PAGE
    // load from storage backend
    _flash_load();
#elif defined(USE_POSIX)
    // allow for fallback to microSD based storage
    sdcard_retry();

    log_fd = AP::FS().open(HAL_STORAGE_FILE, O_RDWR|O_CREAT);
    if (log_fd == -1) {
        ::printf("open failed of " HAL_STORAGE_FILE "\n");
        return;
    }
    int ret = AP::FS().read(log_fd, _buffer, CH_STORAGE_SIZE);
    if (ret < 0) {
        ::printf("read failed for " HAL_STORAGE_FILE "\n");
        AP::FS().close(log_fd);
        log_fd = -1;
        return;
    }
    // pre-fill to full size
    if (AP::FS().lseek(log_fd, ret, SEEK_SET) != ret ||
        AP::FS().write(log_fd, &_buffer[ret], CH_STORAGE_SIZE-ret) != CH_STORAGE_SIZE-ret) {
        ::printf("setup failed for " HAL_STORAGE_FILE "\n");
        AP::FS().close(log_fd);
        log_fd = -1;
        return;
    }
    using_filesystem = true;
#endif

    _save_backup();
    _initialised = true;
}

/*
  save a backup of storage file if we have microSD available. This is
  very handy for diagnostics, and for moving a copy of storage into
  SITL for testing
 */
void Storage::_save_backup(void)
{
#ifdef USE_POSIX
    // allow for fallback to microSD based storage
    sdcard_retry();
    int fd = AP::FS().open(HAL_STORAGE_BACKUP_FILE, O_WRONLY|O_CREAT|O_TRUNC);
    if (fd != -1) {
        AP::FS().write(fd, _buffer, CH_STORAGE_SIZE);
        AP::FS().close(fd);
    }
#endif
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
*/
void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    if (length == 0) {
        return;
    }
    uint16_t end = loc + length - 1;
    for (uint16_t line=loc>>CH_STORAGE_LINE_SHIFT;
         line <= end>>CH_STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (!_initialised) {
        return;
    }
    if (_dirty_mask.empty()) {
        _last_empty_ms = AP_HAL::millis();
        return;
    }

    // write out the first dirty line. We don't write more
    // than one to keep the latency of this call to a minimum
    uint16_t i;
    for (i=0; i<CH_STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == CH_STORAGE_NUM_LINES) {
        // this shouldn't be possible
        return;
    }

#if HAL_WITH_RAMTRON
    if (using_fram) {
        if (fram.write(CH_STORAGE_LINE_SIZE*i, &_buffer[CH_STORAGE_LINE_SIZE*i], CH_STORAGE_LINE_SIZE)) {
            _dirty_mask.clear(i);
        }
        return;
    }
#endif

#ifdef USE_POSIX
    if (using_filesystem && log_fd != -1) {
        uint32_t offset = CH_STORAGE_LINE_SIZE*i;
        if (AP::FS().lseek(log_fd, offset, SEEK_SET) != offset) {
            return;
        }
        if (AP::FS().write(log_fd, &_buffer[offset], CH_STORAGE_LINE_SIZE) != CH_STORAGE_LINE_SIZE) {
            return;
        }
        if (AP::FS().fsync(log_fd) != 0) {
            return;
        }
        _dirty_mask.clear(i);
        return;
    }
#endif

#ifdef STORAGE_FLASH_PAGE
    // save to storage backend
    _flash_write(i);
#endif
}

/*
  load all data from flash
 */
void Storage::_flash_load(void)
{
#ifdef STORAGE_FLASH_PAGE
    _flash_page = STORAGE_FLASH_PAGE;

    ::printf("Storage: Using flash pages %u and %u\n", _flash_page, _flash_page+1);

    if (!_flash.init()) {
        AP_HAL::panic("unable to init flash storage");
    }
#else
    AP_HAL::panic("unable to init storage");
#endif
}

/*
  write one storage line. This also updates _dirty_mask.
*/
void Storage::_flash_write(uint16_t line)
{
#ifdef STORAGE_FLASH_PAGE
    if (_flash.write(line*CH_STORAGE_LINE_SIZE, CH_STORAGE_LINE_SIZE)) {
        // mark the line clean
        _dirty_mask.clear(line);
    }
#endif
}

/*
  callback to write data to flash
 */
bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
#ifdef STORAGE_FLASH_PAGE
    size_t base_address = hal.flash->getpageaddr(_flash_page+sector);
    for (uint8_t i=0; i<STORAGE_FLASH_RETRIES; i++) {
        if (hal.flash->write(base_address+offset, data, length)) {
            return true;
        }
        hal.scheduler->delay(1);
    }
    if (_flash_erase_ok()) {
        // we are getting flash write errors while disarmed. Try
        // re-writing all of flash
        uint32_t now = AP_HAL::millis();
        if (now - _last_re_init_ms > 5000) {
            _last_re_init_ms = now;
            bool ok = _flash.re_initialise();
            ::printf("Storage: failed at %u:%u for %u - re-init %u\n",
                     (unsigned)sector, (unsigned)offset, (unsigned)length, (unsigned)ok);
        }
    }
    return false;
#else
    return false;
#endif
}

/*
  callback to read data from flash
 */
bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    size_t base_address = hal.flash->getpageaddr(_flash_page+sector);
    const uint8_t *b = ((const uint8_t *)base_address)+offset;
    memcpy(data, b, length);
    return true;
}

/*
  callback to erase flash sector
 */
bool Storage::_flash_erase_sector(uint8_t sector)
{
    for (uint8_t i=0; i<STORAGE_FLASH_RETRIES; i++) {
        if (hal.flash->erasepage(_flash_page+sector)) {
            return true;
        }
        hal.scheduler->delay(1);
    }
    return false;
}

/*
  callback to check if erase is allowed
 */
bool Storage::_flash_erase_ok(void)
{
    // only allow erase while disarmed
    return !hal.util->get_soft_armed();
}

/*
  consider storage healthy if we have nothing to write sometime in the
  last 2 seconds
 */
bool Storage::healthy(void)
{
    return _initialised && AP_HAL::millis() - _last_empty_ms < 2000;
}

/*
  erase all storage
 */
bool Storage::erase(void)
{
#if HAL_WITH_RAMTRON
    if (using_fram) {
        return AP_HAL::Storage::erase();
    }
#endif
#ifdef USE_POSIX
    if (using_filesystem) {
        return AP_HAL::Storage::erase();
    }
#endif
#ifdef STORAGE_FLASH_PAGE
    return _flash.erase();
#else
    return false;
#endif
}

#endif // HAL_USE_EMPTY_STORAGE

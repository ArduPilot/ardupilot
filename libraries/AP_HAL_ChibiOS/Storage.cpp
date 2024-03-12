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
#include "HAL_ChibiOS_Class.h"
#include "Scheduler.h"
#include "hwdef/common/flash.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <stdio.h>

using namespace ChibiOS;

#ifndef HAL_USE_EMPTY_STORAGE

extern const AP_HAL::HAL& hal;

#ifndef HAL_STORAGE_FILE
// using SKETCHNAME allows the one microSD to be used
// for multiple vehicle types
#define HAL_STORAGE_FILE "/APM/" SKETCHNAME ".stg"
#endif

#ifndef HAL_STORAGE_BACKUP_FOLDER
// location of backup file
#define HAL_STORAGE_BACKUP_FOLDER "/APM/STRG_BAK"
#endif

#ifndef HAL_STORAGE_BACKUP_COUNT
#define HAL_STORAGE_BACKUP_COUNT 100
#endif

#define STORAGE_FLASH_RETRIES 5

// by default don't allow fallback to sdcard for storage
#ifndef HAL_RAMTRON_ALLOW_FALLBACK
#define HAL_RAMTRON_ALLOW_FALLBACK 0
#endif

void Storage::_storage_open(void)
{
    if (_initialisedType != StorageBackend::None) {
        return;
    }

    _dirty_mask.clearall();

#if HAL_WITH_RAMTRON
    if (fram.init() && fram.read(0, _buffer, CH_STORAGE_SIZE)) {
        _save_backup();
        _initialisedType = StorageBackend::FRAM;
        ::printf("Initialised Storage type=%d\n", _initialisedType);
        return;
    }

#if !HAL_RAMTRON_ALLOW_FALLBACK
    AP_HAL::panic("Unable to init RAMTRON storage");
#endif

#endif // HAL_WITH_RAMTRON

// allow for devices with no FRAM chip to fall through to other storage
#ifdef STORAGE_FLASH_PAGE
        // load from storage backend
        _flash_load();
        _save_backup();
        _initialisedType = StorageBackend::Flash;
#elif defined(USE_POSIX)
        // if we have failed filesystem init don't try again
        if (log_fd == -1) {
            return;
        }

        // use microSD based storage
        if (AP::FS().retry_mount()) {
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
                (CH_STORAGE_SIZE-ret > 0 && AP::FS().write(log_fd, &_buffer[ret], CH_STORAGE_SIZE-ret) != CH_STORAGE_SIZE-ret)) {
                ::printf("setup failed for " HAL_STORAGE_FILE "\n");
                AP::FS().close(log_fd);
                log_fd = -1;
                return;
            }
            _save_backup();
            _initialisedType = StorageBackend::SDCard;
        }
#endif

    if (_initialisedType != StorageBackend::None) {
        ::printf("Initialised Storage type=%d\n", _initialisedType);
    } else {
        AP_HAL::panic("Unable to init Storage backend");
    }
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
    // create the backup directory if need be
    int ret;
    const char* _storage_bak_directory = HAL_STORAGE_BACKUP_FOLDER;

    if (hal.util->was_watchdog_armed()) {
        // we are under watchdog reset
        // ain't got no time...
        return;
    }

    EXPECT_DELAY_MS(3000);

    // We want to do this desperately,
    // So we keep trying this for a second
    uint32_t start_millis = AP_HAL::millis();
    while(!AP::FS().retry_mount() && (AP_HAL::millis() - start_millis) < 1000) {
        hal.scheduler->delay(1);        
    }

    ret = AP::FS().mkdir(_storage_bak_directory);
    if (ret == -1 && errno != EEXIST) {
        return;
    }

    char* fname = nullptr;
    unsigned curr_bak = 0;
    ret = asprintf(&fname, "%s/last_storage_bak", _storage_bak_directory);
    if (fname == nullptr && (ret <= 0)) {
        return;
    }
    int fd = AP::FS().open(fname, O_RDONLY);
    if (fd != -1) {
        char buf[10];
        memset(buf, 0, sizeof(buf));
        if (AP::FS().read(fd, buf, sizeof(buf)-1) > 0) {
            //only record last HAL_STORAGE_BACKUP_COUNT backups
            curr_bak = (strtol(buf, NULL, 10) + 1)%HAL_STORAGE_BACKUP_COUNT;
        }
        AP::FS().close(fd);
    }

    fd = AP::FS().open(fname, O_WRONLY|O_CREAT|O_TRUNC);
    free(fname);
    fname = nullptr;
    if (fd != -1) {
        char buf[10];
        snprintf(buf, sizeof(buf), "%u\r\n", (unsigned)curr_bak);
        const ssize_t to_write = strlen(buf);
        const ssize_t written = AP::FS().write(fd, buf, to_write);
        AP::FS().close(fd);
        if (written < to_write) {
            return;
        }
    } else {
        return;
    }

    // create and write fram data to file
    ret = asprintf(&fname, "%s/STRG%d.bak", _storage_bak_directory, curr_bak);
    if (fname == nullptr || (ret <= 0)) {
        return;
    }
    fd = AP::FS().open(fname, O_WRONLY|O_CREAT|O_TRUNC);
    free(fname);
    fname = nullptr;
    if (fd != -1) {
        //finally dump the fram data
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
    if ((n > sizeof(_buffer)) || (loc > (sizeof(_buffer) - n))) {
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if ((n > sizeof(_buffer)) || (loc > (sizeof(_buffer) - n))) {
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        WITH_SEMAPHORE(sem);
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (_initialisedType == StorageBackend::None) {
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

    {
        // take a copy of the line we are writing with a semaphore held
        WITH_SEMAPHORE(sem);
        memcpy(tmpline, &_buffer[CH_STORAGE_LINE_SIZE*i], CH_STORAGE_LINE_SIZE);
    }

    bool write_ok = false;

#if HAL_WITH_RAMTRON
    if (_initialisedType == StorageBackend::FRAM) {
        if (fram.write(CH_STORAGE_LINE_SIZE*i, tmpline, CH_STORAGE_LINE_SIZE)) {
            write_ok = true;
        }
    }
#endif

#ifdef USE_POSIX
    if ((_initialisedType == StorageBackend::SDCard) && log_fd != -1) {
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
        write_ok = true;
    }
#endif

#ifdef STORAGE_FLASH_PAGE
    if (_initialisedType == StorageBackend::Flash) {
        // save to storage backend
        if (_flash_write(i)) {
            write_ok = true;
        }
    }
#endif

    if (write_ok) {
        WITH_SEMAPHORE(sem);
        // while holding the semaphore we check if the copy of the
        // line is different from the original line. If it is
        // different then someone has re-dirtied the line while we
        // were writing it, in which case we should not mark it
        // clean. If it matches then we know we can mark the line as
        // clean
        if (memcmp(tmpline, &_buffer[CH_STORAGE_LINE_SIZE*i], CH_STORAGE_LINE_SIZE) == 0) {
            _dirty_mask.clear(i);
        }
    }
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
        AP_HAL::panic("Unable to init flash storage");
    }
#else
    AP_HAL::panic("Unable to init storage");
#endif
}

/*
  write one storage line. This also updates _dirty_mask.
*/
bool Storage::_flash_write(uint16_t line)
{
#ifdef STORAGE_FLASH_PAGE
    EXPECT_DELAY_MS(1);
    return _flash.write(line*CH_STORAGE_LINE_SIZE, CH_STORAGE_LINE_SIZE);
#else
    return false;
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
        EXPECT_DELAY_MS(1);
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
#ifdef STORAGE_FLASH_PAGE
    size_t base_address = hal.flash->getpageaddr(_flash_page+sector);
    const uint8_t *b = ((const uint8_t *)base_address)+offset;
    memcpy(data, b, length);
    return true;
#else
    return false;
#endif
}

/*
  callback to erase flash sector
 */
bool Storage::_flash_erase_sector(uint8_t sector)
{
#ifdef STORAGE_FLASH_PAGE
    // erasing a page can take long enough that USB may not initialise properly if it happens
    // while the host is connecting. Only do a flash erase if we have been up for more than 4s
    for (uint8_t i=0; i<STORAGE_FLASH_RETRIES; i++) {
        /*
          a sector erase stops the whole MCU. We need to setup a long
          expected delay, and not only when running in the main
          thread.  We can't use EXPECT_DELAY_MS() as it checks we are
          in the main thread
         */
        EXPECT_DELAY_MS(1000);
        if (hal.flash->erasepage(_flash_page+sector)) {
            return true;
        }
        hal.scheduler->delay(1);
    }
    return false;
#else
    return false;
#endif
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
#ifdef USE_POSIX
    // SD card storage is really slow
    if (_initialisedType == StorageBackend::SDCard) {
        return log_fd != -1 || AP_HAL::millis() - _last_empty_ms < 30000U;
    }
#endif
    return ((_initialisedType != StorageBackend::None) &&
            (AP_HAL::millis() - _last_empty_ms < 2000u));
}

/*
  erase all storage
 */
bool Storage::erase(void)
{
#if HAL_WITH_RAMTRON
    if (_initialisedType == StorageBackend::FRAM) {
        return AP_HAL::Storage::erase();
    }
#endif
#ifdef USE_POSIX
    if (_initialisedType == StorageBackend::SDCard) {
        return AP_HAL::Storage::erase();
    }
#endif
#ifdef STORAGE_FLASH_PAGE
    return _flash.erase();
#else
    return false;
#endif
}

/*
  get storage size and ptr
 */
bool Storage::get_storage_ptr(void *&ptr, size_t &size)
{
    if (_initialisedType==StorageBackend::None) {
        return false;
    }
    ptr = _buffer;
    size = sizeof(_buffer);
    return true;
}


#endif // HAL_USE_EMPTY_STORAGE

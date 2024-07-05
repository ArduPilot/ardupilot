#include "Storage.h"

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

using namespace QURT;

#define QURT_STORAGE_MAX_WRITE 512
#define QURT_STORAGE_LINE_SHIFT 9
#define QURT_STORAGE_LINE_SIZE (1<<QURT_STORAGE_LINE_SHIFT)
#define QURT_STORAGE_NUM_LINES (QURT_STORAGE_SIZE/QURT_STORAGE_LINE_SIZE)

/*
  This stores 'eeprom' data on the SD card, with a 4k size, and a
  in-memory buffer. This keeps the latency down.
 */

// name the storage file after the sketch so you can use the same board
// card for ArduCopter and ArduPlane
#define STORAGE_FILE AP_FILESYSTEM_POSIX_MAP_FILENAME_BASEDIR "/APM/" AP_BUILD_TARGET_NAME ".stg"

extern const AP_HAL::HAL& hal;

bool Storage::_storage_create(void)
{
    int fd = -1;

    fd = open(STORAGE_FILE, O_WRONLY|O_CREAT|O_TRUNC, 0755);
    if (fd == -1) {
        AP_HAL::panic("Failed to open storage: %s", STORAGE_FILE);
        return false;
    }

    // take up all needed space
    if (write(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
        AP_HAL::panic("Failed to init %s", STORAGE_FILE);
        goto fail;
    }

    // ensure the directory is updated with the new size
    close(fd);

    return true;

fail:
    if (fd != -1) {
        close(fd);
    }
    return false;
}

void Storage::init()
{
    if (_initialised) {
        return;
    }

    _dirty_mask = 0;

    int fd = open(STORAGE_FILE, O_RDWR);
    if (fd == -1) {
        _storage_create();
        fd = open(STORAGE_FILE, O_RDWR);
        if (fd == -1) {
            AP_HAL::panic("Failed to create %s", STORAGE_FILE);
        }
    }

    ssize_t ret = read(fd, _buffer, sizeof(_buffer));

    if (ret != sizeof(_buffer)) {
        close(fd);
        _storage_create();
        fd = open(STORAGE_FILE, O_RDWR);
        if (fd == -1) {
            AP_HAL::panic("Failed to open %s", STORAGE_FILE);
        }
        if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
            AP_HAL::panic("Failed to read %s", STORAGE_FILE);
        }
    }

    _fd = fd;
    _initialised = true;
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
    for (uint8_t line=loc>>QURT_STORAGE_LINE_SHIFT;
         line <= end>>QURT_STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask |= 1U << line;
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    init();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        init();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (!_initialised || _dirty_mask == 0 || _fd == -1) {
        return;
    }

    // write out the first dirty set of lines. We don't write more
    // than one to keep the latency of this call to a minimum
    uint8_t i, n;
    for (i=0; i<QURT_STORAGE_NUM_LINES; i++) {
        if (_dirty_mask & (1<<i)) {
            break;
        }
    }
    if (i == QURT_STORAGE_NUM_LINES) {
        // this shouldn't be possible
        return;
    }
    uint32_t write_mask = (1U<<i);
    // see how many lines to write
    for (n=1; (i+n) < QURT_STORAGE_NUM_LINES &&
         n < (QURT_STORAGE_MAX_WRITE>>QURT_STORAGE_LINE_SHIFT); n++) {
        if (!(_dirty_mask & (1<<(n+i)))) {
            break;
        }
        // mark that line clean
        write_mask |= (1<<(n+i));
    }

    /*
      write the lines. This also updates _dirty_mask. Note that
      because this is a SCHED_FIFO thread it will not be preempted
      by the main task except during blocking calls. This means we
      don't need a semaphore around the _dirty_mask updates.
     */
    if (lseek(_fd, i<<QURT_STORAGE_LINE_SHIFT, SEEK_SET) == (i<<QURT_STORAGE_LINE_SHIFT)) {
        _dirty_mask &= ~write_mask;
        if (write(_fd, &_buffer[i<<QURT_STORAGE_LINE_SHIFT], n<<QURT_STORAGE_LINE_SHIFT) != n<<QURT_STORAGE_LINE_SHIFT) {
            // write error - likely EINTR
            _dirty_mask |= write_mask;
            close(_fd);
            _fd = -1;
        }
        if (_dirty_mask == 0) {
            // close and re-open to force storage to sync, QURT has no fsync() call
            close(_fd);
            _fd = open(STORAGE_FILE, O_RDWR);
        }
    }
}

/*
  get storage size and ptr
 */
bool Storage::get_storage_ptr(void *&ptr, size_t &size)
{
    if (!_initialised) {
        return false;
    }
    ptr = _buffer;
    size = sizeof(_buffer);
    return true;
}

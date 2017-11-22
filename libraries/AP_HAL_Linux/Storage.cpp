#include "Storage.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

using namespace Linux;

/*
  This stores 'eeprom' data on the SD card, with a 4k size, and a
  in-memory buffer. This keeps the latency down.
 */

// name the storage file after the sketch so you can use the same board
// card for ArduCopter and ArduPlane
#if APM_BUILD_TYPE(APM_BUILD_Replay)
#define STORAGE_DIR "."
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ
#define STORAGE_DIR "/mnt/APM"
#else
#define STORAGE_DIR HAL_BOARD_STORAGE_DIRECTORY
#endif

#define STORAGE_FILE STORAGE_DIR "/" SKETCHNAME ".stg"

extern const AP_HAL::HAL& hal;

void Storage::_storage_create(void)
{
    mkdir(STORAGE_DIR, 0777);
    unlink(STORAGE_FILE);
    int fd = open(STORAGE_FILE, O_RDWR|O_CREAT|O_CLOEXEC, 0666);
    if (fd == -1) {
        AP_HAL::panic("Failed to create " STORAGE_FILE);
    }
    for (uint16_t loc=0; loc<sizeof(_buffer); loc += LINUX_STORAGE_MAX_WRITE) {
        if (write(fd, &_buffer[loc], LINUX_STORAGE_MAX_WRITE) != LINUX_STORAGE_MAX_WRITE) {
            perror("write");
            AP_HAL::panic("Error filling " STORAGE_FILE);
        }
    }
    // ensure the directory is updated with the new size
    fsync(fd);
    close(fd);
}

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }

    _dirty_mask = 0;
    int fd = open(STORAGE_FILE, O_RDWR|O_CLOEXEC);
    if (fd == -1) {
        _storage_create();
        fd = open(STORAGE_FILE, O_RDWR|O_CLOEXEC);
        if (fd == -1) {
            AP_HAL::panic("Failed to open " STORAGE_FILE);
        }
    }
    memset(_buffer, 0, sizeof(_buffer));
    /*
      we allow a read of size 4096 to cope with the old storage size
      without forcing users to reset all parameters
     */
    ssize_t ret = read(fd, _buffer, sizeof(_buffer));
    if (ret == 4096 && ret != sizeof(_buffer)) {
        if (ftruncate(fd, sizeof(_buffer)) != 0) {
            AP_HAL::panic("Failed to expand " STORAGE_FILE);
        }
        ret = sizeof(_buffer);
    }
    if (ret != sizeof(_buffer)) {
        close(fd);
        _storage_create();
        fd = open(STORAGE_FILE, O_RDONLY|O_CLOEXEC);
        if (fd == -1) {
            AP_HAL::panic("Failed to open " STORAGE_FILE);
        }
        if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
            AP_HAL::panic("Failed to read " STORAGE_FILE);
        }
    }
    close(fd);
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
    uint16_t end = loc + length;
    for (uint8_t line=loc>>LINUX_STORAGE_LINE_SHIFT;
         line <= end>>LINUX_STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask |= 1U << line;
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
    if (!_initialised || _dirty_mask == 0) {
        return;
    }

    if (_fd == -1) {
        _fd = open(STORAGE_FILE, O_WRONLY|O_CLOEXEC);
        if (_fd == -1) {
            return;
        }
    }

    // write out the first dirty set of lines. We don't write more
    // than one to keep the latency of this call to a minimum
    uint8_t i, n;
    for (i=0; i<LINUX_STORAGE_NUM_LINES; i++) {
        if (_dirty_mask & (1<<i)) {
            break;
        }
    }
    if (i == LINUX_STORAGE_NUM_LINES) {
        // this shouldn't be possible
        return;
    }
    uint32_t write_mask = (1U<<i);
    // see how many lines to write
    for (n=1; (i+n) < LINUX_STORAGE_NUM_LINES &&
             n < (LINUX_STORAGE_MAX_WRITE>>LINUX_STORAGE_LINE_SHIFT); n++) {
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
    if (lseek(_fd, i<<LINUX_STORAGE_LINE_SHIFT, SEEK_SET) == (i<<LINUX_STORAGE_LINE_SHIFT)) {
        _dirty_mask &= ~write_mask;
        if (write(_fd, &_buffer[i<<LINUX_STORAGE_LINE_SHIFT], n<<LINUX_STORAGE_LINE_SHIFT) != n<<LINUX_STORAGE_LINE_SHIFT) {
            // write error - likely EINTR
            _dirty_mask |= write_mask;
            close(_fd);
            _fd = -1;
        }
        if (_dirty_mask == 0) {
            if (fsync(_fd) != 0) {
                close(_fd);
                _fd = -1;
            }
        }
    }
}

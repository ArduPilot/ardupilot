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
#define STORAGE_FILE SKETCHNAME ".stg"

extern const AP_HAL::HAL& hal;

static inline int is_dir(const char *path)
{
    struct stat st;

    if (stat(path, &st) < 0) {
        return -errno;
    }

    return S_ISDIR(st.st_mode);
}

static int mkdir_p(const char *path, int len, mode_t mode)
{
    char *start, *end;

    start = strndupa(path, len);
    end = start + len;

    /*
     * scan backwards, replacing '/' with '\0' while the component doesn't
     * exist
     */
    for (;;) {
        int r = is_dir(start);
        if (r > 0) {
            end += strlen(end);

            if (end == start + len) {
                return 0;
            }

            /* end != start, since it would be caught on the first
             * iteration */
            *end = '/';
            break;
        } else if (r == 0) {
            return -ENOTDIR;
        }

        if (end == start) {
            break;
        }

        *end = '\0';

        /* Find the next component, backwards, discarding extra '/'*/
        while (end > start && *end != '/') {
            end--;
        }

        while (end > start && *(end - 1) == '/') {
            end--;
        }
    }

    while (end < start + len) {
        if (mkdir(start, mode) < 0 && errno != EEXIST) {
            return -errno;
        }

        end += strlen(end);
        *end = '/';
    }

    return 0;
}

int Storage::_storage_create(const char *dpath)
{
    int dfd = -1;

    mkdir_p(dpath, strlen(dpath), 0777);
    dfd = open(dpath, O_RDONLY|O_CLOEXEC);
    if (dfd == -1) {
        fprintf(stderr, "Failed to open storage directory: %s (%m)\n", dpath);
        return -1;
    }

    unlinkat(dfd, dpath, 0);
    int fd = openat(dfd, STORAGE_FILE, O_RDWR|O_CREAT|O_CLOEXEC, 0666);

    close(dfd);

    if (fd == -1) {
        fprintf(stderr, "Failed to create storage file %s/%s\n", dpath,
                STORAGE_FILE);
        goto fail;
    }

    // take up all needed space
    if (ftruncate(fd, sizeof(_buffer)) == -1) {
        fprintf(stderr, "Failed to set file size to %lu kB (%m)\n",
                sizeof(_buffer) / 1024);
        goto fail;
    }

    // ensure the directory is updated with the new size
    fsync(fd);
    fsync(dfd);

    close(dfd);

    return fd;

fail:
    close(dfd);
    return -1;
}

void Storage::init()
{
    const char *dpath;

    if (_initialised) {
        return;
    }

    _dirty_mask = 0;

    dpath = hal.util->get_custom_storage_directory();
    if (!dpath) {
        dpath = HAL_BOARD_STORAGE_DIRECTORY;
    }

    int fd = open(dpath, O_RDWR|O_CLOEXEC);
    if (fd == -1) {
        fd = _storage_create(dpath);
        if (fd == -1) {
            AP_HAL::panic("Cannot create storage %s (%m)", dpath);
        }
    }

    ssize_t ret = read(fd, _buffer, sizeof(_buffer));

    if (ret != sizeof(_buffer)) {
        close(fd);
        _storage_create(dpath);
        fd = open(dpath, O_RDONLY|O_CLOEXEC);
        if (fd == -1) {
            AP_HAL::panic("Failed to open %s (%m)", dpath);
        }
        if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
            AP_HAL::panic("Failed to read %s (%m)", dpath);
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

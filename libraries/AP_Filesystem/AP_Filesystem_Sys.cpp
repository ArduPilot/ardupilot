// 2 or 3 structures, select one that is before target point, closest to target

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  ArduPilot filesystem interface for system information
 */
#include "AP_Filesystem.h"
#include "AP_Filesystem_Sys.h"
#include <AP_Math/AP_Math.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Scheduler/AP_Scheduler.h>

extern const AP_HAL::HAL& hal;

int AP_Filesystem_Sys::open(const char *fname, int flags)
{
    if ((flags & O_ACCMODE) != O_RDONLY) {
        errno = EROFS;
        return -1;
    }
    uint8_t idx;
    for (idx=0; idx<max_open_file; idx++) {
        if (!file[idx].open) {
            break;
        }
    }
    if (idx == max_open_file) {
        errno = ENFILE;
        return -1;
    }
    struct rfile &r = file[idx];
    r.data = new file_data;
    if (r.data == nullptr) {
        errno = ENOMEM;
        return -1;
    }
    if (strcmp(fname, "threads.txt") == 0) {
        const uint32_t max_size = 1024;
        r.data->data = (char *)malloc(max_size);
        if (r.data->data) {
            r.data->length = hal.util->thread_info(r.data->data, max_size);
        }
    }
    if (strcmp(fname, "tasks.txt") == 0) {
        const uint32_t max_size = 6500;
        r.data->data = (char *)malloc(max_size);
        if (r.data->data) {
            r.data->length = AP::scheduler().task_info(r.data->data, max_size);
            if (r.data->length == 0) { // the feature may be disabled
                free(r.data->data);
                r.data->data = nullptr;
            }
        }
    }
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    int8_t can_stats_num = -1;
    if (strcmp(fname, "can_log.txt") == 0) {
        const uint32_t max_size = 1024;
        r.data->data = (char *)malloc(max_size);
        if (r.data->data) {
            r.data->length = AP::can().log_retrieve(r.data->data, max_size);
        }
    } else if (strcmp(fname, "can0_stats.txt") == 0) {
        can_stats_num = 0;
    } else if (strcmp(fname, "can1_stats.txt") == 0) {
        can_stats_num = 1;
    }
    if (can_stats_num != -1 && can_stats_num < HAL_NUM_CAN_IFACES) {
        if (hal.can[can_stats_num] != nullptr) {
            const uint32_t max_size = 1024;
            r.data->data = (char *)malloc(max_size);
            if (r.data->data) {
                r.data->length = hal.can[can_stats_num]->get_stats(r.data->data, max_size);
            }
        }
    }
#endif
    if (r.data->data == nullptr) {
        delete r.data;
        errno = ENOENT;
        return -1;
    }
    r.file_ofs = 0;
    r.open = true;
    return idx;
}

int AP_Filesystem_Sys::close(int fd)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
    r.open = false;
    free(r.data->data);
    delete r.data;
    return 0;
}

int32_t AP_Filesystem_Sys::read(int fd, void *buf, uint32_t count)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
    count = MIN(count, r.data->length - r.file_ofs);
    memcpy(buf, &r.data->data[r.file_ofs], count);
    r.file_ofs += count;
    return count;
}

int32_t AP_Filesystem_Sys::lseek(int fd, int32_t offset, int seek_from)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
    switch (seek_from) {
    case SEEK_SET:
        r.file_ofs = MIN(offset, int32_t(r.data->length));
        break;
    case SEEK_CUR:
        r.file_ofs = MIN(r.data->length, offset+r.file_ofs);
        break;
    case SEEK_END:
        errno = EINVAL;
        return -1;
    }
    return r.file_ofs;
}

int AP_Filesystem_Sys::stat(const char *name, struct stat *stbuf)
{
    memset(stbuf, 0, sizeof(*stbuf));
    // give fixed size, EOF on read gives real size
    stbuf->st_size = 1024*1024;
    return 0;
}

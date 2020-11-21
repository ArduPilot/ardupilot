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

struct SysFileList {
    const char* name;
    uint32_t filesize; 
};

static const SysFileList sysfs_file_list[] = {
    {"threads.txt", 1024},
    {"tasks.txt", 6500},
    {"dma.txt", 1024},
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    {"can_log.txt", 1024},
    {"can0_stats.txt", 1024},
    {"can1_stats.txt", 1024},
#endif
};

int8_t AP_Filesystem_Sys::file_in_sysfs(const char *fname) {
    for (uint8_t i = 0; i <  ARRAY_SIZE(sysfs_file_list); i++) {
        if (strcmp(fname, sysfs_file_list[i].name) == 0) {
            return i;
        }
    }
    return -1;
}

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

    // This ensure that whenever new sys file is added its also added to list above
    int8_t pos = file_in_sysfs(fname);
    uint32_t max_size = 0;
    if (pos >= 0) {
        max_size = sysfs_file_list[pos].filesize;
    } else {
        errno = ENOENT;
        return -1;
    }

    if (strcmp(fname, "threads.txt") == 0) {
        r.data->data = (char *)malloc(max_size);
        if (r.data->data) {
            r.data->length = hal.util->thread_info(r.data->data, max_size);
        }
    }
    if (strcmp(fname, "tasks.txt") == 0) {
        r.data->data = (char *)malloc(max_size);
        if (r.data->data) {
            r.data->length = AP::scheduler().task_info(r.data->data, max_size);
            if (r.data->length == 0) { // the feature may be disabled
                free(r.data->data);
                r.data->data = nullptr;
            }
        }
    }
    if (strcmp(fname, "dma.txt") == 0) {
        r.data->data = (char *)malloc(max_size);
        if (r.data->data) {
            r.data->length = hal.util->dma_info(r.data->data, max_size);
        }
    }
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    int8_t can_stats_num = -1;
    if (strcmp(fname, "can_log.txt") == 0) {
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

void *AP_Filesystem_Sys::opendir(const char *pathname)
{
    if (strlen(pathname) > 0) {
        // no sub directories
        errno = ENOENT;
        return nullptr;
    }
    DirReadTracker *dtracker = new DirReadTracker;
    if (dtracker == nullptr) {
        errno = ENOMEM;
        return nullptr;
    }
    return dtracker;
}

struct dirent *AP_Filesystem_Sys::readdir(void *dirp)
{
    DirReadTracker* dtracker = ((DirReadTracker*)dirp);
    if (dtracker->file_offset >= ARRAY_SIZE(sysfs_file_list)) {
        // we have reached end of list
        return nullptr;
    }
    dtracker->curr_file.d_type = DT_REG;
    size_t max_length = ARRAY_SIZE(dtracker->curr_file.d_name);
    strncpy_noterm(dtracker->curr_file.d_name, sysfs_file_list[dtracker->file_offset].name, max_length);
    dtracker->file_offset++;
    return &dtracker->curr_file;
}

int AP_Filesystem_Sys::closedir(void *dirp)
{
    if (dirp == nullptr) {
        errno = EINVAL;
        return -1;
    }
    delete (DirReadTracker*)dirp;
    return 0;
}

int AP_Filesystem_Sys::stat(const char *pathname, struct stat *stbuf)
{
    if (pathname == nullptr || stbuf == nullptr || (strlen(pathname) == 0)) {
        errno = EINVAL;
        return -1;
    }
    memset(stbuf, 0, sizeof(*stbuf));
    if (strlen(pathname) == 1 && pathname[0] == '/') {
        stbuf->st_size = 0; // just a placeholder value
        return 0;
    }
    const char *pathname_noslash = pathname;
    if (pathname[0] == '/') {
        pathname_noslash = &pathname[1];
    }
    int8_t pos = file_in_sysfs(pathname_noslash);
    if (pos < 0) {
        errno = ENOENT;
        return -1;
    }
    stbuf->st_size = sysfs_file_list[pos].filesize;
    return 0;
}

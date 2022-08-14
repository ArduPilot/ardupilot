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

#if AP_FILESYSTEM_SYS_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Common/ExpandingString.h>

extern const AP_HAL::HAL& hal;

struct SysFileList {
    const char* name;
};

static const SysFileList sysfs_file_list[] = {
    {"threads.txt"},
    {"tasks.txt"},
    {"dma.txt"},
    {"memory.txt"},
    {"uarts.txt"},
    {"timers.txt"},
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    {"can_log.txt"},
#endif
#if HAL_NUM_CAN_IFACES > 0
    {"can0_stats.txt"},
    {"can1_stats.txt"},
#endif
#if !defined(HAL_BOOTLOADER_BUILD) && (defined(STM32F7) || defined(STM32H7))
    {"persistent.parm"},
#endif
    {"crash_dump.bin"},
    {"storage.bin"},
};

int8_t AP_Filesystem_Sys::file_in_sysfs(const char *fname) {
    for (uint8_t i = 0; i <  ARRAY_SIZE(sysfs_file_list); i++) {
        if (strcmp(fname, sysfs_file_list[i].name) == 0) {
            return i;
        }
    }
    return -1;
}

int AP_Filesystem_Sys::open(const char *fname, int flags, bool allow_absolute_paths)
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
    r.str = new ExpandingString;
    if (r.str == nullptr) {
        errno = ENOMEM;
        return -1;
    }

    // This ensure that whenever new sys file is added its also added to list above
    int8_t pos = file_in_sysfs(fname);
    if (pos < 0) {
        delete r.str;
        r.str = nullptr;
        errno = ENOENT;
        return -1;
    }

    if (strcmp(fname, "threads.txt") == 0) {
        hal.util->thread_info(*r.str);
    }
#if HAL_SCHEDULER_ENABLED
    if (strcmp(fname, "tasks.txt") == 0) {
        AP::scheduler().task_info(*r.str);
    }
#endif
    if (strcmp(fname, "dma.txt") == 0) {
        hal.util->dma_info(*r.str);
    }
    if (strcmp(fname, "memory.txt") == 0) {
        hal.util->mem_info(*r.str);
    }
#if HAL_UART_STATS_ENABLED
    if (strcmp(fname, "uarts.txt") == 0) {
        hal.util->uart_info(*r.str);
    }
#endif
    if (strcmp(fname, "timers.txt") == 0) {
        hal.util->timer_info(*r.str);
    }
#if HAL_CANMANAGER_ENABLED
    if (strcmp(fname, "can_log.txt") == 0) {
        AP::can().log_retrieve(*r.str);
    }
#endif
#if HAL_NUM_CAN_IFACES > 0
    int8_t can_stats_num = -1;
    if (strcmp(fname, "can0_stats.txt") == 0) {
        can_stats_num = 0;
    } else if (strcmp(fname, "can1_stats.txt") == 0) {
        can_stats_num = 1;
    }
    if (can_stats_num != -1 && can_stats_num < HAL_NUM_CAN_IFACES) {
        if (hal.can[can_stats_num] != nullptr) {
            hal.can[can_stats_num]->get_stats(*r.str);
        }
    }
#endif
    if (strcmp(fname, "persistent.parm") == 0) {
        hal.util->load_persistent_params(*r.str);
    }
#if AP_CRASHDUMP_ENABLED
    if (strcmp(fname, "crash_dump.bin") == 0) {
        r.str->set_buffer((char*)hal.util->last_crash_dump_ptr(), hal.util->last_crash_dump_size(), hal.util->last_crash_dump_size());
    }
#endif
    if (strcmp(fname, "storage.bin") == 0) {
        // we don't want to store the contents of storage.bin
        // we read directly from the storage driver
        void *ptr = nullptr;
        size_t size = 0;
        if (hal.storage->get_storage_ptr(ptr, size)) {
            r.str->set_buffer((char*)ptr, size, size);
        }
    }
    
    if (r.str->get_length() == 0) {
        errno = r.str->has_failed_allocation()?ENOMEM:ENOENT;
        delete r.str;
        r.str = nullptr;
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
    delete r.str;
    r.str = nullptr;
    return 0;
}

int32_t AP_Filesystem_Sys::read(int fd, void *buf, uint32_t count)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
    count = MIN(count, r.str->get_length() - r.file_ofs);
    memcpy(buf, &r.str->get_string()[r.file_ofs], count);

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
        r.file_ofs = MIN(offset, int32_t(r.str->get_length()));
        break;
    case SEEK_CUR:
        r.file_ofs = MIN(r.str->get_length(), offset+r.file_ofs);
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
    // give a fixed size for stat. It is too expensive to
    // read every file for a directory listing
    if (strcmp(pathname_noslash, "storage.bin") == 0) {
        stbuf->st_size = HAL_STORAGE_SIZE;
#if AP_CRASHDUMP_ENABLED
    } else if (strcmp(pathname_noslash, "crash_dump.bin") == 0) {
        stbuf->st_size = hal.util->last_crash_dump_size();
#endif
    } else {
        stbuf->st_size = 100000;
    }
    return 0;
}

#endif  // AP_FILESYSTEM_SYS_ENABLED

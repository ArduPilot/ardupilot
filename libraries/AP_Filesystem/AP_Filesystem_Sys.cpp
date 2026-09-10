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

#ifndef AP_CRASHDUMP_FLASH_ENABLED
#define AP_CRASHDUMP_FLASH_ENABLED 0
#endif

extern const AP_HAL::HAL& hal;

#if defined(RP2350) && AP_RP2350_PC_SAMPLER_ENABLED
// Full per-core PC-sampler histogram, from
// AP_HAL_ChibiOS/rp2350_pc_sampler.cpp. Present only when a hwdef turns the
// sampler on.
void rp2350_pc_sampler_dump_full(ExpandingString &str, unsigned core);
#endif

struct SysFileList {
    const char* name;
};

static const SysFileList sysfs_file_list[] = {
    {"threads.txt"},
    {"tasks.txt"},
#if defined(RP2350) && AP_RP2350_PC_SAMPLER_ENABLED
    {"pcprof.txt"},
    {"pcprof0.txt"},
#endif
    {"dma.txt"},
    {"memory.txt"},
    {"uarts.txt"},
    {"timers.txt"},
#if HAL_NUM_CAN_IFACES > 0
    {"can0_stats.txt"},
    {"can1_stats.txt"},
#endif
#if !defined(HAL_BOOTLOADER_BUILD) && (defined(STM32F7) || defined(STM32H7))
    {"persistent.parm"},
#endif
#if AP_CRASHDUMP_FLASH_ENABLED
    {"crash_dump.bin"},
#endif
    {"storage.bin"},
#if AP_FILESYSTEM_SYS_FLASH_ENABLED
    {"flash.bin"},
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
    r.str = NEW_NOTHROW ExpandingString;
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

    r.file_index = uint8_t(pos);
    r.generated = false;

    // Pre-reserve a contiguous buffer for lazily-generated text files so that
    // ensure_generated() only needs to do one heap allocation rather than many
    // incremental reallocs.  On RP2350 (and other boards) late-boot heap
    // fragmentation means the small 512-byte ExpandingString expand increments
    // can fail even with plenty of total free memory.  A single upfront alloc
    // of the expected max size succeeds when many small ones would not.
    // reserve() does not set allocation_failed on failure, so incremental
    // growth still works as a fallback.
    if (strcmp(fname, "threads.txt") == 0) {
        // ~100 bytes per thread with stats, 40 threads max is a safe upper bound.
        // Pre-reserve a single contiguous block to avoid ExpandingString realloc
        // after late-boot heap fragmentation on RP2350.
        r.str->reserve(100 * 40);
    } else if (strcmp(fname, "tasks.txt") == 0) {
        // ArduCopter has ~108 vehicle + common tasks.  Extended format prints
        // 87 bytes/line: 108 * 87 = 9396 bytes + 8 byte header = ~9.4 KB.
        // Reserve a single contiguous block to avoid fragmented realloc fails.
        r.str->reserve(120 * 100);
#if defined(RP2350) && AP_RP2350_PC_SAMPLER_ENABLED
    } else if (strcmp(fname, "pcprof.txt") == 0 ||
               strcmp(fname, "pcprof0.txt") == 0) {
        // Top ~512 PCs at ~14 bytes/token; one modest block that fits the tight
        // runtime heap (~20 KB free after EKF init).
        r.str->reserve(512 * 14);
#endif
    } else if (strcmp(fname, "memory.txt") == 0 ||
               strcmp(fname, "uarts.txt") == 0 ||
               strcmp(fname, "timers.txt") == 0) {
        r.str->reserve(512);
    }

#if AP_CRASHDUMP_FLASH_ENABLED
    if (strcmp(fname, "crash_dump.bin") == 0) {
        void *ptr = hal.util->last_crash_dump_ptr();
        if (ptr != nullptr) {
            r.str->set_buffer((char*)ptr, hal.util->last_crash_dump_size(), hal.util->last_crash_dump_size());
        }
    }
#endif
    if (strcmp(fname, "storage.bin") == 0) {
        // we don't want to store the contents of storage.bin
        // we read directly from the storage driver
        void *ptr = nullptr;
        size_t size = 0;
        if (hal.storage->get_storage_ptr(ptr, size)) {
            r.str->set_buffer((char*)ptr, size, size);
            r.generated = true;
        }
    }
#if AP_FILESYSTEM_SYS_FLASH_ENABLED
    if (strcmp(fname, "flash.bin") == 0) {
        void *ptr = (void*)0x08000000;
        const size_t size = HAL_PROGRAM_SIZE_LIMIT_KB*1024;
        r.str->set_buffer((char*)ptr, size, size);
        r.generated = true;
    }
#endif

    // For lazily generated text files we allow zero length at open()
    // and populate content on the first read()/lseek().
    if (r.generated && r.str->get_length() == 0) {
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
    if (!ensure_generated(r)) {
        return -1;
    }
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
    if (!ensure_generated(r)) {
        return -1;
    }

    int64_t new_ofs = -1;  // -1 being invalid
    switch (seek_from) {
    case SEEK_SET:
        new_ofs = offset;
        break;
    case SEEK_CUR:
        // Compute the new offset in signed 64-bit space to avoid
        // 32-bit overflows:
        new_ofs = int64_t(r.file_ofs) + int64_t(offset);
        break;
    case SEEK_END:
        // we don't support this, leave new_ofs at -1 meaning "invalid"
        break;
    }

    // special semantics for Sysfs - don't allow seeking outside the file
    if (new_ofs < 0 || new_ofs > r.str->get_length()) {
        errno = EINVAL;
        return -1;
    }

    r.file_ofs = (uint32_t)new_ofs;

    // note conversion from uint32_t to int32_t in return value here.
    // Above we clamp to r.str->get_length(), so in practise no
    // truncation can occur here.
    return r.file_ofs;
}

bool AP_Filesystem_Sys::ensure_generated(struct rfile &r)
{
    if (r.generated) {
        return true;
    }

    const char *const fname = sysfs_file_list[r.file_index].name;

    if (strcmp(fname, "threads.txt") == 0) {
        hal.util->thread_info(*r.str);
    }
#if AP_SCHEDULER_ENABLED
    else if (strcmp(fname, "tasks.txt") == 0) {
        AP::scheduler().task_info(*r.str);
    }
#endif
    else if (strcmp(fname, "dma.txt") == 0) {
        hal.util->dma_info(*r.str);
    }
    else if (strcmp(fname, "memory.txt") == 0) {
        hal.util->mem_info(*r.str);
    }
#if HAL_UART_STATS_ENABLED
    else if (strcmp(fname, "uarts.txt") == 0) {
        hal.util->uart_info(*r.str);
    }
#endif
    else if (strcmp(fname, "timers.txt") == 0) {
        hal.util->timer_info(*r.str);
    }
#if HAL_NUM_CAN_IFACES > 0
    else if (strcmp(fname, "can0_stats.txt") == 0 || strcmp(fname, "can1_stats.txt") == 0) {
        const int8_t can_stats_num = (fname[3] == '0') ? 0 : 1;
        if (can_stats_num < HAL_NUM_CAN_IFACES && hal.can[can_stats_num] != nullptr) {
            hal.can[can_stats_num]->get_stats(*r.str);
        }
    }
#endif
    else if (strcmp(fname, "persistent.parm") == 0) {
        hal.util->load_persistent_params(*r.str);
    }
#if defined(RP2350) && AP_RP2350_PC_SAMPLER_ENABLED
    else if (strcmp(fname, "pcprof.txt") == 0) {
        rp2350_pc_sampler_dump_full(*r.str, 1);  // core1 (rate/IMU)
    }
    else if (strcmp(fname, "pcprof0.txt") == 0) {
        rp2350_pc_sampler_dump_full(*r.str, 0);  // core0 (main loop/EKF)
    }
#endif

    if (r.str->has_failed_allocation()) {
        errno = ENOMEM;
        return false;
    }

    if (r.str->get_length() == 0) {
        errno = ENOENT;
        return false;
    }

    r.generated = true;
    return true;
}

void *AP_Filesystem_Sys::opendir(const char *pathname)
{
    if (strlen(pathname) > 0) {
        // no sub directories
        errno = ENOENT;
        return nullptr;
    }
    DirReadTracker *dtracker = NEW_NOTHROW DirReadTracker;
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
#if AP_FILESYSTEM_HAVE_DIRENT_DTYPE
    dtracker->curr_file.d_type = DT_REG;
#endif
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
#if AP_CRASHDUMP_FLASH_ENABLED
    } else if (strcmp(pathname_noslash, "crash_dump.bin") == 0) {
        stbuf->st_size = hal.util->last_crash_dump_size();
#endif
    } else {
        stbuf->st_size = 100000;
    }
    return 0;
}

#endif  // AP_FILESYSTEM_SYS_ENABLED

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
  posix filesystem backend for SITL with a simulated disk size cap
 */
#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_SITL_ENABLED

#include "AP_Filesystem_SITL.h"
#include <SITL/SITL.h>

/*
  add the size of regular files in a directory to used_bytes. Only
  the logs directory is scanned as the SITL working directory holds
  many unrelated files (eeprom.bin, terrain, ...)
 */
void AP_Filesystem_SITL::rescan_used(const char *path)
{
    void *d = AP_Filesystem_Posix::opendir(path);
    if (d == nullptr) {
        return;
    }
    struct dirent *de;
    while ((de = AP_Filesystem_Posix::readdir(d)) != nullptr) {
        if (de->d_name[0] == '.') {
            continue;
        }
        char *child = nullptr;
        if (asprintf(&child, "%s/%s", path, de->d_name) < 0) {
            continue;
        }
        struct stat st;
        if (AP_Filesystem_Posix::stat(child, &st) == 0 && S_ISREG(st.st_mode)) {
            used_bytes += st.st_size;
        }
        ::free(child);
    }
    AP_Filesystem_Posix::closedir(d);
}

/*
  return SIM_DISK_MAX in bytes, or -1 when disabled. When the value
  changes the logs directory is rescanned so files from a previous
  run count against the cap
 */
int64_t AP_Filesystem_SITL::disk_max_bytes()
{
    auto *sitl = AP::sitl();
    if (sitl == nullptr) {
        return -1;
    }
    const int32_t mb = sitl->sim_disk_max_mb.get();
    if (mb <= 0) {
        last_max_mb = mb;
        return -1;
    }
    if (mb != last_max_mb) {
        used_bytes = 0;
        rescan_used(HAL_BOARD_LOG_DIRECTORY);
        last_max_mb = mb;
    }
    // decimal MB, matching AP_Logger's MB_to_B
    return (int64_t)mb * 1000000;
}

int32_t AP_Filesystem_SITL::write(int fd, const void *buf, uint32_t count)
{
    WITH_SEMAPHORE(sem);
    const int64_t cap = disk_max_bytes();
    if (cap >= 0 && used_bytes + (int64_t)count > cap) {
        errno = ENOSPC;
        return -1;
    }
    const int32_t ret = AP_Filesystem_Posix::write(fd, buf, count);
    if (ret > 0 && cap >= 0) {
        used_bytes += ret;
    }
    return ret;
}

int AP_Filesystem_SITL::unlink(const char *pathname)
{
    WITH_SEMAPHORE(sem);
    int64_t freed = 0;
    if (disk_max_bytes() >= 0) {
        struct stat st;
        if (AP_Filesystem_Posix::stat(pathname, &st) == 0 && S_ISREG(st.st_mode)) {
            freed = st.st_size;
        }
    }
    const int ret = AP_Filesystem_Posix::unlink(pathname);
    if (ret == 0) {
        used_bytes = MAX(used_bytes - freed, 0);
    }
    return ret;
}

int64_t AP_Filesystem_SITL::disk_free(const char *path)
{
    WITH_SEMAPHORE(sem);
    const int64_t cap = disk_max_bytes();
    if (cap < 0) {
        return AP_Filesystem_Posix::disk_free(path);
    }
    return MAX(cap - used_bytes, 0);
}

int64_t AP_Filesystem_SITL::disk_space(const char *path)
{
    WITH_SEMAPHORE(sem);
    const int64_t cap = disk_max_bytes();
    if (cap < 0) {
        return AP_Filesystem_Posix::disk_space(path);
    }
    return cap;
}

#endif  // AP_FILESYSTEM_SITL_ENABLED

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

#if HAVE_FILESYSTEM_SUPPORT

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

ssize_t AP_Filesystem_Sys::read(int fd, void *buf, size_t count)
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

ssize_t AP_Filesystem_Sys::write(int fd, const void *buf, size_t count)
{
    errno = EROFS;
    return -1;
}

int AP_Filesystem_Sys::fsync(int fd)
{
    return 0;
}

off_t AP_Filesystem_Sys::lseek(int fd, off_t offset, int seek_from)
{
    if (fd < 0 || fd >= max_open_file || !file[fd].open) {
        errno = EBADF;
        return -1;
    }
    struct rfile &r = file[fd];
    switch (seek_from) {
    case SEEK_SET:
        r.file_ofs = MIN(offset, r.data->length);
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

int AP_Filesystem_Sys::unlink(const char *pathname)
{
    errno = EROFS;
    return -1;
}

int AP_Filesystem_Sys::mkdir(const char *pathname)
{
    errno = EROFS;
    return -1;
}

void *AP_Filesystem_Sys::opendir(const char *pathname)
{
    errno = EINVAL;
    return nullptr;
}

struct dirent *AP_Filesystem_Sys::readdir(void *dirp)
{
    errno = EBADF;
    return nullptr;
}

int AP_Filesystem_Sys::closedir(void *dirp)
{
    errno = EBADF;
    return -1;
}

// return free disk space in bytes
int64_t AP_Filesystem_Sys::disk_free(const char *path)
{
    return 0;
}

// return total disk space in bytes
int64_t AP_Filesystem_Sys::disk_space(const char *path)
{
    return 0;
}

/*
  set mtime on a file
 */
bool AP_Filesystem_Sys::set_mtime(const char *filename, const time_t mtime_sec)
{
    return false;
}

#endif

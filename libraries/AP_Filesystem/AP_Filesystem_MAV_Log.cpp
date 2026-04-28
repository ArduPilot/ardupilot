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
#include "AP_Filesystem.h"
#include "AP_Filesystem_MAV_Log.h"

#if AP_FILESYSTEM_MAV_LOG_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <errno.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// return the active log directory, honouring a custom override
static const char *log_directory()
{
    const char *custom = hal.util->get_custom_log_directory();
    if (custom != nullptr) {
        return custom;
    }
    return HAL_BOARD_LOG_DIRECTORY;
}

// rewrite "foo/bar" to "<log_dir>/foo/bar" in the caller-supplied buffer.
// An empty fname resolves to the log directory itself.
// Returns false if the rewrite doesn't fit.
static bool rewrite_path(const char *fname, char *buf, size_t buflen)
{
    const char *dir = log_directory();
    int n;
    if (fname[0] == '\0') {
        n = snprintf(buf, buflen, "%s", dir);
    } else {
        n = snprintf(buf, buflen, "%s/%s", dir, fname);
    }
    if (n < 0 || size_t(n) >= buflen) {
        errno = ENAMETOOLONG;
        return false;
    }
    return true;
}

int AP_Filesystem_MAV_Log::open(const char *fname, int flags, bool allow_absolute_paths)
{
    if ((flags & O_ACCMODE) != O_RDONLY) {
        errno = EROFS;
        return -1;
    }
    char path[128];
    if (!rewrite_path(fname, path, sizeof(path))) {
        return -1;
    }
    return AP::FS().local_backend().open(path, flags, true);
}

int AP_Filesystem_MAV_Log::close(int fd)
{
    return AP::FS().local_backend().close(fd);
}

int32_t AP_Filesystem_MAV_Log::read(int fd, void *buf, uint32_t count)
{
    return AP::FS().local_backend().read(fd, buf, count);
}

int32_t AP_Filesystem_MAV_Log::lseek(int fd, int32_t offset, int whence)
{
    return AP::FS().local_backend().lseek(fd, offset, whence);
}

int AP_Filesystem_MAV_Log::stat(const char *pathname, struct stat *stbuf)
{
    char path[128];
    if (!rewrite_path(pathname, path, sizeof(path))) {
        return -1;
    }
    return AP::FS().local_backend().stat(path, stbuf);
}

void *AP_Filesystem_MAV_Log::opendir(const char *pathname)
{
    char path[128];
    if (!rewrite_path(pathname, path, sizeof(path))) {
        return nullptr;
    }
    return AP::FS().local_backend().opendir(path);
}

struct dirent *AP_Filesystem_MAV_Log::readdir(void *dirp)
{
    return AP::FS().local_backend().readdir(dirp);
}

int AP_Filesystem_MAV_Log::closedir(void *dirp)
{
    return AP::FS().local_backend().closedir(dirp);
}

#endif  // AP_FILESYSTEM_MAV_LOG_ENABLED

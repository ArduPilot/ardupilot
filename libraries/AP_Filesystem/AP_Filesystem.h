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
  ArduPilot filesystem interface. This offsets a minimal subset of
  full functionality offered by posix type interfaces, meeting the
  needs of ArduPilot
 */
#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_Filesystem_Available.h"

#if HAVE_FILESYSTEM_SUPPORT
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "AP_Filesystem_FATFS.h"
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_Filesystem_posix.h"
#endif

class AP_Filesystem {

public:
    AP_Filesystem() {}

    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags);
    int close(int fd);
    ssize_t read(int fd, void *buf, size_t count);
    ssize_t write(int fd, const void *buf, size_t count);
    int fsync(int fd);
    off_t lseek(int fd, off_t offset, int whence);
    int stat(const char *pathname, struct stat *stbuf);
    int unlink(const char *pathname);
    int mkdir(const char *pathname);
    DIR *opendir(const char *pathname);
    struct dirent *readdir(DIR *dirp);
    int closedir(DIR *dirp);

    // return free disk space in bytes, -1 on error
    int64_t disk_free(const char *path);

    // return total disk space in bytes, -1 on error
    int64_t disk_space(const char *path);

    // set modification time on a file
    bool set_mtime(const char *filename, const time_t mtime_sec);
};

namespace AP {
    AP_Filesystem &FS();
};
#endif // HAVE_FILESYSTEM_SUPPORT

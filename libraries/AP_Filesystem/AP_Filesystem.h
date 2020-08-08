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

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#if HAVE_FILESYSTEM_SUPPORT
#include "AP_Filesystem_FATFS.h"
#endif
#define DT_REG 0
#define DT_DIR 1
#if defined(FF_MAX_LFN) && FF_USE_LFN != 0
#define MAX_NAME_LEN FF_MAX_LFN 
#else
#define MAX_NAME_LEN 13
#endif
struct dirent {
   char    d_name[MAX_NAME_LEN]; /* filename */
   uint8_t d_type;
};
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#endif // HAL_BOARD_CHIBIOS
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_Filesystem_posix.h"
#endif

#include "AP_Filesystem_backend.h"

class AP_Filesystem {
private:
    struct DirHandle {
        uint8_t fs_index;
        void *dir;
    };

public:
    AP_Filesystem() {}

    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags);
    int close(int fd);
    int32_t read(int fd, void *buf, uint32_t count);
    int32_t write(int fd, const void *buf, uint32_t count);
    int fsync(int fd);
    int32_t lseek(int fd, int32_t offset, int whence);
    int stat(const char *pathname, struct stat *stbuf);
    int unlink(const char *pathname);
    int mkdir(const char *pathname);

    DirHandle *opendir(const char *pathname);
    struct dirent *readdir(DirHandle *dirp);
    int closedir(DirHandle *dirp);

    // return free disk space in bytes, -1 on error
    int64_t disk_free(const char *path);

    // return total disk space in bytes, -1 on error
    int64_t disk_space(const char *path);

    // set modification time on a file
    bool set_mtime(const char *filename, const uint32_t mtime_sec);

private:
    struct Backend {
        const char *prefix;
        AP_Filesystem_Backend &fs;
    };
    static const struct Backend backends[];

    /*
      find backend by path
     */
    const Backend &backend_by_path(const char *&path) const;

    /*
      find backend by open fd
     */
    const Backend &backend_by_fd(int &fd) const;
};

namespace AP {
    AP_Filesystem &FS();
};


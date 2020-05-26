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
  ArduPilot filesystem backend interface.
 */
#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_Filesystem_Available.h"

class AP_Filesystem_Backend {

public:
    // functions that closely match the equivalent posix calls
    virtual int open(const char *fname, int flags) {
        return -1;
    }
    virtual int close(int fd) { return -1; }
    virtual int32_t read(int fd, void *buf, uint32_t count) { return -1; }
    virtual int32_t write(int fd, const void *buf, uint32_t count) { return -1; }
    virtual int fsync(int fd) { return 0; }
    virtual int32_t lseek(int fd, int32_t offset, int whence) { return -1; }
    virtual int stat(const char *pathname, struct stat *stbuf) { return -1; }
    virtual int unlink(const char *pathname) { return -1; }
    virtual int mkdir(const char *pathname) { return -1; }
    virtual void *opendir(const char *pathname) { return nullptr; }
    virtual struct dirent *readdir(void *dirp) { return nullptr; }
    virtual int closedir(void *dirp) { return -1; }

    // return free disk space in bytes, -1 on error
    virtual int64_t disk_free(const char *path) { return 0; }

    // return total disk space in bytes, -1 on error
    virtual int64_t disk_space(const char *path) { return 0; }

    // set modification time on a file
    virtual bool set_mtime(const char *filename, const uint32_t mtime_sec) { return false; }
};

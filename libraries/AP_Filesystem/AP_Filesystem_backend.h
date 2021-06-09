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

// returned structure from a load_file() call
class FileData {
public:
    uint32_t length;
    const uint8_t *data;

    FileData(void *_backend) :
        backend(_backend) {}
    
    // destructor to free data
    ~FileData();
private:
    const void *backend;
};

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

    // retry mount of filesystem if needed
    virtual bool retry_mount(void) { return true; }

    // unmount filesystem for reboot
    virtual void unmount(void) {}

    /*
      load a full file. Use delete to free the data
     */
    virtual FileData *load_file(const char *filename);

    // unload data from load_file()
    virtual void unload_file(FileData *fd);

protected:
    // return true if file operations are allowed
    bool file_op_allowed(void) const;
};

#define FS_CHECK_ALLOWED(retfail) do { if (!file_op_allowed()) { return retfail; } } while(0)

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

#include "AP_Filesystem_config.h"

#ifndef MAX_NAME_LEN
#define MAX_NAME_LEN 255
#endif

#if (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS) || (CONFIG_HAL_BOARD == HAL_BOARD_ESP32)
#define DT_REG 0
#define DT_DIR 1
#define DT_LNK 10
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#if AP_FILESYSTEM_FATFS_ENABLED
#include "AP_Filesystem_FATFS.h"
#endif
#if AP_FILESYSTEM_LITTLEFS_ENABLED
#include "AP_Filesystem_FlashMemory_LittleFS.h"
#endif

struct dirent {
   char    d_name[MAX_NAME_LEN]; /* filename */
   uint8_t d_type;
};

#endif // HAL_BOARD_CHIBIOS

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#ifndef AP_FILESYSTEM_FORMAT_ENABLED
#define AP_FILESYSTEM_FORMAT_ENABLED 1
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include "AP_Filesystem_posix.h"
#if AP_FILESYSTEM_LITTLEFS_ENABLED
#include "AP_Filesystem_FlashMemory_LittleFS.h"
#endif
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#include "AP_Filesystem_ESP32.h"
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
    int open(const char *fname, int flags, bool allow_absolute_paths = false);
    int close(int fd);
    int32_t read(int fd, void *buf, uint32_t count);
    int32_t write(int fd, const void *buf, uint32_t count);
    int fsync(int fd);
    int32_t lseek(int fd, int32_t offset, int whence);
    int stat(const char *pathname, struct stat *stbuf);

    // stat variant for scripting
    typedef struct Stat {
        uint32_t size;
        int32_t mode;
        uint32_t mtime;
        uint32_t atime;
        uint32_t ctime;
        bool is_directory(void) const {
            return (mode & S_IFMT) == S_IFDIR;
        }
    } stat_t;
    bool stat(const char *pathname, stat_t &stbuf);

    int unlink(const char *pathname);
    int mkdir(const char *pathname);
    int rename(const char *oldpath, const char *newpath);

    DirHandle *opendir(const char *pathname);
    struct dirent *readdir(DirHandle *dirp);
    int closedir(DirHandle *dirp);

    // return number of bytes that should be written before fsync for optimal
    // streaming performance/robustness. if zero, any number can be written.
    uint32_t bytes_until_fsync(int fd);

    // return free disk space in bytes, -1 on error
    int64_t disk_free(const char *path);

    // return total disk space in bytes, -1 on error
    int64_t disk_space(const char *path);

    // set modification time on a file
    bool set_mtime(const char *filename, const uint32_t mtime_sec);

    // if filesystem is not running then try a remount. Return true if fs is mounted
    bool retry_mount(void);

    // unmount filesystem for reboot
    void unmount(void);

    // returns null-terminated string; cr or lf terminates line
    bool fgets(char *buf, uint8_t buflen, int fd);

    // run crc32 over file with given name, returns true if successful
    bool crc32(const char *fname, uint32_t& checksum) WARN_IF_UNUSED;

    // format filesystem.  This is async, monitor get_format_status for progress
    bool format(void);

    // retrieve status of format process:
    AP_Filesystem_Backend::FormatStatus get_format_status() const;

    /*
      Load a file's contents into memory. Returned object must be `delete`d to
      free the data. The data is guaranteed to be null-terminated such that it
      can be treated as a string.
     */
    FileData *load_file(const char *filename);

    // get_singleton for scripting
    static AP_Filesystem *get_singleton(void);

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

    // support for listing out virtual directory entries (e.g. @SYS
    // then @MISSION)
    struct {
        uint8_t backend_ofs;
        struct dirent de;
        uint8_t d_off;
    } virtual_dirent;
};

namespace AP {
    AP_Filesystem &FS();
};


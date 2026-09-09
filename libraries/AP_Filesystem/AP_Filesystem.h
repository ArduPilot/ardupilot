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

#if AP_FILESYSTEM_ALIAS_ENABLED
#include <AP_HAL/Semaphores.h>
#endif

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

// used by LittleFS
#define AP_FILESYSTEM_FLASH_JEDEC_NOR 1
#define AP_FILESYSTEM_FLASH_W25NXX 2

#ifndef AP_FILESYSTEM_LITTLEFS_FLASH_TYPE
#define AP_FILESYSTEM_LITTLEFS_FLASH_TYPE AP_FILESYSTEM_FLASH_JEDEC_NOR
#endif

#ifndef AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC
// this requires AP_FILESYSTEM_POSIX_MAP_FILENAME_BASEDIR
#define AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC 0
#endif

#ifndef AP_FILESYSTEM_HAVE_DIRENT_DTYPE
#define AP_FILESYSTEM_HAVE_DIRENT_DTYPE 1
#endif

#ifndef AP_FATFS_MAX_IO_SIZE
#define AP_FATFS_MAX_IO_SIZE 4096
#endif

#ifndef AP_FATFS_MIN_IO_SIZE
#define AP_FATFS_MIN_IO_SIZE 4096
#endif

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

    // reads a line into buf, guaranteeing null-termination.  buflen
    // is the full size of buf, including the space required for the
    // null terminator, so up to buflen-1 characters are returned.  cr
    // or lf terminates the line and is consumed but not returned.
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
        // if set, prefix is an alias for this directory in fs; a function as
        // the directory may only be known at runtime
        const char *(*root)(void);
    };
    static const struct Backend backends[];

    /*
      find backend by path
     */
    const Backend &backend_by_path(const char *&path) const;

    // a path resolved to its backend; an alias's rewritten path lives only as long as this does
    class ResolvedPath {
    public:
        // SHARED uses alias_path under alias_sem; OWN allocates, for a second path in one call
        enum class Buffer : uint8_t {
            SHARED,
            OWN,
        };

        ResolvedPath(AP_Filesystem &filesystem, const char *path, Buffer buffer=Buffer::SHARED);
        ~ResolvedPath();
        CLASS_NO_COPY(ResolvedPath);

        // false (with errno set) if an alias path could not be built; the caller must fail
        bool valid(void) const { return _path != nullptr; }

        const Backend &backend(void) const { return *_backend; }
        const char *path(void) const { return _path; }

    private:
        const Backend *_backend;
        const char *_path;
#if AP_FILESYSTEM_ALIAS_ENABLED
        AP_Filesystem &_filesystem;
        char *_own_buffer;
        bool _holds_shared_buffer;
#endif
    };

#if AP_FILESYSTEM_ALIAS_ENABLED
    // allocated on first alias use and kept
    char *alias_path = nullptr;

    /*
      WARNING: held across the whole backend call using alias_path.  if that
      IO never returns, this is never given back and every later alias call
      blocks forever
     */
    HAL_Semaphore alias_sem;

    // alias_sem is recursive; this stops its holder overwriting alias_path in use
    bool alias_path_in_use = false;
#endif

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


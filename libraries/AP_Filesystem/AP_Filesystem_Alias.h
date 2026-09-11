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
  ArduPilot filesystem alias: one prefix standing for a directory in another
  filesystem, in the manner of a symbolic link
 */
#pragma once

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_ALIAS_ENABLED

#include "AP_Filesystem_backend.h"

/*
  A filesystem which is nothing but another name for a directory in a second
  filesystem: every path has the alias' root put on the front of it and is
  then handed to that filesystem. It exists so that a prefix like @MAV_LOG can
  name a directory whose real location differs from board to board, without a
  GCS having to know where that is.

  The root is fetched through a function rather than stored, so an alias can
  follow a directory which is only settled at runtime.
 */
class AP_Filesystem_Alias : public AP_Filesystem_Backend
{
public:
    AP_Filesystem_Alias(AP_Filesystem_Backend &_target, const char *(*_root)(void)) :
        target(_target), root(_root) {}

    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    int close(int fd) override;
    int32_t read(int fd, void *buf, uint32_t count) override;
    int32_t write(int fd, const void *buf, uint32_t count) override;
    int fsync(int fd) override;
    int32_t lseek(int fd, int32_t offset, int whence) override;
    int stat(const char *pathname, struct stat *stbuf) override;
    int unlink(const char *pathname) override;
    int mkdir(const char *pathname) override;
    void *opendir(const char *pathname) override;
    struct dirent *readdir(void *dirp) override;
    int closedir(void *dirp) override;
    int rename(const char *oldpath, const char *newpath) override;
    int64_t disk_free(const char *path) override;
    int64_t disk_space(const char *path) override;
    bool set_mtime(const char *filename, const uint32_t mtime_sec) override;

private:
    // the filesystem which does all of the work
    AP_Filesystem_Backend &target;

    // the directory in that filesystem this alias stands for
    const char *(*root)(void);

    // longest path we can build; this is also the longest an FTP request can
    // carry, which is where these paths come from
    static const uint8_t max_path_len = 255;
};

#endif  // AP_FILESYSTEM_ALIAS_ENABLED

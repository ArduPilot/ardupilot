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

#pragma once

#include "AP_Filesystem_backend.h"

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_SYS_ENABLED

class ExpandingString;

class AP_Filesystem_Sys : public AP_Filesystem_Backend
{
public:
    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    int close(int fd) override;
    int32_t read(int fd, void *buf, uint32_t count) override;
    int32_t lseek(int fd, int32_t offset, int whence) override;
    int stat(const char *pathname, struct stat *stbuf) override;
    void *opendir(const char *pathname) override;
    struct dirent *readdir(void *dirp) override;
    int closedir(void *dirp) override;

private:
    // only allow up to 4 files at a time
    static constexpr uint8_t max_open_file = 4;
    int8_t file_in_sysfs(const char *fname);

    struct DirReadTracker {
        size_t file_offset;
        struct dirent curr_file;
    };

    struct rfile {
        bool open;
        uint32_t file_ofs;
        ExpandingString *str;
    } file[max_open_file];
};

#endif  // AP_FILESYSTEM_SYS_ENABLED

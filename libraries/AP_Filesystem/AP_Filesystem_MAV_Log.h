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

#if AP_FILESYSTEM_MAV_LOG_ENABLED

/*
  Virtual path "@MAV_LOG" that points at the active log directory.
  All operations are delegated to the local filesystem backend after
  rewriting the path. The view is read-only so GCS tools can download
  logs over MAVLink FTP without needing to know the board-specific
  log directory path.
 */
class AP_Filesystem_MAV_Log : public AP_Filesystem_Backend
{
public:
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    int close(int fd) override;
    int32_t read(int fd, void *buf, uint32_t count) override;
    int32_t lseek(int fd, int32_t offset, int whence) override;
    int stat(const char *pathname, struct stat *stbuf) override;
    void *opendir(const char *pathname) override;
    struct dirent *readdir(void *dirp) override;
    int closedir(void *dirp) override;
};

#endif  // AP_FILESYSTEM_MAV_LOG_ENABLED

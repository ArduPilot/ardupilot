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

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_DRONECAN_ENABLED

#include "AP_Filesystem_backend.h"

class AP_DroneCAN;
class AP_Filesystem_DroneCAN : public AP_Filesystem_Backend
{
public:
    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
    int close(int fd) override;
    int32_t read(int fd, void *buf, uint32_t count) override;
    int32_t write(int fd, const void *buf, uint32_t count) override;
    int32_t lseek(int fd, int32_t offset, int whence) override;
    int stat(const char *pathname, struct stat *stbuf) override;
    int unlink(const char *pathname) override;
    int mkdir(const char *pathname) override;
    void *opendir(const char *pathname) override;
    struct dirent *readdir(void *dirp) override;
    int closedir(void *dirp) override;

private:

    // Get driver, return pointer to remainder of file path
    // nullptr if invalid
    char* populate_driver(const char *fname, AP_DroneCAN*& driver);

    // Get node id, return pointer to remainder of file path
    // nullptr if invalid
    char* populate_node_id(const char *fname, const AP_DroneCAN* driver, uint8_t& node_id);

    // Get both driver and node id, return pointer to remainder of file path
    // nullptr if invalid
    char* populate_driver_and_node_id(const char *fname, AP_DroneCAN*& driver, uint8_t& node_id);

    // Get file stat from remote node
    int node_stat(AP_DroneCAN* driver, uint8_t node_id, const char *node_path, struct stat *stbuf);

    // only allow up to 4 files at a time
    static constexpr uint8_t max_open_file = 4;
    static constexpr uint8_t max_open_dir = 4;
    struct rfile {
        char *path;
        AP_DroneCAN *driver;
        uint8_t node_id;
        uint32_t size;
        uint32_t ofs;
    } file[max_open_file];

    // allow up to 4 directory opens
    struct rdir {
        char *path;
        char *node_path;
        AP_DroneCAN *driver;
        uint8_t node_id;
        uint16_t ofs;
        struct dirent de;
    } dir[max_open_dir];

};

#endif  // AP_FILESYSTEM_DRONECAN_ENABLED

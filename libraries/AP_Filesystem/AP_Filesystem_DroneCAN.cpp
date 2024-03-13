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
  ArduPilot filesystem interface for DroneCAN
 */

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_DRONECAN_ENABLED

#include "AP_Filesystem.h"
#include "AP_Filesystem_DroneCAN.h"
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// Timeout for DroneCAN response
#define TIMEOUT_MS 250

#ifdef ENOTBLK
    #define MAIN_THREAD_ERROR ENOTBLK;
#else
    #define MAIN_THREAD_ERROR EPERM;
#endif

extern const AP_HAL::HAL& hal;

int AP_Filesystem_DroneCAN::open(const char *fname, int flags, bool allow_absolute_paths)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    // Find free file object
    uint8_t idx;
    for (idx=0; idx<max_open_file; idx++) {
        if (file[idx].path == nullptr) {
            break;
        }
    }
    if (idx == max_open_file) {
        errno = ENFILE;
        return -1;
    }
    if (file[idx].path != nullptr) {
        errno = EBUSY;
        return -1;
    }

    // Try and pull driver and node ID from file path
    char* node_path = populate_driver_and_node_id(fname, file[idx].driver, file[idx].node_id);
    if (node_path == nullptr) {
        errno = ENOENT;
        return -1;
    }

    // Get stats and check its a file not a directory
    struct stat st;
    uint32_t size = 0;
    if (node_stat(file[idx].driver, file[idx].node_id, node_path, &st) == 0) {
        // Got a stat result
        if ((st.st_mode & S_IFMT) == S_IFDIR) {
            // Can't open directory as file
            errno = ENOENT;
            return -1;
        }
        size = st.st_size;
    } else if ((flags & O_CREAT) == 0) {
        // No stat result and not trying to create a file
        errno = ENOENT;
        return -1;
    }

    // Setup file obj
    file[idx].path = strdup(node_path);
    if (file[idx].path == nullptr) {
        // strdup failed
        errno = ENOMEM;
        return -1;
    }

    // Set offset to size if appending
    file[idx].ofs = (flags & O_APPEND) ? size : 0;
    file[idx].size = size;
    return idx;
}

int AP_Filesystem_DroneCAN::close(int fd)
{
    // Check valid file
    if ((fd < 0) || (fd >= max_open_file) || (file[fd].path == nullptr)) {
        errno = EBADF;
        return -1;
    }

    // DroneCAN does not support telling the remote to close
    free(file[fd].path);
    file[fd].path = nullptr;
    return 0;
}

int32_t AP_Filesystem_DroneCAN::read(int fd, void *buf, uint32_t count)
{
    // Check valid file
    if ((fd < 0 ) || (fd >= max_open_file)) {
        errno = EBADF;
        return -1;
    }

    // Check valid driver and node
    if ((file[fd].path == nullptr) || (file[fd].driver == nullptr) || !file[fd].driver->seen_node_id(file[fd].node_id)) {
        errno = ENOENT;
        return -1;
    }

    // Request from node
    file[fd].driver->filesystem.request_read(file[fd].path, file[fd].node_id, file[fd].ofs);

    uavcan_protocol_file_ReadResponse msg {};

    // Wait for response
    const uint32_t wait_start = AP_HAL::millis();
    do {
        if (!file[fd].driver->filesystem.request_read_response(msg)) {
            hal.scheduler->delay(1);
            continue;
        }

        // Got response
        if (msg.error.value != 0) {
            // DroneCAN follows same error convention
            errno = msg.error.value;
            return -1;
        }

        // Copy and return data
        count = MIN(count, msg.data.len);
        memcpy(buf, &msg.data.data, count);
        file[fd].ofs += count;
        return count;

    } while ((AP_HAL::millis() - wait_start) < TIMEOUT_MS);

    // Time out
    errno = EBUSY;
    return -1;
}

int32_t AP_Filesystem_DroneCAN::write(int fd, const void *buf, uint32_t count)
{
    // Check valid file
    if ((fd < 0 ) || (fd >= max_open_file)) {
        errno = EBADF;
        return -1;
    }

    // Check valid driver and node
    if ((file[fd].path == nullptr) || (file[fd].driver == nullptr) || !file[fd].driver->seen_node_id(file[fd].node_id)) {
        errno = ENOENT;
        return -1;
    }

    // Request from node
    count = file[fd].driver->filesystem.request_write(file[fd].path, file[fd].node_id, file[fd].ofs, buf, count);

    uavcan_protocol_file_WriteResponse msg {};

    // Wait for response
    const uint32_t wait_start = AP_HAL::millis();
    do {
        if (!file[fd].driver->filesystem.request_write_response(msg)) {
            hal.scheduler->delay(1);
            continue;
        }

        // Got response
        if (msg.error.value != 0) {
            // DroneCAN follows same error convention
            errno = msg.error.value;
            return -1;
        }

        // Return length written
        file[fd].ofs += count;
        file[fd].size = MAX(file[fd].size, file[fd].ofs);
        return count;

    } while ((AP_HAL::millis() - wait_start) < TIMEOUT_MS);

    // Time out
    errno = EBUSY;
    return -1;
}

int32_t AP_Filesystem_DroneCAN::lseek(int fd, int32_t offset, int seek_from)
{
    if ((fd < 0) || (fd >= max_open_file) || (file[fd].path == nullptr)) {
        // Invalid file
        errno = EBADF;
        return -1;
    }

    // Just move the local copy of the offset, there is no DroneCAN command for seek
    // This means that the next read starts in the correct spot
    switch (seek_from) {
    case SEEK_SET:
        if (offset < 0) {
            errno = EINVAL;
            return -1;
        }
        file[fd].ofs = MIN(file[fd].size, (uint32_t)offset);
        break;
    case SEEK_CUR:
        file[fd].ofs = MIN(file[fd].size, offset+file[fd].ofs);
        break;
    case SEEK_END:
        file[fd].ofs = file[fd].size;
        break;
    }
    return file[fd].ofs;
}

int AP_Filesystem_DroneCAN::stat(const char *name, struct stat *stbuf)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    AP_DroneCAN *driver = nullptr;
    uint8_t node_id;
    char* node_path = populate_driver_and_node_id(name, driver, node_id);
    if (node_path == nullptr) {
        errno = ENOENT;
        return -1;
    }

    return node_stat(driver, node_id, node_path, stbuf);
}

// Get file stat from remote node
int AP_Filesystem_DroneCAN::node_stat(AP_DroneCAN* driver, uint8_t node_id, const char *node_path, struct stat *stbuf)
{
    if ((driver == nullptr) || !driver->seen_node_id(node_id)) {
        errno = ENOENT;
        return -1;
    }

    // Request from node
    driver->filesystem.request_info(node_path, node_id);

    uavcan_protocol_file_GetInfoResponse msg {};

    // Wait for response
    const uint32_t wait_start = AP_HAL::millis();
    do {
        if (!driver->filesystem.request_info_response(msg)) {
            hal.scheduler->delay(1);
            continue;
        }

        // Got response
        if (msg.error.value != 0) {
            // DroneCAN follows same error convention
            errno = msg.error.value;
            return -1;
        }

        stbuf->st_size = msg.size;
        stbuf->st_mode = 0;
        if (msg.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE) {
            stbuf->st_mode |= S_IFREG;
        }
        if (msg.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY) {
            stbuf->st_mode |= S_IFDIR;
        }
        if (msg.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_SYMLINK) {
            stbuf->st_mode |= S_IFLNK;
        }
        if (msg.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_READABLE) {
            stbuf->st_mode |= S_IREAD;
        }
        if (msg.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_WRITEABLE) {
            stbuf->st_mode |= S_IWRITE;
        }
        return 0;

    } while ((AP_HAL::millis() - wait_start) < TIMEOUT_MS);

    // Time out
    errno = EBUSY;
    return -1;
}

int AP_Filesystem_DroneCAN::unlink(const char *pathname)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    AP_DroneCAN *driver = nullptr;
    uint8_t node_id;
    char* node_path = populate_driver_and_node_id(pathname, driver, node_id);
    if (node_path == nullptr) {
        errno = ENOENT;
        return -1;
    }

    // Request from node
    driver->filesystem.request_delete(node_path, node_id);

    uavcan_protocol_file_DeleteResponse msg {};

    // Wait for response
    const uint32_t wait_start = AP_HAL::millis();
    do {
        if (!driver->filesystem.request_delete_response(msg)) {
            hal.scheduler->delay(1);
            continue;
        }

        // Got response
        if (msg.error.value != 0) {
            // DroneCAN follows same error convention
            errno = msg.error.value;
            return -1;
        }
        return 0;

    } while ((AP_HAL::millis() - wait_start) < TIMEOUT_MS);

    // Time out
    errno = EBUSY;
    return -1;
}

int AP_Filesystem_DroneCAN::mkdir(const char *pathname)
{
    // DroneCAN protocol does not support directory creation
    errno = ENOSYS;
    return -1;
}

void *AP_Filesystem_DroneCAN::opendir(const char *pathname)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return nullptr;
    }

    // Find free object
    uint8_t idx;
    for (idx=0; idx<max_open_dir; idx++) {
        if (dir[idx].path == nullptr) {
            break;
        }
    }
    if (idx == max_open_dir) {
        // No free objects
        errno = ENFILE;
        return nullptr;
    }

    // Init object
    dir[idx].ofs = 0;
    dir[idx].node_id = UINT8_MAX;
    dir[idx].path = strdup(pathname);
    if (!dir[idx].path) {
        // Strdup failed
        errno = ENOMEM;
        return nullptr;
    }

    size_t len = strlen(pathname);
    if (len == 0 || ((len == 1) && (pathname[0] == '/'))) {
        // Top level, always allow open
        return (void*)&dir[idx];
    }

    char* driver_end = populate_driver(pathname, dir[idx].driver);
    if (driver_end == nullptr) {
        // Invalid driver
        closedir(&dir[idx]);
        errno = ENOENT;
        return nullptr;
    }

    len = strlen(driver_end);
    if ((len == 0) || ((len == 1) && (driver_end[0] == '/'))) {
        // Node ID level, always allow open
        return (void*)&dir[idx];
    }

    dir[idx].node_path = populate_node_id(driver_end, dir[idx].driver, dir[idx].node_id);
    if (dir[idx].node_path == nullptr) {
        // Invalid node id
        closedir(&dir[idx]);
        errno = ENOENT;
        return nullptr;
    }

    // If no path then use "/"
    const char *node_path = (strlen(dir[idx].node_path) == 0) ? "/" : dir[idx].node_path;

    // Get stat from node
    struct stat st;
    if (node_stat(dir[idx].driver, dir[idx].node_id, node_path, &st) != 0) {
        // no such directory
        closedir(&dir[idx]);
        errno = ENOENT;
        return nullptr;
    }

    if ((st.st_mode & S_IFMT) != S_IFDIR) {
        // A file not directory
        closedir(&dir[idx]);
        errno = ENOTDIR;
        return nullptr;
    }

    // Valid directory
    return (void*)&dir[idx];
}

struct dirent *AP_Filesystem_DroneCAN::readdir(void *dirp)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        // This really should not happen, because open dir should fail, could panic here
        closedir(dirp);
        errno = MAIN_THREAD_ERROR;
        return nullptr;
    }

    uint32_t idx = ((rdir*)dirp) - &dir[0];
    if (idx >= max_open_dir) {
        // Invalid rdir obj
        errno = EBADF;
        return nullptr;
    }

    if (dir[idx].driver == nullptr) {
        // Assume top level rather than the driver disappearing
        // Return available drivers
        const uint8_t can_num_drivers = AP::can().get_num_drivers();
        for (; dir[idx].ofs < can_num_drivers; dir[idx].ofs++) {
            if (AP::can().get_driver_type(dir[idx].ofs) == AP_CAN::Protocol::DroneCAN) {
                snprintf(dir[idx].de.d_name, sizeof(dir[idx].de.d_name), "%u", dir[idx].ofs);
                dir[idx].de.d_type = DT_DIR;
                dir[idx].ofs++;
                return &dir[idx].de;
            }
        }
        // Reached end of available drivers
        return nullptr;
    }

    if (dir[idx].node_id == UINT8_MAX) {
        // Return available nodes
        for (; dir[idx].ofs < UINT8_MAX; dir[idx].ofs++) {
            if (dir[idx].driver->seen_node_id(dir[idx].ofs)) {
                snprintf(dir[idx].de.d_name, sizeof(dir[idx].de.d_name), "%u", dir[idx].ofs);
                dir[idx].de.d_type = DT_DIR;
                dir[idx].ofs++;
                return &dir[idx].de;
            }
        }
        // Reached end of available nodes
        return nullptr;
    }

    // If no path then use "/"
    const char *node_path = (strlen(dir[idx].node_path) == 0) ? "/" : dir[idx].node_path;

    // Request from node
    dir[idx].driver->filesystem.request_directory_entry_info(node_path, dir[idx].node_id, dir[idx].ofs);

    uavcan_protocol_file_GetDirectoryEntryInfoResponse msg {};

    // Wait for response
    const uint32_t wait_start = AP_HAL::millis();
    do {
        hal.scheduler->delay(10);
        if (!dir[idx].driver->filesystem.request_get_directory_entry_info_response(msg)) {
            continue;
        }

        // Got response
        if (msg.error.value != 0) {
            // DroneCAN follows same error convention
            errno = msg.error.value;
            return nullptr;
        }

        // Translate type
        if (msg.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE) {
            dir[idx].de.d_type = DT_REG;
        } else if (msg.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_DIRECTORY) {
            dir[idx].de.d_type = DT_DIR;
        } else if (msg.entry_type.flags & UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_SYMLINK) {
            dir[idx].de.d_type = DT_LNK;
        } else {
            errno = EBADF;
            return nullptr;
        }

        // Copy name
        strncpy(dir[idx].de.d_name, (char*)msg.entry_full_path.path.data, sizeof(dir[idx].de.d_name));

        dir[idx].ofs++;
        return &dir[idx].de;

    } while ((AP_HAL::millis() - wait_start) < TIMEOUT_MS);

    // Time out
    errno = EBUSY;
    return nullptr;
}

int AP_Filesystem_DroneCAN::closedir(void *dirp)
{
    // DroneCAN does not support closing on the remote, just close local
    uint32_t idx = ((rdir *)dirp) - &dir[0];
    if (idx >= max_open_dir) {
        errno = EBADF;
        return -1;
    }
    free(dir[idx].path);
    dir[idx].path = nullptr;
    dir[idx].node_path = nullptr;
    dir[idx].driver = nullptr;
    dir[idx].node_id = UINT8_MAX;
    return 0;
}

char* AP_Filesystem_DroneCAN::populate_driver(const char *fname, AP_DroneCAN*& driver)
{
    // Remove starting /
    if ((strlen(fname) >= 1) && (fname[0] == '/')) {
        fname += 1;
    }

    char* driver_end;
    const uint32_t driver_index = strtoul(fname, &driver_end, 10);
    const bool strtoul_fail = (driver_end == nullptr) || (fname == driver_end);

    if (strtoul_fail || (driver_index > AP::can().get_num_drivers()) || (AP::can().get_driver_type(driver_index) != AP_CAN::Protocol::DroneCAN)) {
        return nullptr;
    }

    driver = AP_DroneCAN::get_dronecan(driver_index);
    if (driver == nullptr) {
        // This really should not happen, we just checked the index is valid
        return nullptr;
    }

    return driver_end;
}

char* AP_Filesystem_DroneCAN::populate_node_id(const char *fname, const AP_DroneCAN* driver, uint8_t& node_id)
{
    // Remove starting /
    if ((strlen(fname) >= 1) && (fname[0] == '/')) {
        fname += 1;
    }

    char* node_end;
    const uint32_t tmp_node_id = strtoul(fname, &node_end, 10);
    const bool strtoul_fail = (node_end == nullptr) || (fname == node_end);
    const bool invalid_id = (driver == nullptr) || (tmp_node_id > UINT8_MAX);
    if (strtoul_fail || invalid_id || !driver->seen_node_id(tmp_node_id)) {
        return nullptr;
    }

    node_id = tmp_node_id;
    return node_end;
}

char* AP_Filesystem_DroneCAN::populate_driver_and_node_id(const char *fname, AP_DroneCAN*& driver, uint8_t& node_id)
{
    char* driver_end = populate_driver(fname, driver);
    if (driver_end == nullptr) {
        return nullptr;
    }
    return populate_node_id(driver_end, driver, node_id);
}

#endif // AP_FILESYSTEM_DRONECAN_ENABLED

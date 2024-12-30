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
#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_P92000_ENABLED

#include "AP_Filesystem.h"
#include "AP_Filesystem_9P2000.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

#ifdef ENOTBLK
    #define MAIN_THREAD_ERROR ENOTBLK;
#else
    #define MAIN_THREAD_ERROR EPERM;
#endif

// Wait for response, blocking
bool AP_Filesystem_9P2000::wait_for_tag(AP_Networking::NineP2000& fs, const uint16_t tag) const
{
    // Tag is invalid
    if (tag == fs.NOTAG) {
        errno = ENFILE;
        return false;
    }

    const uint32_t timeout = 250;
    const uint32_t wait_start = AP_HAL::millis();
    do {
        if (fs.tag_response(tag)) {
            return true;
        }
        hal.scheduler->delay(1);
    } while ((AP_HAL::millis() - wait_start) < timeout);

    // Timeout!
    fs.clear_tag(tag);
    errno = EBUSY;
    return false;
}

// Open a given file ID with flags
bool AP_Filesystem_9P2000::open_fileId(AP_Networking::NineP2000& fs, const uint32_t fileId, const int flags)
{
    // Request the open
    const uint16_t tag = fs.request_open(fileId, flags);

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        return false;
    }

    // Got reply
    return fs.open_result(tag);
}

int AP_Filesystem_9P2000::open(const char *fname, int flags, bool allow_absolute_paths)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    return -1;
}

int AP_Filesystem_9P2000::close(int fd)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    return -1;
}

int32_t AP_Filesystem_9P2000::read(int fd, void *buf, uint32_t count)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    return -1;
}

int32_t AP_Filesystem_9P2000::write(int fd, const void *buf, uint32_t count)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    return -1;
}

int32_t AP_Filesystem_9P2000::lseek(int fd, int32_t offset, int seek_from)
{
    if ((fd < 0) || (fd >= max_open_file) || (file[fd].path == nullptr)) {
        // Invalid file
        errno = EBADF;
        return -1;
    }

    // Just move the local copy of the offset, there is no 9P2000 command for seek
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

int AP_Filesystem_9P2000::stat(const char *name, struct stat *stbuf)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    return -1;
}

int AP_Filesystem_9P2000::unlink(const char *pathname)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    errno = EROFS;
    return -1;
}

int AP_Filesystem_9P2000::mkdir(const char *pathname)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    errno = EROFS;
    return -1;
}

void *AP_Filesystem_9P2000::opendir(const char *pathname)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return nullptr;
    }

    // Make sure filesystem is mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
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
    dir[idx].path = strdup(pathname);
    if (!dir[idx].path) {
        // Strdup failed
        errno = ENOMEM;
        return nullptr;
    }

    // Special case root, can always open if attached
    if (strcmp(pathname, "") == 0) {
        dir[idx].fileId = 0;
        return (void*)&dir[idx];
    }

    // Navigate to the given path
    const uint16_t tag = fs.request_walk(pathname);

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        free(dir[idx].path);
        dir[idx].path = nullptr;
        return nullptr;
    }

    // Grab reply, should be none zero
    const uint32_t fid = fs.dir_walk_result(tag);
    if (fid == 0) {
        // walk failed
        free(dir[idx].path);
        dir[idx].path = nullptr;
        errno = ENOENT;
        return nullptr;
    }

    // Got a valid directory, now open it
    if (!open_fileId(fs, fid, O_RDONLY)) {
        // open failed
        free(dir[idx].path);
        dir[idx].path = nullptr;
        errno = ENOENT;
        return nullptr;
    }

    // Valid directory
    dir[idx].fileId = fid;
    return (void*)&dir[idx];
}

struct dirent *AP_Filesystem_9P2000::readdir(void *dirp)
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

    // Make sure filesystem is still mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
        return nullptr;
    }

    // Read the next item, use large count which will be shortened
    // Because the file stat includes names we can't be sure how long it is.
    const uint16_t tag = fs.request_read(dir[idx].fileId, dir[idx].ofs, UINT32_MAX);

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        return nullptr;
    }

    // Grab reply, should be none zero
    const uint32_t size = fs.dir_read_result(tag, dir[idx].de);
    if (size == 0) {
        // Got to the last item (or read failed)
        return nullptr;
    }

    // Increment offset for next call
    dir[idx].ofs += size;

    return &dir[idx].de;
}

int AP_Filesystem_9P2000::closedir(void *dirp)
{
    uint32_t idx = ((rdir *)dirp) - &dir[0];
    if (idx >= max_open_dir) {
        errno = EBADF;
        return -1;
    }

    if ((dir[idx].path != nullptr) && (dir[idx].fileId != 0)) {
        AP::network().get_filesystem().free_file_id(dir[idx].fileId);
    }

    free(dir[idx].path);
    dir[idx].path = nullptr;
    return 0;
}

int AP_Filesystem_9P2000::rename(const char *oldpath, const char *newpath)
{
    return -1;
}

// set modification time on a file
bool AP_Filesystem_9P2000::set_mtime(const char *filename, const uint32_t mtime_sec)
{
    return false;
}

#endif // AP_FILESYSTEM_P92000_ENABLED

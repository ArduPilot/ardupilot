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

// Get file id for a given path, return 0 if fail
uint32_t AP_Filesystem_9P2000::get_file_id(AP_Networking::NineP2000& fs, const char *name, const AP_Networking::NineP2000::walkType type) const
{
    // Navigate to the given path
    const uint16_t tag = fs.request_walk(name);

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        return 0;
    }

    // Grab reply, should be none zero
    return fs.walk_result(tag, type);
}

bool AP_Filesystem_9P2000::create_file(AP_Networking::NineP2000& fs, const char *fname, bool is_dir)
{
    // Copy string a split into path and name
    const uint16_t len = strlen(fname);

    uint16_t split = 0;
    bool found = false;
    for (uint16_t i = 0; i < len; i++) {
        if (fname[i] == '/') {
            found = true;
            split = i;
        }
    }

    char name[len + 1] {};
    memcpy(&name, fname, len);
    if (found) {
        name[split] = 0;
    }

    // Navigate to parent directory
    const uint32_t fid = get_file_id(fs, found ? name : "", AP_Networking::NineP2000::walkType::Directory);
    if (fid == 0) {
        return false;
    }

    // Create the new item
    const uint8_t name_start = split + (found ? 1 : 0);
    const uint16_t tag = fs.request_create(fid, &name[name_start], is_dir);
    if (tag == fs.NOTAG) {
        return false;
    }

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        fs.free_file_id(fid);
        return false;
    }

    const bool ret = fs.create_result(tag);

    fs.free_file_id(fid);

    return ret;
}

int AP_Filesystem_9P2000::open(const char *fname, int flags, bool allow_absolute_paths)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    // Make sure filesystem is mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
        return -1;
    }

    // Find free file object
    uint8_t idx;
    for (idx=0; idx<max_open_file; idx++) {
        if (file[idx].fileId == 0) {
            break;
        }
    }
    if (idx == max_open_file) {
        errno = ENFILE;
        return -1;
    }

    // Navigate to the given path
    const uint32_t fid = get_file_id(fs, fname, AP_Networking::NineP2000::walkType::File);
    if (fid == 0) {
        // Can't get file id if the file does not exist.
        // If flags are not readonly try and create a new file
        const bool readonly = (flags & O_ACCMODE) == 0;
        if (!readonly && create_file(fs, fname, false)) {
            // Try open again
            return open(fname, flags, allow_absolute_paths);
        }
        errno = ENOENT;
        return -1;
    }

    // Request stats to get length for seek commands
    const uint16_t tag = fs.request_stat(fid);
    if (tag == fs.NOTAG) {
        return -1;
    }

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        fs.free_file_id(fid);
        return -1;
    }

    // Decode stats and open file
    struct stat stats;
    if (!fs.stat_result(tag, &stats) || !open_fileId(fs, fid, flags)) {
        fs.free_file_id(fid);
        errno = ENOENT;
        return -1;
    }

    // Populate file struct
    file[idx].fileId = fid;

    // Set offset to size if appending
    file[idx].ofs = (flags & O_APPEND) ? stats.st_size : 0;
    file[idx].size = stats.st_size;

    return idx;
}

int AP_Filesystem_9P2000::close(int fd)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    // Check valid file
    if ((fd < 0) || (fd >= (int)ARRAY_SIZE(file)) || (file[fd].fileId == 0)) {
        errno = EBADF;
        return -1;
    }

    // Make sure filesystem is mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
        return -1;
    }

    // Free ID and clear file object for reused
    fs.free_file_id(file[fd].fileId);
    file[fd].fileId = 0;
    return 0;
}

int32_t AP_Filesystem_9P2000::read(int fd, void *buf, uint32_t count)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    if ((fd < 0) || (fd >= (int)ARRAY_SIZE(file)) || (file[fd].fileId == 0)) {
        // Invalid file
        errno = EBADF;
        return -1;
    }

    // Make sure filesystem is mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
        return -1;
    }

    // Send read command
    const uint16_t tag = fs.request_read(file[fd].fileId, file[fd].ofs, count);

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        return -1;
    }

    const int read = fs.file_read_result(tag, buf);
    if (read < 0) {
        return -1;
    }

    file[fd].ofs += read;
    return read;
}

int32_t AP_Filesystem_9P2000::write(int fd, const void *buf, uint32_t count)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    if ((fd < 0) || (fd >= (int)ARRAY_SIZE(file)) || (file[fd].fileId == 0)) {
        // Invalid file
        errno = EBADF;
        return -1;
    }

    // Make sure filesystem is mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
        return -1;
    }

    // Send write command
    const uint16_t tag = fs.request_write(file[fd].fileId, file[fd].ofs, count, buf);

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        return -1;
    }

    const int32_t written = fs.write_result(tag);
    if (written < 0) {
        return -1;
    }

    // Return length written
    file[fd].ofs += written;
    file[fd].size = MAX(file[fd].size, file[fd].ofs);
    return written;
}

int32_t AP_Filesystem_9P2000::lseek(int fd, int32_t offset, int seek_from)
{
    if ((fd < 0) || (fd >= (int)ARRAY_SIZE(file)) || (file[fd].fileId == 0)) {
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

    // Make sure filesystem is mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
        return -1;
    }

    // Navigate to the given path
    const uint32_t fid = get_file_id(fs, name, AP_Networking::NineP2000::walkType::File);
    if (fid == 0) {
        errno = ENOENT;
        return -1;
    }

    // Request stat
    const uint16_t tag = fs.request_stat(fid);
    if (tag == fs.NOTAG) {
        return -1;
    }

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        fs.free_file_id(fid);
        return -1;
    }

    // Get result
    const int ret = fs.stat_result(tag, stbuf) ? 0 : -1;

    // Return file handle
    fs.free_file_id(fid);

    return ret;
}

int AP_Filesystem_9P2000::unlink(const char *pathname)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    // Make sure filesystem is mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
        return -1;
    }

    // Navigate to the given path
    const uint32_t fid = get_file_id(fs, pathname, AP_Networking::NineP2000::walkType::Any);
    if (fid == 0) {
        errno = ENOENT;
        return -1;
    }

    // Request stat
    const uint16_t tag = fs.request_remove(fid);
    if (tag == fs.NOTAG) {
        return -1;
    }

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        fs.free_file_id(fid);
        return -1;
    }

    // Get result
    const bool ret = fs.remove_result(tag);

    // Return file handle if the remove failed
    // If remove succeeds then the file id is removed automaticaly.
    if (!ret) {
        fs.free_file_id(fid);
    }

    return ret ? 0 : -1;
}

int AP_Filesystem_9P2000::mkdir(const char *pathname)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    // Make sure filesystem is mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
        return -1;
    }

    // Make folder
    if (!create_file(fs, pathname, true)) {
        errno = EROFS;
        return -1;
    }

    return 0;
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

    // File id should be none zero
    const uint32_t fid = get_file_id(fs, pathname, AP_Networking::NineP2000::walkType::Directory);
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
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return -1;
    }

    // Make sure filesystem is mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
        return -1;
    }

    // Should have a common path.
    const uint16_t oldlen = strlen(oldpath);
    const uint16_t newlen = strlen(newpath);

    uint16_t common = 0;
    uint16_t split = 0;
    bool found_split = false;
    for (uint16_t i = 0; i < MIN(oldlen, newlen); i++) {
        if (oldpath[i] == newpath[i]) {
            common = i;
        }
        if ((oldpath[i] == '/') || (newpath[i] == '/')) {
            found_split = true;
            split = i;
        }
    }

    // Path must be common, only name different
    if (found_split && (common < split)) {
        errno = EPERM;
        return -1;
    }

    // Navigate to the given path
    const uint32_t fid = get_file_id(fs, oldpath, AP_Networking::NineP2000::walkType::Any);
    if (fid == 0) {
        errno = ENOENT;
        return -1;
    }

    // Request rename, pass new name
    const uint8_t name_start = split + (found_split ? 1 : 0);
    const uint16_t tag = fs.request_rename(fid, &newpath[name_start]);
    if (tag == fs.NOTAG) {
        return -1;
    }

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        fs.free_file_id(fid);
        return -1;
    }

    fs.free_file_id(fid);

    return fs.stat_update_result(tag) ? 0 : -1;
}

// set modification time on a file
bool AP_Filesystem_9P2000::set_mtime(const char *filename, const uint32_t mtime_sec)
{
    if (hal.scheduler->in_main_thread()) {
        // Too slow for the main thread
        errno = MAIN_THREAD_ERROR;
        return false;
    }

    // Make sure filesystem is mounted.
    AP_Networking::NineP2000& fs = AP::network().get_filesystem();
    if (!fs.mounted()) {
        errno = ENODEV;
        return false;
    }

    // Navigate to the given path
    const uint32_t fid = get_file_id(fs, filename, AP_Networking::NineP2000::walkType::Any);
    if (fid == 0) {
        errno = ENOENT;
        return false;
    }

    // Request update, pass new time
    const uint16_t tag = fs.request_set_mtime(fid, mtime_sec);
    if (tag == fs.NOTAG) {
        return -1;
    }

    // Wait for the reply
    if (!wait_for_tag(fs, tag)) {
        fs.free_file_id(fid);
        return -1;
    }

    fs.free_file_id(fid);

    return fs.stat_update_result(tag);
}

#endif // AP_FILESYSTEM_P92000_ENABLED

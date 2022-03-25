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
  ArduPilot filesystem interface for ROMFS
 */
#include "AP_Filesystem.h"
#include "AP_Filesystem_ROMFS.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_ROMFS/AP_ROMFS.h>

#if defined(HAL_HAVE_AP_ROMFS_EMBEDDED_H)

int AP_Filesystem_ROMFS::open(const char *fname, int flags, bool allow_absolute_paths)
{
    if ((flags & O_ACCMODE) != O_RDONLY) {
        errno = EROFS;
        return -1;
    }
    uint8_t idx;
    for (idx=0; idx<max_open_file; idx++) {
        if (file[idx].data == nullptr) {
            break;
        }
    }
    if (idx == max_open_file) {
        errno = ENFILE;
        return -1;
    }
    if (file[idx].data != nullptr) {
        errno = EBUSY;
        return -1;
    }
    file[idx].data = AP_ROMFS::find_decompress(fname, file[idx].size);
    if (file[idx].data == nullptr) {
        errno = ENOENT;
        return -1;
    }
    file[idx].ofs = 0;
    return idx;
}

int AP_Filesystem_ROMFS::close(int fd)
{
    if (fd < 0 || fd >= max_open_file || file[fd].data == nullptr) {
        errno = EBADF;
        return -1;
    }
    AP_ROMFS::free(file[fd].data);
    file[fd].data = nullptr;
    return 0;
}

int32_t AP_Filesystem_ROMFS::read(int fd, void *buf, uint32_t count)
{
    if (fd < 0 || fd >= max_open_file || file[fd].data == nullptr) {
        errno = EBADF;
        return -1;
    }
    count = MIN(file[fd].size - file[fd].ofs, count);
    if (count == 0) {
        return 0;
    }
    memcpy(buf, &file[fd].data[file[fd].ofs], count);
    file[fd].ofs += count;
    return count;
}

int32_t AP_Filesystem_ROMFS::write(int fd, const void *buf, uint32_t count)
{
    errno = EROFS;
    return -1;
}

int AP_Filesystem_ROMFS::fsync(int fd)
{
    return 0;
}

int32_t AP_Filesystem_ROMFS::lseek(int fd, int32_t offset, int seek_from)
{
    if (fd < 0 || fd >= max_open_file || file[fd].data == nullptr) {
        errno = EBADF;
        return -1;
    }
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

int AP_Filesystem_ROMFS::stat(const char *name, struct stat *stbuf)
{
    uint32_t size;
    const uint8_t *data = AP_ROMFS::find_decompress(name, size);
    if (data == nullptr) {
        errno = ENOENT;
        return -1;
    }
    AP_ROMFS::free(data);
    memset(stbuf, 0, sizeof(*stbuf));
    stbuf->st_size = size;
    return 0;
}

int AP_Filesystem_ROMFS::unlink(const char *pathname)
{
    errno = EROFS;
    return -1;
}

int AP_Filesystem_ROMFS::mkdir(const char *pathname)
{
    errno = EROFS;
    return -1;
}

void *AP_Filesystem_ROMFS::opendir(const char *pathname)
{
    uint8_t idx;
    for (idx=0; idx<max_open_dir; idx++) {
        if (dir[idx].path == nullptr) {
            break;
        }
    }
    if (idx == max_open_dir) {
        errno = ENFILE;
        return nullptr;
    }
    dir[idx].ofs = 0;
    dir[idx].path = strdup(pathname);
    if (!dir[idx].path) {
        return nullptr;
    }
    return (void*)&dir[idx];
}

struct dirent *AP_Filesystem_ROMFS::readdir(void *dirp)
{
    uint32_t idx = ((rdir*)dirp) - &dir[0];
    if (idx >= max_open_dir) {
        errno = EBADF;
        return nullptr;
    }
    const char *name = AP_ROMFS::dir_list(dir[idx].path, dir[idx].ofs);
    if (!name) {
        return nullptr;
    }
    const uint32_t plen = strlen(dir[idx].path);
    if (strncmp(name, dir[idx].path, plen) != 0 || name[plen] != '/') {
        return nullptr;
    }
    name += plen + 1;
    dir[idx].de.d_type = DT_REG;
    strncpy(dir[idx].de.d_name, name, sizeof(dir[idx].de.d_name));
    return &dir[idx].de;
}

int AP_Filesystem_ROMFS::closedir(void *dirp)
{
    uint32_t idx = ((rdir *)dirp) - &dir[0];
    if (idx >= max_open_dir) {
        errno = EBADF;
        return -1;
    }
    free(dir[idx].path);
    dir[idx].path = nullptr;
    return 0;
}

// return free disk space in bytes
int64_t AP_Filesystem_ROMFS::disk_free(const char *path)
{
    return 0;
}

// return total disk space in bytes
int64_t AP_Filesystem_ROMFS::disk_space(const char *path)
{
    return 0;
}

/*
  set mtime on a file
 */
bool AP_Filesystem_ROMFS::set_mtime(const char *filename, const uint32_t mtime_sec)
{
    return false;
}

/*
  load a full file. Use delete to free the data
  we override this in ROMFS to avoid taking twice the memory
*/
FileData *AP_Filesystem_ROMFS::load_file(const char *filename)
{
    FileData *fd = new FileData(this);
    if (!fd) {
        return nullptr;
    }
    fd->data = AP_ROMFS::find_decompress(filename, fd->length);
    if (fd->data == nullptr) {
        delete fd;
        return nullptr;
    }
    return fd;
}

// unload data from load_file()
void AP_Filesystem_ROMFS::unload_file(FileData *fd)
{
    AP_ROMFS::free(fd->data);
}

#endif // HAL_HAVE_AP_ROMFS_EMBEDDED_H

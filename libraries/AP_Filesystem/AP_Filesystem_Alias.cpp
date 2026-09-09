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

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_ALIAS_ENABLED

#include "AP_Filesystem_Alias.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>

/*
  put the alias' root on the front of a path. returns false if the result
  would not fit, in which case nothing could have been opened anyway
 */
static bool full_path(char *buf, size_t buflen, const char *dir, const char *path)
{
    // an empty path is the aliased directory itself
    if (path == nullptr || path[0] == 0) {
        return snprintf(buf, buflen, "%s", dir) < (int)buflen;
    }
    // the root may already end in a separator; another would name a
    // different place
    const size_t dir_len = strlen(dir);
    const char *sep = (dir_len > 0 && dir[dir_len - 1] != '/') ? "/" : "";
    return snprintf(buf, buflen, "%s%s%s", dir, sep, path) < (int)buflen;
}

#define WITH_FULL_PATH(path, failure_return)                        \
    char pathbuf[max_path_len];                                     \
    if (!full_path(pathbuf, sizeof(pathbuf), root(), path)) {       \
        errno = ENAMETOOLONG;                                       \
        return failure_return;                                      \
    }

int AP_Filesystem_Alias::open(const char *fname, int flags, bool allow_absolute_paths)
{
    WITH_FULL_PATH(fname, -1);
    return target.open(pathbuf, flags, allow_absolute_paths);
}

int AP_Filesystem_Alias::close(int fd)
{
    return target.close(fd);
}

int32_t AP_Filesystem_Alias::read(int fd, void *buf, uint32_t count)
{
    return target.read(fd, buf, count);
}

int32_t AP_Filesystem_Alias::write(int fd, const void *buf, uint32_t count)
{
    return target.write(fd, buf, count);
}

int AP_Filesystem_Alias::fsync(int fd)
{
    return target.fsync(fd);
}

int32_t AP_Filesystem_Alias::lseek(int fd, int32_t offset, int whence)
{
    return target.lseek(fd, offset, whence);
}

int AP_Filesystem_Alias::stat(const char *pathname, struct stat *stbuf)
{
    WITH_FULL_PATH(pathname, -1);
    return target.stat(pathbuf, stbuf);
}

int AP_Filesystem_Alias::unlink(const char *pathname)
{
    WITH_FULL_PATH(pathname, -1);
    return target.unlink(pathbuf);
}

int AP_Filesystem_Alias::mkdir(const char *pathname)
{
    WITH_FULL_PATH(pathname, -1);
    return target.mkdir(pathbuf);
}

void *AP_Filesystem_Alias::opendir(const char *pathname)
{
    WITH_FULL_PATH(pathname, nullptr);
    return target.opendir(pathbuf);
}

struct dirent *AP_Filesystem_Alias::readdir(void *dirp)
{
    return target.readdir(dirp);
}

int AP_Filesystem_Alias::closedir(void *dirp)
{
    return target.closedir(dirp);
}

int AP_Filesystem_Alias::rename(const char *oldpath, const char *newpath)
{
    char oldbuf[max_path_len];
    char newbuf[max_path_len];
    const char *dir = root();
    if (!full_path(oldbuf, sizeof(oldbuf), dir, oldpath) ||
        !full_path(newbuf, sizeof(newbuf), dir, newpath)) {
        errno = ENAMETOOLONG;
        return -1;
    }
    return target.rename(oldbuf, newbuf);
}

int64_t AP_Filesystem_Alias::disk_free(const char *path)
{
    WITH_FULL_PATH(path, -1);
    return target.disk_free(pathbuf);
}

int64_t AP_Filesystem_Alias::disk_space(const char *path)
{
    WITH_FULL_PATH(path, -1);
    return target.disk_space(pathbuf);
}

bool AP_Filesystem_Alias::set_mtime(const char *filename, const uint32_t mtime_sec)
{
    WITH_FULL_PATH(filename, false);
    return target.set_mtime(pathbuf, mtime_sec);
}

#endif  // AP_FILESYSTEM_ALIAS_ENABLED

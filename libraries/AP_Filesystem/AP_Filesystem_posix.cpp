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
  ArduPilot filesystem interface for posix systems
 */
#include "AP_Filesystem.h"
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#if defined(__APPLE__)
#include <sys/mount.h>
#else
#include <sys/vfs.h>
#endif
#include <utime.h>

extern const AP_HAL::HAL& hal;

/*
  map a filename for SITL so operations are relative to the current directory
 */
static const char *map_filename(const char *fname)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // on SITL only allow paths under subdirectory. Users can still
    // escape with .. if they want to
    if (strcmp(fname, "/") == 0) {
        return ".";
    }
    if (*fname == '/') {
        fname++;
    }
#endif
    // on Linux allow original name
    return fname;
}

int AP_Filesystem_Posix::open(const char *fname, int flags)
{
    fname = map_filename(fname);
    // we automatically add O_CLOEXEC as we always want it for ArduPilot FS usage
    return ::open(fname, flags | O_CLOEXEC, 0644);
}

int AP_Filesystem_Posix::close(int fd)
{
    return ::close(fd);
}

int32_t AP_Filesystem_Posix::read(int fd, void *buf, uint32_t count)
{
    return ::read(fd, buf, count);
}

int32_t AP_Filesystem_Posix::write(int fd, const void *buf, uint32_t count)
{
    return ::write(fd, buf, count);
}

int AP_Filesystem_Posix::fsync(int fd)
{
    return ::fsync(fd);
}

int32_t AP_Filesystem_Posix::lseek(int fd, int32_t offset, int seek_from)
{
    return ::lseek(fd, offset, seek_from);
}

int AP_Filesystem_Posix::stat(const char *pathname, struct stat *stbuf)
{
    pathname = map_filename(pathname);
    return ::stat(pathname, stbuf);
}

int AP_Filesystem_Posix::unlink(const char *pathname)
{
    pathname = map_filename(pathname);
    // we match the FATFS interface and use unlink
    // for both files and directories
    int ret = ::rmdir(pathname);
    if (ret == -1) {
        ret = ::unlink(pathname);
    }
    return ret;
}

int AP_Filesystem_Posix::mkdir(const char *pathname)
{
    pathname = map_filename(pathname);
    return ::mkdir(pathname, 0775);
}

void *AP_Filesystem_Posix::opendir(const char *pathname)
{
    pathname = map_filename(pathname);
    return (void*)::opendir(pathname);
}

struct dirent *AP_Filesystem_Posix::readdir(void *dirp)
{
    return ::readdir((DIR *)dirp);
}

int AP_Filesystem_Posix::closedir(void *dirp)
{
    return ::closedir((DIR *)dirp);
}

// return free disk space in bytes
int64_t AP_Filesystem_Posix::disk_free(const char *path)
{
    path = map_filename(path);
    struct statfs stats;
    if (::statfs(path, &stats) < 0) {
        return -1;
    }
    return (((int64_t)stats.f_bavail) * stats.f_bsize);
}

// return total disk space in bytes
int64_t AP_Filesystem_Posix::disk_space(const char *path)
{
    path = map_filename(path);
    struct statfs stats;
    if (::statfs(path, &stats) < 0) {
        return -1;
    }
    return (((int64_t)stats.f_blocks) * stats.f_bsize);
}


/*
  set mtime on a file
 */
bool AP_Filesystem_Posix::set_mtime(const char *filename, const uint32_t mtime_sec)
{
    filename = map_filename(filename);
    struct utimbuf times {};
    times.actime = mtime_sec;
    times.modtime = mtime_sec;

    return utime(filename, &times) == 0;
}

#endif // CONFIG_HAL_BOARD


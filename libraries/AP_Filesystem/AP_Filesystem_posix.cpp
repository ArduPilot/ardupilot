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

int AP_Filesystem::open(const char *fname, int flags)
{
    // we automatically add O_CLOEXEC as we always want it for ArduPilot FS usage
    return ::open(fname, flags | O_CLOEXEC, 0644);
}

int AP_Filesystem::close(int fd)
{
    return ::close(fd);
}

ssize_t AP_Filesystem::read(int fd, void *buf, size_t count)
{
    return ::read(fd, buf, count);
}

ssize_t AP_Filesystem::write(int fd, const void *buf, size_t count)
{
    return ::write(fd, buf, count);
}

int AP_Filesystem::fsync(int fd)
{
    return ::fsync(fd);
}

off_t AP_Filesystem::lseek(int fd, off_t offset, int seek_from)
{
    return ::lseek(fd, offset, seek_from);
}

int AP_Filesystem::stat(const char *pathname, struct stat *stbuf)
{
    return ::stat(pathname, stbuf);
}

int AP_Filesystem::unlink(const char *pathname)
{
    // we match the FATFS interface and use unlink
    // for both files and directories
    int ret = ::rmdir(pathname);
    if (ret == -1) {
        ret = ::unlink(pathname);
    }
    return ret;
}

int AP_Filesystem::mkdir(const char *pathname)
{
    return ::mkdir(pathname, 0775);
}

DIR *AP_Filesystem::opendir(const char *pathname)
{
    return ::opendir(pathname);
}

struct dirent *AP_Filesystem::readdir(DIR *dirp)
{
    return ::readdir(dirp);
}

int AP_Filesystem::closedir(DIR *dirp)
{
    return ::closedir(dirp);
}

// return free disk space in bytes
int64_t AP_Filesystem::disk_free(const char *path)
{
    struct statfs stats;
    if (::statfs(path, &stats) < 0) {
        return -1;
    }
    return (((int64_t)stats.f_bavail) * stats.f_bsize);
}

// return total disk space in bytes
int64_t AP_Filesystem::disk_space(const char *path)
{
    struct statfs stats;
    if (::statfs(path, &stats) < 0) {
        return -1;
    }
    return (((int64_t)stats.f_blocks) * stats.f_bsize);
}


/*
  set mtime on a file
 */
bool AP_Filesystem::set_mtime(const char *filename, const time_t mtime_sec)
{
    struct utimbuf times {};
    times.actime = mtime_sec;
    times.modtime = mtime_sec;

    return utime(filename, &times) == 0;
}

#endif // CONFIG_HAL_BOARD


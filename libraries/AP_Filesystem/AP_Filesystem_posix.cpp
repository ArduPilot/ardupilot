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

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_POSIX_ENABLED

#include "AP_Filesystem.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#if defined(__APPLE__) || defined(__OpenBSD__)
#include <sys/mount.h>
#elif CONFIG_HAL_BOARD != HAL_BOARD_QURT
#include <sys/vfs.h>
#endif

#if AP_FILESYSTEM_POSIX_HAVE_UTIME
#include <utime.h>
#endif

extern const AP_HAL::HAL& hal;

/*
  map a filename so operations are relative to the current directory if needed
 */
static const char *map_filename(const char *fname)
{
#if AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC
    // this system needs to add a prefix to the filename, which means
    // memory allocation. This is needed for QURT which lacks chdir()
    char *fname2 = nullptr;
    asprintf(&fname2, "%s/%s", AP_FILESYSTEM_POSIX_MAP_FILENAME_BASEDIR, fname);
    return fname2;
#else

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !APM_BUILD_TYPE(APM_BUILD_Replay)
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
#endif // AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC
}

static void map_filename_free(const char *fname)
{
#if AP_FILESYSTEM_POSIX_MAP_FILENAME_ALLOC
    ::free(const_cast<char*>(fname));
#endif
}

int AP_Filesystem_Posix::open(const char *fname, int flags, bool allow_absolute_paths)
{
    FS_CHECK_ALLOWED(-1);
    if (! allow_absolute_paths) {
        fname = map_filename(fname);
    }
    struct stat st;
    if (::stat(fname, &st) == 0 &&
        ((st.st_mode & S_IFMT) != S_IFREG && (st.st_mode & S_IFMT) != S_IFLNK)) {
        // only allow links and files
        if (!allow_absolute_paths) {
            map_filename_free(fname);
        }
        return -1;
    }

    // we automatically add O_CLOEXEC as we always want it for ArduPilot FS usage
    auto ret = ::open(fname, flags | O_CLOEXEC, 0644);
    if (!allow_absolute_paths) {
        map_filename_free(fname);
    }
    return ret;
}

int AP_Filesystem_Posix::close(int fd)
{
    FS_CHECK_ALLOWED(-1);
    return ::close(fd);
}

int32_t AP_Filesystem_Posix::read(int fd, void *buf, uint32_t count)
{
    FS_CHECK_ALLOWED(-1);
    return ::read(fd, buf, count);
}

int32_t AP_Filesystem_Posix::write(int fd, const void *buf, uint32_t count)
{
    FS_CHECK_ALLOWED(-1);
    return ::write(fd, buf, count);
}

int AP_Filesystem_Posix::fsync(int fd)
{
#if AP_FILESYSTEM_POSIX_HAVE_FSYNC
    FS_CHECK_ALLOWED(-1);
    return ::fsync(fd);
#else
    // we have to pass success here as otherwise it is assumed to be
    // failed IO and the caller may close the fd and give up
    return 0;
#endif
}

int32_t AP_Filesystem_Posix::lseek(int fd, int32_t offset, int seek_from)
{
    FS_CHECK_ALLOWED(-1);
    return ::lseek(fd, offset, seek_from);
}

int AP_Filesystem_Posix::stat(const char *pathname, struct stat *stbuf)
{
    FS_CHECK_ALLOWED(-1);
    pathname = map_filename(pathname);
    auto ret = ::stat(pathname, stbuf);
    map_filename_free(pathname);
    return ret;
}

int AP_Filesystem_Posix::unlink(const char *pathname)
{
    FS_CHECK_ALLOWED(-1);
    pathname = map_filename(pathname);
    // we match the FATFS interface and use unlink
    // for both files and directories
    int ret = ::rmdir(const_cast<char*>(pathname));
    if (ret == -1) {
        ret = ::unlink(pathname);
    }
    map_filename_free(pathname);
    return ret;
}

int AP_Filesystem_Posix::mkdir(const char *pathname)
{
    FS_CHECK_ALLOWED(-1);
    pathname = map_filename(pathname);
    auto ret = ::mkdir(const_cast<char*>(pathname), 0775);
    map_filename_free(pathname);
    return ret;
}

void *AP_Filesystem_Posix::opendir(const char *pathname)
{
    FS_CHECK_ALLOWED(nullptr);
    pathname = map_filename(pathname);
    auto *ret = (void*)::opendir(pathname);
    map_filename_free(pathname);
    return ret;
}

struct dirent *AP_Filesystem_Posix::readdir(void *dirp)
{
    FS_CHECK_ALLOWED(nullptr);
    return ::readdir((DIR *)dirp);
}

int AP_Filesystem_Posix::closedir(void *dirp)
{
    FS_CHECK_ALLOWED(-1);
    return ::closedir((DIR *)dirp);
}

int AP_Filesystem_Posix::rename(const char *oldpath, const char *newpath)
{
    FS_CHECK_ALLOWED(-1);
    oldpath = map_filename(oldpath);
    newpath = map_filename(newpath);
    auto ret = ::rename(oldpath, newpath);
    map_filename_free(oldpath);
    map_filename_free(newpath);
    return ret;
}

// return free disk space in bytes
int64_t AP_Filesystem_Posix::disk_free(const char *path)
{
#if AP_FILESYSTEM_POSIX_HAVE_STATFS
    FS_CHECK_ALLOWED(-1);
    path = map_filename(path);
    struct statfs stats;
    if (::statfs(path, &stats) < 0) {
        map_filename_free(path);
        return -1;
    }
    map_filename_free(path);
    return (((int64_t)stats.f_bavail) * stats.f_bsize);
#else
    return -1;
#endif
}

// return total disk space in bytes
int64_t AP_Filesystem_Posix::disk_space(const char *path)
{
#if AP_FILESYSTEM_POSIX_HAVE_STATFS
    FS_CHECK_ALLOWED(-1);
    path = map_filename(path);
    struct statfs stats;
    if (::statfs(path, &stats) < 0) {
        map_filename_free(path);
        return -1;
    }
    map_filename_free(path);
    return (((int64_t)stats.f_blocks) * stats.f_bsize);
#else
    return -1;
#endif
}


/*
  set mtime on a file
 */
bool AP_Filesystem_Posix::set_mtime(const char *filename, const uint32_t mtime_sec)
{
#if AP_FILESYSTEM_POSIX_HAVE_UTIME
    FS_CHECK_ALLOWED(false);
    filename = map_filename(filename);
    struct utimbuf times {};
    times.actime = mtime_sec;
    times.modtime = mtime_sec;

    auto ret = utime(filename, &times) == 0;
    map_filename_free(filename);
    return ret;
#else
    return false;
#endif
}

#endif  // AP_FILESYSTEM_POSIX_ENABLED

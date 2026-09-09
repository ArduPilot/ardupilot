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

#include "AP_Filesystem.h"

#include "AP_Filesystem_config.h"

#if AP_FILESYSTEM_FILE_READING_ENABLED

#include <AP_HAL/HAL.h>
#include <AP_HAL/Util.h>
#include <AP_Math/AP_Math.h>

static AP_Filesystem fs;

// create exactly one "local" filesystem:
#if AP_FILESYSTEM_FATFS_ENABLED
#include "AP_Filesystem_FATFS.h"
static AP_Filesystem_FATFS fs_local;
#elif AP_FILESYSTEM_ESP32_ENABLED
#include "AP_Filesystem_ESP32.h"
static AP_Filesystem_ESP32 fs_local;
#elif AP_FILESYSTEM_LITTLEFS_ENABLED
#include "AP_Filesystem_FlashMemory_LittleFS.h"
static AP_Filesystem_FlashMemory_LittleFS fs_local;
#elif AP_FILESYSTEM_POSIX_ENABLED
#include "AP_Filesystem_posix.h"
static AP_Filesystem_Posix fs_local;
#else
static AP_Filesystem_Backend fs_local;
int errno;
#endif

#if AP_FILESYSTEM_ROMFS_ENABLED
#include "AP_Filesystem_ROMFS.h"
static AP_Filesystem_ROMFS fs_romfs;
#endif

#if AP_FILESYSTEM_PARAM_ENABLED
#include "AP_Filesystem_Param.h"
static AP_Filesystem_Param fs_param;
#endif

#if AP_FILESYSTEM_SYS_ENABLED
#include "AP_Filesystem_Sys.h"
static AP_Filesystem_Sys fs_sys;
#endif

#if AP_FILESYSTEM_MISSION_ENABLED
#include "AP_Filesystem_Mission.h"
static AP_Filesystem_Mission fs_mission;
#endif

#if AP_FILESYSTEM_MAVLOG_ENABLED
// @MAV_LOG needs logs on a filesystem.  tested here, not in our config
// header, because AP_Logger_config.h includes that header
#include <AP_Logger/AP_Logger_config.h>
#endif

#if AP_FILESYSTEM_MAVLOG_ENABLED && HAL_LOGGING_FILESYSTEM_ENABLED
#define AP_FILESYSTEM_MAVLOG_ROW_ENABLED 1
#else
#define AP_FILESYSTEM_MAVLOG_ROW_ENABLED 0
#endif

#if AP_FILESYSTEM_MAVLOG_ROW_ENABLED
// defined below, once the HAL is in scope
static const char *mavlog_root(void);

// log directory + '/' + longest FTP request + '/' + longest name, or listings silently lose entries
static_assert(sizeof(HAL_BOARD_LOG_DIRECTORY) + 1 + 238 + 1 + 255 <= AP_FILESYSTEM_ALIAS_PATH_MAX,
              "AP_FILESYSTEM_ALIAS_PATH_MAX is too small for this board's log directory");
#endif

/*
  mapping from filesystem prefix to backend
 */
const AP_Filesystem::Backend AP_Filesystem::backends[] = {
    { nullptr, fs_local, nullptr },
#if AP_FILESYSTEM_ROMFS_ENABLED
    { "@ROMFS", fs_romfs, nullptr },
#endif
#if AP_FILESYSTEM_PARAM_ENABLED
    { "@PARAM", fs_param, nullptr },
#endif
#if AP_FILESYSTEM_SYS_ENABLED
    { "@SYS", fs_sys, nullptr },
#endif
#if AP_FILESYSTEM_MISSION_ENABLED
    { "@MISSION", fs_mission, nullptr },
#endif
#if AP_FILESYSTEM_MAVLOG_ROW_ENABLED
    // an alias for the log directory on the local filesystem
    { "@MAV_LOG", fs_local, mavlog_root },
#endif
};

extern const AP_HAL::HAL& hal;

#if AP_FILESYSTEM_MAVLOG_ROW_ENABLED
/*
  the log directory, chosen as AP_Logger_File does.  the static_assert can't
  see a custom directory, so a long one can push long listing paths past
  AP_FILESYSTEM_ALIAS_PATH_MAX
 */
static const char *mavlog_root(void)
{
    const char *custom_dir = hal.util->get_custom_log_directory();
    if (custom_dir != nullptr) {
        return custom_dir;
    }
    return HAL_BOARD_LOG_DIRECTORY;
}
#endif

#define MAX_FD_PER_BACKEND 256U
#define NUM_BACKENDS ARRAY_SIZE(backends)
#define LOCAL_BACKEND backends[0]
#define BACKEND_IDX(backend) (&(backend) - &backends[0])

/*
  find backend by path
 */
const AP_Filesystem::Backend &AP_Filesystem::backend_by_path(const char *&path) const
{
    // ignore leading slashes:
    const char *path_with_no_leading_slash = path;
    if (path_with_no_leading_slash[0] == '/') {
        path_with_no_leading_slash = &path_with_no_leading_slash[1];
    }
    for (uint8_t i=1; i<NUM_BACKENDS; i++) {
        const uint8_t plen = strlen(backends[i].prefix);
        if (strncmp(path_with_no_leading_slash, backends[i].prefix, plen) != 0) {
            continue;
        }
        // the prefix must be the whole first component ("@SYSfoo" is not @SYS)
        const char after = path_with_no_leading_slash[plen];
        if (after == 0 || after == '/') {
            path = path_with_no_leading_slash;
            path += plen;
            if (strlen(path) > 0 && path[0] == '/') {
                path++;
            }
            return backends[i];
        }
    }
    // default to local filesystem
    return LOCAL_BACKEND;
}

// resolve a path to its backend, rewriting it if the prefix is an alias
AP_Filesystem::ResolvedPath::ResolvedPath(AP_Filesystem &filesystem, const char *path, Buffer buffer) :
    _backend(&filesystem.backend_by_path(path)),
    _path(path)
#if AP_FILESYSTEM_ALIAS_ENABLED
    ,_filesystem(filesystem),
    _own_buffer(nullptr),
    _holds_shared_buffer(false)
#endif
{
#if AP_FILESYSTEM_ALIAS_ENABLED
    if (_backend->root == nullptr) {
        // not an alias; the backend takes the path as it stands
        return;
    }

    char *rewritten;
    if (buffer == Buffer::OWN) {
        _own_buffer = NEW_NOTHROW char[AP_FILESYSTEM_ALIAS_PATH_MAX];
        rewritten = _own_buffer;
    } else {
        // held until the destructor; see the warning on alias_sem
        filesystem.alias_sem.take_blocking();
        if (filesystem.alias_path_in_use) {
            // this thread already has the buffer, still in use
            filesystem.alias_sem.give();
            _path = nullptr;
            errno = EBUSY;
            return;
        }
        _holds_shared_buffer = true;
        filesystem.alias_path_in_use = true;
        if (filesystem.alias_path == nullptr) {
            filesystem.alias_path = NEW_NOTHROW char[AP_FILESYSTEM_ALIAS_PATH_MAX];
        }
        rewritten = filesystem.alias_path;
    }
    if (rewritten == nullptr) {
        _path = nullptr;
        errno = ENOMEM;
        return;
    }

    const char *dir = _backend->root();
    const size_t dir_len = strlen(dir);
    // don't double a separator the root already ends in
    const char *sep = (dir_len > 0 && dir[dir_len - 1] != '/' && _path[0] != 0) ? "/" : "";
    if (uint32_t(hal.util->snprintf(rewritten, AP_FILESYSTEM_ALIAS_PATH_MAX, "%s%s%s", dir, sep, _path)) >= AP_FILESYSTEM_ALIAS_PATH_MAX) {
        // refuse rather than truncate: a shortened path may name another file
        _path = nullptr;
        errno = ENAMETOOLONG;
        return;
    }
    _path = rewritten;
#endif
}

AP_Filesystem::ResolvedPath::~ResolvedPath()
{
#if AP_FILESYSTEM_ALIAS_ENABLED
    delete[] _own_buffer;
    if (_holds_shared_buffer) {
        _filesystem.alias_path_in_use = false;
        _filesystem.alias_sem.give();
    }
#endif
}

/*
  return backend by file descriptor
 */
const AP_Filesystem::Backend &AP_Filesystem::backend_by_fd(int &fd) const
{
    if (fd < 0 || uint32_t(fd) >= NUM_BACKENDS*MAX_FD_PER_BACKEND) {
        return LOCAL_BACKEND;
    }
    const uint8_t idx = uint32_t(fd) / MAX_FD_PER_BACKEND;
    fd -= idx * MAX_FD_PER_BACKEND;
    return backends[idx];
}

int AP_Filesystem::open(const char *fname, int flags, bool allow_absolute_paths)
{
    const ResolvedPath resolved { *this, fname };
    if (!resolved.valid()) {
        return -1;
    }
    const Backend &backend = resolved.backend();
    int fd = backend.fs.open(resolved.path(), flags, allow_absolute_paths);
    if (fd < 0) {
        return -1;
    }
    if (uint32_t(fd) >= MAX_FD_PER_BACKEND) {
        backend.fs.close(fd);
        errno = ERANGE;
        return -1;
    }
    // offset fd so we can recognise the backend
    const uint8_t idx = (&backend - &backends[0]);
    fd += idx * MAX_FD_PER_BACKEND;
    return fd;
}

int AP_Filesystem::close(int fd)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.close(fd);
}

int32_t AP_Filesystem::read(int fd, void *buf, uint32_t count)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.read(fd, buf, count);
}

int32_t AP_Filesystem::write(int fd, const void *buf, uint32_t count)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.write(fd, buf, count);
}

int AP_Filesystem::fsync(int fd)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.fsync(fd);
}

int32_t AP_Filesystem::lseek(int fd, int32_t offset, int seek_from)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.lseek(fd, offset, seek_from);
}

int AP_Filesystem::stat(const char *pathname, struct stat *stbuf)
{
    const ResolvedPath resolved { *this, pathname };
    if (!resolved.valid()) {
        return -1;
    }
    return resolved.backend().fs.stat(resolved.path(), stbuf);
}

int AP_Filesystem::unlink(const char *pathname)
{
    const ResolvedPath resolved { *this, pathname };
    if (!resolved.valid()) {
        return -1;
    }
    return resolved.backend().fs.unlink(resolved.path());
}

int AP_Filesystem::mkdir(const char *pathname)
{
    const ResolvedPath resolved { *this, pathname };
    if (!resolved.valid()) {
        return -1;
    }
    return resolved.backend().fs.mkdir(resolved.path());
}

int AP_Filesystem::rename(const char *oldpath, const char *newpath)
{
    const ResolvedPath oldresolved { *this, oldpath };

    // Don't need the backend again, but we also need to remove the backend pre-fix from the new path.
    // a second live path can't share the alias buffer
    const ResolvedPath newresolved { *this, newpath, ResolvedPath::Buffer::OWN };

    if (!oldresolved.valid() || !newresolved.valid()) {
        return -1;
    }

    // Don't try and rename between backends.
    if (&oldresolved.backend() != &newresolved.backend()) {
        return -1;
    }

    return oldresolved.backend().fs.rename(oldresolved.path(), newresolved.path());
}

AP_Filesystem::DirHandle *AP_Filesystem::opendir(const char *pathname)
{
    // support reading a list of "@" filesystems (e.g. @SYS) in
    // listing of root directory.  Note that backend_by_path modifies
    // its parameter.
    if (strlen(pathname) == 0 ||
        (strlen(pathname) == 1 && pathname[0] == '/')) {
        virtual_dirent.backend_ofs = 0;
        virtual_dirent.d_off = 0;
#if AP_FILESYSTEM_HAVE_DIRENT_DTYPE
        virtual_dirent.de.d_type = DT_DIR;
#endif
    } else {
        virtual_dirent.backend_ofs = 255;
    }

    const ResolvedPath resolved { *this, pathname };
    if (!resolved.valid()) {
        return nullptr;
    }
    const Backend &backend = resolved.backend();
    DirHandle *h = NEW_NOTHROW DirHandle;
    if (!h) {
        return nullptr;
    }
    h->dir = backend.fs.opendir(resolved.path());
    if (h->dir == nullptr) {
        delete h;
        return nullptr;
    }
    h->fs_index = BACKEND_IDX(backend);

    return h;
}

struct dirent *AP_Filesystem::readdir(DirHandle *dirp)
{
    if (!dirp) {
        return nullptr;
    }
    const Backend &backend = backends[dirp->fs_index];
    struct dirent * ret = backend.fs.readdir(dirp->dir);
    if (ret != nullptr) {
        return ret;
    }

    // virtual directory entries in the root directory (e.g. @SYS, @MISSION)
    for (; ret == nullptr && virtual_dirent.backend_ofs < ARRAY_SIZE(AP_Filesystem::backends); virtual_dirent.backend_ofs++) {
        const char *prefix = backends[virtual_dirent.backend_ofs].prefix;
        if (prefix == nullptr) {
            continue;
        }
        if (prefix[0] != '@') {
            continue;
        }

        // only return @ entries in root if we can successfully opendir them
        // (an alias at its own root)
        const Backend &probed = backends[virtual_dirent.backend_ofs];
        auto *d = probed.fs.opendir(probed.root != nullptr ? probed.root() : "");
        if (d == nullptr) {
            continue;
        }
        backends[virtual_dirent.backend_ofs].fs.closedir(d);

        // found a virtual directory we haven't returned yet
        strncpy_noterm(virtual_dirent.de.d_name, prefix, sizeof(virtual_dirent.de.d_name));
        virtual_dirent.d_off++;
        ret = &virtual_dirent.de;
    }
    return ret;
}

int AP_Filesystem::closedir(DirHandle *dirp)
{
    if (!dirp) {
        return -1;
    }
    const Backend &backend = backends[dirp->fs_index];
    int ret = backend.fs.closedir(dirp->dir);
    delete dirp;
    return ret;
}

// return number of bytes that should be written before fsync for optimal
// streaming performance/robustness. if zero, any number can be written.
uint32_t AP_Filesystem::bytes_until_fsync(int fd)
{
    const Backend &backend = backend_by_fd(fd);
    return backend.fs.bytes_until_fsync(fd);
}

// return free disk space in bytes
int64_t AP_Filesystem::disk_free(const char *path)
{
    const ResolvedPath resolved { *this, path };
    if (!resolved.valid()) {
        return -1;
    }
    return resolved.backend().fs.disk_free(resolved.path());
}

// return total disk space in bytes
int64_t AP_Filesystem::disk_space(const char *path)
{
    const ResolvedPath resolved { *this, path };
    if (!resolved.valid()) {
        return -1;
    }
    return resolved.backend().fs.disk_space(resolved.path());
}


/*
  set mtime on a file
 */
bool AP_Filesystem::set_mtime(const char *filename, const uint32_t mtime_sec)
{
    const ResolvedPath resolved { *this, filename };
    if (!resolved.valid()) {
        return false;
    }
    return resolved.backend().fs.set_mtime(resolved.path(), mtime_sec);
}

// if filesystem is not running then try a remount
bool AP_Filesystem::retry_mount(void)
{
    return LOCAL_BACKEND.fs.retry_mount();
}

// unmount filesystem for reboot
void AP_Filesystem::unmount(void)
{
    return LOCAL_BACKEND.fs.unmount();
}

/*
  Load a file's contents into memory. Returned object must be `delete`d to free
  the data. The data is guaranteed to be null-terminated such that it can be
  treated as a string.
 */
FileData *AP_Filesystem::load_file(const char *filename)
{
    const ResolvedPath resolved { *this, filename };
    if (!resolved.valid()) {
        return nullptr;
    }
    return resolved.backend().fs.load_file(resolved.path());
}

// reads a line into buf, guaranteeing null-termination.  buflen is
// the full size of buf, including the space required for the null
// terminator, so up to buflen-1 characters are returned.  cr or lf
// terminates the line and is consumed but not returned.
bool AP_Filesystem::fgets(char *buf, uint8_t buflen, int fd)
{
    if (buflen == 0) {
        // we can't null-terminate the buffer, which we guarantee
        return false;
    }

    const Backend &backend = backend_by_fd(fd);

    // we will need to seek back to the right location at the end
    auto offset_start = backend.fs.lseek(fd, 0, SEEK_CUR);
    if (offset_start < 0) {
        return false;
    }

    auto n = backend.fs.read(fd, buf, buflen-1U);
    if (n <= 0) {
        return false;
    }

    uint8_t i = 0;
    for (; i < n; i++) {
        if (buf[i] == '\r' || buf[i] == '\n') {
            break;
        }
    }
    buf[i] = '\0';

    // get back to the right offset, consuming the line terminator if
    // we found one.  If we did not find one we have either filled the
    // buffer or returned an unterminated final line; in both cases
    // the seek target must not extend past the data we consumed -
    // backends such as ROMFS refuse to seek past the end of the file.
    const int32_t new_offset = offset_start + i + (i < n ? 1 : 0);
    if (backend.fs.lseek(fd, new_offset, SEEK_SET) != new_offset) {
        // we need to fail if we can't seek back or the caller may loop or get corrupt data
        return false;
    }

    return true;
}

// run crc32 over file with given name, returns true if successful
bool AP_Filesystem::crc32(const char *fname, uint32_t& checksum)
{
    // Ensure value is initialized
    checksum = 0;

    // Open file in readonly mode
    int fd = open(fname, O_RDONLY);
    if (fd == -1) {
        return false;
    }

    // Buffer to store data temporarily
    const ssize_t buff_len = 64;
    uint8_t buf[buff_len];

    // Read into buffer and run crc
    ssize_t read_size;
    do {
        read_size = read(fd, buf, buff_len);
        if (read_size == -1) {
            // Read error, note that we have changed the checksum value in this case
            close(fd);
            return false;
        }
        checksum = crc_crc32(checksum, buf, MIN(read_size, buff_len));
    } while (read_size > 0);

    close(fd);

    return true;
}


#if AP_FILESYSTEM_FORMAT_ENABLED
// format filesystem
bool AP_Filesystem::format(void)
{
    if (hal.util->get_soft_armed()) {
        return false;
    }
    return LOCAL_BACKEND.fs.format();
}
AP_Filesystem_Backend::FormatStatus AP_Filesystem::get_format_status(void) const
{
    return LOCAL_BACKEND.fs.get_format_status();
}
#endif

/*
  stat wrapper for scripting
 */
bool AP_Filesystem::stat(const char *pathname, stat_t &stbuf)
{
    struct stat st;
    if (fs.stat(pathname, &st) != 0) {
        return false;
    }
    stbuf.size = st.st_size;
    stbuf.mode = st.st_mode;
    // these wrap in 2038
    stbuf.atime = st.st_atime;
    stbuf.ctime = st.st_ctime;
    stbuf.mtime = st.st_mtime;
    return true;
}

// get_singleton for scripting
AP_Filesystem *AP_Filesystem::get_singleton(void)
{
    return &fs;
}

namespace AP
{
AP_Filesystem &FS()
{
    return fs;
}
}

#endif // AP_FILESYSTEM_FILE_READING_ENABLED

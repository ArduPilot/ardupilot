#include "FS.h"

#include <AP_HAL/AP_HAL.h>

#if HAL_OS_FATFS_IO

#include <stdio.h>

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

void FS::last_errorstring(char *ret, uint8_t retlen)
{
    snprintf(ret, retlen, "%s", strerror(errno));
}

int FS::open_for_read(const char *path)
{
    return ::open(path, O_RDONLY|O_CLOEXEC);
}
int FS::open_for_write(const char *path)
{
    return ::open(path, O_WRONLY|O_TRUNC|O_CREAT|O_CLOEXEC);
}

void FS::close(int fd)
{
    ::close(fd);
}

int32_t FS::seek_relative(int fd, int32_t off)
{
    return ::lseek(fd, off_t(off), SEEK_CUR);
}
int32_t FS::seek_absolute(int fd, uint32_t off)
{
    return ::lseek(fd, off_t(off), SEEK_SET);
}

int FS::unlink(const char *path)
{
    return ::unlink(path);
}

int32_t FS::read(int fd, uint8_t *buffer, uint32_t len)
{
    return ::read(fd, buffer, len);
}

int32_t FS::write(int fd, const uint8_t *buffer, uint32_t len)
{
    return ::write(fd, buffer, len);
}

bool FS::exists(const char *path)
{
    struct stat st;
    return ::stat(path, &st) == 0;
}

int FS::mkdir(const char *path, uint32_t perms)
{
    if (::mkdir(path, perms) == -1) {
        hal.console->printf("Failed to mkdir(%s): %s\n", path, strerror(errno));
        return -1;
    }
    return 0;
}

bool FS::get_file_size(const char *path, uint64_t &ret)
{
    struct stat st;
    if (::stat(path, &st) != 0) {
        return false;
    }
    ret = st.st_size;
    return true;
}

bool FS::get_file_modtime(const char *path, uint32_t &time_utc)
{
    struct stat st;
    if (::stat(path, &st) != 0) {
        return false;
    }
    time_utc = st.st_mtime;
    return true;
}

int64_t FS::disk_space_avail(const char *path)
{
    return fs_getfree();
}

int64_t FS::disk_space(const char *path)
{
    return fs_gettotal();
}

AP_HAL::FS::DIRHANDLE FS::opendir(const char *path)
{
    return (AP_HAL::FS::DIRHANDLE)::opendir(path);
}
bool FS::readdir(AP_HAL::FS::DIRHANDLE d, AP_HAL::FS::DIRENT &entry)
{
    if (!readdir_semaphore.take(10)) {
        return false;
    }

    dirent *ent = ::readdir((DIR*)d);
    if (ent == nullptr) {
        readdir_semaphore.give();
        return false;
    }
    strncpy(entry.d_name, ent->d_name, ARRAY_SIZE(entry.d_name));
    readdir_semaphore.give();
    return true;
}

void FS::closedir(AP_HAL::FS::DIRHANDLE d)
{
    ::closedir((DIR*)d);
}

#endif

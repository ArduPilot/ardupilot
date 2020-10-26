/*
  FATFS backend for AP_Filesystem
 */

#pragma once

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stddef.h>
#include <ff.h>
#include "AP_Filesystem_backend.h"

// Seek offset macros
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

#if FF_USE_LFN != 0
#define MAX_NAME_LEN FF_MAX_LFN 
#else
#define MAX_NAME_LEN 13
#endif

class AP_Filesystem_FATFS : public AP_Filesystem_Backend
{
public:
    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags) override;
    int close(int fd) override;
    int32_t read(int fd, void *buf, uint32_t count) override;
    int32_t write(int fd, const void *buf, uint32_t count) override;
    int fsync(int fd) override;
    int32_t lseek(int fd, int32_t offset, int whence) override;
    int stat(const char *pathname, struct stat *stbuf) override;
    int unlink(const char *pathname) override;
    int mkdir(const char *pathname) override;
    void *opendir(const char *pathname) override;
    struct dirent *readdir(void *dirp) override;
    int closedir(void *dirp) override;

    // return free disk space in bytes, -1 on error
    int64_t disk_free(const char *path) override;

    // return total disk space in bytes, -1 on error
    int64_t disk_space(const char *path) override;

    // set modification time on a file
    bool set_mtime(const char *filename, const uint32_t mtime_sec) override;

    // retry mount of filesystem if needed
    bool retry_mount(void) override;

    // unmount filesystem for reboot
    void unmount(void) override;
};

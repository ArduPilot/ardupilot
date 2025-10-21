#pragma once

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#include "AP_Filesystem_backend.h"

class AP_Filesystem_ESP32 : public AP_Filesystem_Backend
{
public:
    // functions that closely match the equivalent posix calls
    int open(const char *fname, int flags, bool allow_absolute_paths = false) override;
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
    int rename(const char *oldpath, const char *newpath) override;

    // return free disk space in bytes, -1 on error
    int64_t disk_free(const char *path) override;

    // return total disk space in bytes, -1 on error
    int64_t disk_space(const char *path) override;

    // set modification time on a file
    bool set_mtime(const char *filename, const uint32_t mtime_sec) override;
};


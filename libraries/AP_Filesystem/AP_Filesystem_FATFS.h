/*
  FATFS backend for AP_Filesystem
 */

#pragma once

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stddef.h>
#include "AP_Filesystem_backend.h"

// Seek offset macros
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

class AP_Filesystem_FATFS : public AP_Filesystem_Backend
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
    int rename(const char *oldpath, const char *newpath) override;
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

    // format sdcard.  This is async, monitor get_format_status for progress
    bool format(void) override;
    AP_Filesystem_Backend::FormatStatus get_format_status() const override;

private:
    void format_handler(void);
    FormatStatus format_status;
};

#pragma once

#include "AP_HAL_ChibiOS.h"
#include "AP_HAL_ChibiOS_Namespace.h"

#include <stdint.h>

namespace ChibiOS {

class FS : public AP_HAL::FS
{
public:
    void last_errorstring(char *ret, uint8_t retlen) override;

    int open_for_read(const char *path) override;
    int open_for_write(const char *path) override;
    void close(int fd);

    int32_t read(int fd, uint8_t *buffer, uint32_t len) override;
    int32_t write(int fd, const uint8_t *buffer, uint32_t len) override;

    int32_t seek_relative(int fd, int32_t off) override;
    int32_t seek_absolute(int fd, uint32_t off) override;

    bool exists(const char *path) override;

    int unlink(const char *path) override;

    bool get_file_modtime(const char *path, uint32_t &time_utc) override;
    bool get_file_size(const char *path, uint64_t &size) override;

    int64_t disk_space_avail(const char *path) override;
    int64_t disk_space(const char *path) override;

    DIRHANDLE opendir(const char *path) override;

    bool readdir(DIRHANDLE d, AP_HAL::FS::DIRENT &entry) override;
    void closedir(DIRHANDLE d) override;

    int mkdir(const char *path, uint32_t perms) override;

private:

    Semaphore readdir_semaphore;

};

}

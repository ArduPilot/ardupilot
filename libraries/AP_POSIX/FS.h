#pragma once

#include "AP_POSIX_HAL_Namespace.h"
#include <AP_HAL/AP_HAL.h>

#include <stdint.h>

class AP_POSIX_HAL::FS : public AP_HAL::FS
{
public:

    virtual void last_errorstring(char *ret, uint8_t retlen) override;

    virtual int open_for_read(const char *path) override;
    virtual int open_for_write(const char *path) override;
    virtual void close(int fd) override;

    virtual int32_t read(int fd, uint8_t *buffer, uint32_t len) override;
    virtual int32_t write(int fd, const uint8_t *buffer, uint32_t len) override;

    virtual int32_t seek_relative(int fd, int32_t off) override;
    virtual int32_t seek_absolute(int fd, uint32_t off) override;

    virtual bool exists(const char *path) override;

    virtual int unlink(const char *path) override;

    virtual int64_t disk_space_avail(const char *path) override;
    virtual int64_t disk_space(const char *path) override;

    virtual bool get_file_size(const char *path, uint64_t &size) override;
    virtual bool get_file_modtime(const char *path, uint32_t &time_utc) override;

    virtual int mkdir(const char *path, uint32_t perms) override;

    virtual DIRHANDLE opendir(const char *path) override;
    virtual bool readdir(DIRHANDLE d, AP_HAL::FS::DIRENT &entry) override;
    virtual void closedir(DIRHANDLE d) override;

private:

    HAL_Semaphore readdir_semaphore;

};

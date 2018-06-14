#pragma once

#include "AP_HAL_Namespace.h"

#include <stdint.h>

class AP_HAL::FS
{
public:
    // returns a null-terminated string describing the last error that
    // occured when using the library
    virtual void last_errorstring(char *ret, uint8_t retlen) = 0;

    virtual int open_for_read(const char *path) = 0;
    virtual int open_for_write(const char *path) = 0;
    virtual void close(int fd) = 0;

    virtual int32_t seek_relative(int fd, int32_t off) = 0;
    virtual int32_t seek_absolute(int fd, uint32_t off) = 0;

    virtual int32_t read(int fd, uint8_t *buffer, uint32_t len) = 0;
    virtual int32_t write(int fd, const uint8_t *buffer, uint32_t len) = 0;

    // returns true if path exists:
    virtual bool exists(const char *path) = 0;

    // make directory; returns false if it already exists
    virtual int mkdir(const char *path, uint32_t perms) = 0;
    // make directory and all bits up to it; returns true if it already exits
    // bool mkpath(const char *path, uint32_t perms);

    // returns the amount of disk space available in path (in bytes)
    // returns -1 on error
    virtual int64_t disk_space_avail(const char *path) = 0;
    // returns the total amount of disk space (in use + available) in
    // _log_directory (in bytes).
    // returns -1 on error
    virtual int64_t disk_space(const char *path) = 0;

    virtual bool get_file_size(const char *path, uint64_t &size) = 0;
    //returns the file modification time in Unix epoch time
    virtual bool get_file_modtime(const char *path, uint32_t &utc_time) = 0;

    // virtual int opendir(const char *path) = 0;
    // virtual void closedir(int dd) = 0;

    virtual int unlink(const char *path) = 0;

    typedef void* DIRHANDLE;
    class DIRENT {
    public:
        char d_name[32];
    };
    virtual DIRHANDLE opendir(const char *path) = 0;
    virtual bool readdir(DIRHANDLE d, DIRENT &entry) = 0;
    virtual void closedir(DIRHANDLE d) = 0;

};

#include "SITLFS.h"

#include <AP_HAL/HAL.h>

int HALSITL::SITLFS::open_for_read(const char *path)
{
    if (_num_open_fds >= _max_open_fds) {
        AP_HAL::panic("Too many open file descriptors");
    }
    // call superclass function:
    int ret = AP_POSIX_HAL::FS::open_for_read(path);
    if (ret != -1) {
        _open_fds[_num_open_fds++] = ret;
    }
    return ret;
}
int HALSITL::SITLFS::open_for_write(const char *path)
{
    if (_num_open_fds >= _max_open_fds) {
        AP_HAL::panic("Too many open file descriptors");
    }
    // call superclass function:
    int ret = AP_POSIX_HAL::FS::open_for_write(path);
    if (ret != -1) {
        _open_fds[_num_open_fds++] = ret;
    }
    return ret;
}

void HALSITL::SITLFS::close(int fd)
{
    bool found = false;
    uint8_t i;
    for (i=0; i<_num_open_fds; i++) {
        if (_open_fds[i] == fd) {
            found = true;
            break;
        }
    }
    if (!found) {
        AP_HAL::panic("Closing unopened file descriptor");
    }
    _num_open_fds--;
    memcpy(&_open_fds[i], &_open_fds[i+1], _num_open_fds-i);
    AP_POSIX_HAL::FS::close(fd);
}

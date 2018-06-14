#pragma once

#include "AP_HAL_SITL_Namespace.h"
#include <AP_HAL/FS.h>
#include <AP_POSIX/FS.h>

class HALSITL::SITLFS : public AP_POSIX_HAL::FS {
public:
    int open_for_read(const char *path) override;
    int open_for_write(const char *path) override;
    void close(int fd) override;

private:
    static const uint8_t _max_open_fds = 255;
    uint8_t _num_open_fds;
    uint8_t _open_fds[_max_open_fds];
};

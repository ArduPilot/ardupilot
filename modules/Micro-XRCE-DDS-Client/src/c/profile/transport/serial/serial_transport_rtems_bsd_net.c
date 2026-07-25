#include <uxr/client/profile/transport/serial/serial_transport_rtems_bsd_net.h>
#include <uxr/client/profile/transport/serial/serial_transport_platform.h>

#include <unistd.h>
#include <errno.h>

bool uxr_init_serial_platform(
        void* args,
        int fd,
        uint8_t remote_addr,
        uint8_t local_addr)
{
    (void) remote_addr;
    (void) local_addr;

    struct uxrSerialPlatform* platform = (struct uxrSerialPlatform*) args;

    platform->fd = fd;

    /* Poll setup. */
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wsign-conversion"
    FD_ZERO(&platform->select_fd);
    FD_SET(platform->fd, &platform->select_fd);
    #pragma GCC diagnostic pop
    return true;
}

bool uxr_close_serial_platform(
        void* args)
{
    struct uxrSerialPlatform* platform = (struct uxrSerialPlatform*) args;
    return (-1 == platform->fd) ? true : (0 == close(platform->fd));
}

size_t uxr_write_serial_data_platform(
        void* args,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode)
{
    size_t rv = 0;
    struct uxrSerialPlatform* platform = (struct uxrSerialPlatform*) args;

    ssize_t bytes_written = write(platform->fd, (void*)buf, (size_t)len);
    if (-1 != bytes_written)
    {
        rv = (size_t)bytes_written;
        *errcode = 0;
    }
    else
    {
        *errcode = 1;
    }
    return rv;
}

size_t uxr_read_serial_data_platform(
        void* args,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode)
{
    size_t rv = 0;
    struct uxrSerialPlatform* platform = (struct uxrSerialPlatform*) args;

    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

    fd_set fds = platform->select_fd;
    int32_t poll_rv = select(platform->fd + 1, &fds, NULL, NULL, &tv);
    if (0 < poll_rv)
    {
        ssize_t bytes_read = read(platform->fd, buf, len);
        if (-1 != bytes_read)
        {
            rv = (size_t)bytes_read;
            *errcode = 0;
        }
        else
        {
            *errcode = 1;
        }
    }
    else
    {
        *errcode = (0 == poll_rv) ? 0 : 1;
    }
    return rv;
}
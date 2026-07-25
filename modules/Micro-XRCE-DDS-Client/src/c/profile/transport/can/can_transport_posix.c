#include <uxr/client/profile/transport/can/can_transport_posix.h>
#include "can_transport_internal.h"

#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>

bool uxr_init_can_platform(
        uxrCANPlatform* platform,
        const char* dev,
        uint32_t can_id)
{
    static int enable_canfd = 1;
    bool rv = false;

    platform->poll_fd.fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (-1 != platform->poll_fd.fd)
    {
        struct sockaddr_can address;
        struct ifreq ifr;

        // Get interface index by name
        strcpy(ifr.ifr_name, dev);
        ioctl(platform->poll_fd.fd, SIOCGIFINDEX, &ifr);

        memset(&address, 0, sizeof(address));
        address.can_family = AF_CAN;
        address.can_ifindex = ifr.ifr_ifindex;

        if (-1 != bind(platform->poll_fd.fd,
                (struct sockaddr*) &address,
                sizeof(address)))
        {
            // Enable CAN FD
            if (-1 != setsockopt(platform->poll_fd.fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                    &enable_canfd, sizeof(enable_canfd)))
            {
                platform->poll_fd.events = POLLIN;
                platform->can_id = can_id | CAN_EFF_FLAG;

                // Add CAN filter for micro-ROS data
                struct can_filter rfilter;
                rfilter.can_id = platform->can_id;
                rfilter.can_mask = CAN_EFF_MASK;
                setsockopt(platform->poll_fd.fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
                rv = true;
            }
        }
    }
    return rv;
}

bool uxr_close_can_platform(
        uxrCANPlatform* platform)
{
    return (-1 == platform->poll_fd.fd) ? true : (0 == close(platform->poll_fd.fd));
}

size_t uxr_write_can_data_platform(
        uxrCANPlatform* platform,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode)
{
    struct canfd_frame frame = {
        .can_id = platform->can_id, .len = (uint8_t) (len + 1)
    };
    struct pollfd poll_fd_write_ = {
        .fd = platform->poll_fd.fd, .events = POLLOUT
    };
    size_t rv = 0;

    int poll_rv = poll(&poll_fd_write_, 1, 0);

    if (0 < poll_rv)
    {
        memcpy(&frame.data[1], buf, len);
        frame.data[0] = (uint8_t) len;

        ssize_t bytes_sent = write(poll_fd_write_.fd, &frame, sizeof(struct canfd_frame));

        if (-1 != bytes_sent)
        {
            rv = len;
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

size_t uxr_read_can_data_platform(
        uxrCANPlatform* platform,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode)
{
    (void) len;

    struct canfd_frame frame = {
        0
    };
    size_t rv = 0;

    int poll_rv = poll(&platform->poll_fd, 1, timeout);

    if (0 < poll_rv)
    {
        ssize_t bytes_received = read(platform->poll_fd.fd, &frame, sizeof(struct canfd_frame));

        if (-1 != bytes_received && frame.data[0] < CANFD_MTU)
        {
            memcpy(buf, &frame.data[1], frame.data[0]);
            rv = (size_t) frame.data[0];
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

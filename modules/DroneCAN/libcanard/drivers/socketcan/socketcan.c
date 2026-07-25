/*
 * Copyright (c) 2016-2018 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

// This is needed to enable necessary declarations in sys/
#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <net/if.h>
#include "socketcan.h"
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#ifdef __NuttX__
#include <nuttx/can.h>
#include <netpacket/can.h>
#else
#include <linux/can.h>
#include <linux/can/raw.h>
#endif
#include <errno.h>
#include <stdlib.h>

/// Returns the current errno as negated int16_t
static int16_t getErrorCode()
{
    const int out = -abs(errno);
    if (out < 0)
    {
        if (out >= INT16_MIN)
        {
            return (int16_t)out;
        }
        else
        {
            return INT16_MIN;
        }
    }
    else
    {
        assert(false);          // Requested an error when errno is zero?
        return INT16_MIN;
    }
}

#if CANARD_ENABLE_CANFD
int16_t socketcanInit(SocketCANInstance* out_ins, const char* can_iface_name, bool canfd)
#else
int16_t socketcanInit(SocketCANInstance* out_ins, const char* can_iface_name)
#endif
{
    const size_t iface_name_size = strlen(can_iface_name) + 1;
    if (iface_name_size > IFNAMSIZ)
    {
        goto fail0;
    }

    const int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);  // NOLINT
    if (fd < 0)
    {
        goto fail0;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    memcpy(ifr.ifr_name, can_iface_name, iface_name_size);

    const int ioctl_result = ioctl(fd, SIOCGIFINDEX, &ifr);
    if (ioctl_result < 0)
    {
        goto fail1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

#if CANARD_ENABLE_CANFD
    if(canfd)
    {
    	const int on = 1;
        if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0)
        {
            goto fail1;
        }
    }
    out_ins->canfd = canfd;
#endif

    const int bind_result = bind(fd, (struct sockaddr*)&addr, sizeof(addr));
    if (bind_result < 0)
    {
        goto fail1;
    }

    out_ins->fd = fd;
    return 0;

fail1:
    (void)close(fd);
fail0:
    return getErrorCode();
}

int16_t socketcanClose(SocketCANInstance* ins)
{
    const int close_result = close(ins->fd);
    ins->fd = -1;
    return (int16_t)((close_result == 0) ? 0 : getErrorCode());
}

int16_t socketcanTransmit(SocketCANInstance* ins, const CanardCANFrame* frame, int32_t timeout_msec)
{
    struct pollfd fds;
    memset(&fds, 0, sizeof(fds));
    fds.fd = ins->fd;
    fds.events |= POLLOUT;

    const int poll_result = poll(&fds, 1, timeout_msec);
    if (poll_result < 0)
    {
        return getErrorCode();
    }
    if (poll_result == 0)
    {
        return 0;
    }
    if (((uint32_t)fds.revents & (uint32_t)POLLOUT) == 0)
    {
        return -EIO;
    }

#if CANARD_ENABLE_CANFD
    if(frame->canfd)
    {
        struct canfd_frame transmit_frame;
        memset(&transmit_frame, 0, sizeof(transmit_frame));
        transmit_frame.can_id = frame->id;                  // TODO: Map flags properly
        transmit_frame.len = frame->data_len;
        memcpy(transmit_frame.data, frame->data, frame->data_len);
        const ssize_t nbytes = write(ins->fd, &transmit_frame, sizeof(transmit_frame));

        if (nbytes < 0)
        {
            return getErrorCode();
        }
        if ((size_t)nbytes != sizeof(transmit_frame))
        {
            return -EIO;
        }
    }
    else
#endif
    {
        struct can_frame transmit_frame;
        memset(&transmit_frame, 0, sizeof(transmit_frame));
        transmit_frame.can_id = frame->id;                  // TODO: Map flags properly
        transmit_frame.can_dlc = frame->data_len;
        memcpy(transmit_frame.data, frame->data, frame->data_len);
        const ssize_t nbytes = write(ins->fd, &transmit_frame, sizeof(transmit_frame));

        if (nbytes < 0)
        {
            return getErrorCode();
        }
        if ((size_t)nbytes != sizeof(transmit_frame))
        {
            return -EIO;
        }
    }

    return 1;
}

int16_t socketcanReceive(SocketCANInstance* ins, CanardCANFrame* out_frame, int32_t timeout_msec)
{
    struct pollfd fds;
    memset(&fds, 0, sizeof(fds));
    fds.fd = ins->fd;
    fds.events |= POLLIN;

    const int poll_result = poll(&fds, 1, timeout_msec);
    if (poll_result < 0)
    {
        return getErrorCode();
    }
    if (poll_result == 0)
    {
        return 0;
    }
    if (((uint32_t)fds.revents & (uint32_t)POLLIN) == 0)
    {
        return -EIO;
    }

#if CANARD_ENABLE_CANFD
    if(ins->canfd)
    {
        struct canfd_frame receive_frame;
        const ssize_t nbytes = read(ins->fd, &receive_frame, sizeof(receive_frame));
        if (nbytes < 0)
        {
            return getErrorCode();
        }
        if ((size_t)nbytes != sizeof(receive_frame))
        {
            return -EIO;
        }

        if (receive_frame.len > CANFD_MAX_DLEN)           // Appeasing Coverity Scan
        {
            return -EIO;
        }

        out_frame->id = receive_frame.can_id;               // TODO: Map flags properly
        out_frame->data_len = receive_frame.len;
        memcpy(out_frame->data, &receive_frame.data, receive_frame.len);
    }
    else
#endif
    {
        struct can_frame receive_frame;
        const ssize_t nbytes = read(ins->fd, &receive_frame, sizeof(receive_frame));
        if (nbytes < 0)
        {
            return getErrorCode();
        }
        if ((size_t)nbytes != sizeof(receive_frame))
        {
            return -EIO;
        }

        if (receive_frame.can_dlc > CAN_MAX_DLEN)           // Appeasing Coverity Scan
        {
            return -EIO;
        }

        out_frame->id = receive_frame.can_id;               // TODO: Map flags properly
        out_frame->data_len = receive_frame.can_dlc;
        memcpy(out_frame->data, &receive_frame.data, receive_frame.can_dlc);
    }

    // assume a single interface
    out_frame->iface_id = 0;

    return 1;
}

int socketcanGetSocketFileDescriptor(const SocketCANInstance* ins)
{
    return ins->fd;
}

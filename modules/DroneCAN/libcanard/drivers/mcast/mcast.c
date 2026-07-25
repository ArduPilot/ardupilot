/*
 * Copyright (c) 2023 DroneCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 */

#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <net/if.h>
#include "mcast.h"
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <fcntl.h>

#define MCAST_ADDRESS_BASE "239.65.82.0"
#define MCAST_PORT 57732U
#define MCAST_MAGIC 0x2934U
#define MCAST_FLAG_CANFD 0x0001
#define MCAST_MAX_PKT_LEN 74 // 64 byte data + 10 byte header

struct __attribute__((packed)) mcast_pkt
{
    uint16_t magic;
    uint16_t crc;
    uint16_t flags;
    uint32_t message_id;
    uint8_t data[MCAST_MAX_PKT_LEN-10];
};

// Returns the current errno as negated int16_t
static int16_t getErrorCode()
{
    const int out = -abs(errno);
    if (out < 0) {
        if (out >= INT16_MIN) {
            return (int16_t)out;
        } else {
            return INT16_MIN;
        }
    }
    assert(false);          // Requested an error when errno is zero?
    return INT16_MIN;
}

#if CANARD_ENABLE_CANFD
int16_t mcastInit(MCASTCANInstance* out_ins, const char* can_iface_name, bool canfd)
#else
int16_t mcastInit(MCASTCANInstance* out_ins, const char* can_iface_name)
#endif
{
    out_ins->fd_in = -1;
    out_ins->fd_out = -1;
    if (strncmp(can_iface_name, "mcast:", 6) != 0) {
        // invalid
        return -EINVAL;
    }
    int bus_num = 0;
    if (strlen(can_iface_name) > 6) {
        bus_num = atoi(can_iface_name+6);
        if (bus_num < 0 || bus_num > 9) {
            return -EINVAL;
        }
    }

    /*
      setup incoming multicast socket
     */
    char address[] = MCAST_ADDRESS_BASE;
    struct sockaddr_in sockaddr;
    int ret;

    memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif

    address[strlen(address)-1] = '0' + bus_num;

    sockaddr.sin_port = htons(MCAST_PORT);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(address);

    out_ins->fd_in = socket(AF_INET, SOCK_DGRAM, 0);
    if (out_ins->fd_in == -1) {
        goto fail;
    }
    ret = fcntl(out_ins->fd_in, F_SETFD, FD_CLOEXEC);
    if (ret == -1) {
        goto fail;
    }
    int one = 1;
    if (setsockopt(out_ins->fd_in, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) == -1) {
        goto fail;
    }

    // close on exec, to allow reboot
    fcntl(out_ins->fd_in, F_SETFD, FD_CLOEXEC);

    ret = bind(out_ins->fd_in, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        goto fail;
    }

    struct ip_mreq mreq;
    memset(&mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr.s_addr = inet_addr(address);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    ret = setsockopt(out_ins->fd_in, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    if (ret == -1) {
        goto fail;
    }

    // setup outgoing socket
    out_ins->fd_out = socket(AF_INET, SOCK_DGRAM, 0);
    if (out_ins->fd_out == -1) {
        goto fail;
    }
    ret = fcntl(out_ins->fd_out, F_SETFD, FD_CLOEXEC);
    if (ret == -1) {
        goto fail;
    }

    ret = connect(out_ins->fd_out, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        goto fail;
    }
    
    return 0;

fail:
    if (out_ins->fd_in != -1) {
        (void)close(out_ins->fd_in);
        out_ins->fd_in = -1;
    }
    if (out_ins->fd_out != -1) {
        (void)close(out_ins->fd_out);
        out_ins->fd_out = -1;
    }
    return getErrorCode();
}

int16_t mcastClose(MCASTCANInstance* ins)
{
    (void)close(ins->fd_in);
    (void)close(ins->fd_out);
    ins->fd_in = -1;
    ins->fd_out = -1;
    return 0;
}

/*
  CCITT 16 bit CRC with starting value 0xFFFF
 */
static uint16_t crc16_CCITT(const uint8_t *bytes, uint32_t len)
{
    uint16_t crc_val = 0xFFFFU;
    while (len--) {
        const uint16_t byte = *bytes++;
        crc_val ^= (uint16_t) (byte << 8U);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc_val & 0x8000U) {
                crc_val = (uint16_t) ((uint16_t) (crc_val << 1U) ^ 0x1021U);
            } else {
                crc_val = (uint16_t) (crc_val << 1U);
            }
        }
    }
    return crc_val;
}


int16_t mcastTransmit(MCASTCANInstance* ins, const CanardCANFrame* frame, int32_t timeout_msec)
{
    struct pollfd fds;
    memset(&fds, 0, sizeof(fds));
    fds.fd = ins->fd_out;
    fds.events |= POLLOUT;

    const int poll_result = poll(&fds, 1, timeout_msec);
    if (poll_result < 0) {
        return getErrorCode();
    }
    if (poll_result == 0) {
        return 0;
    }
    if (((uint32_t)fds.revents & (uint32_t)POLLOUT) == 0) {
        return -EIO;
    }

    struct mcast_pkt pkt;
    pkt.magic = MCAST_MAGIC;
    pkt.flags = 0;
#if CANARD_ENABLE_CANFD
    if (frame->canfd) {
        pkt.flags |= MCAST_FLAG_CANFD;
    }
#endif
    pkt.message_id = frame->id;
    memcpy(pkt.data, frame->data, frame->data_len);
    pkt.crc = crc16_CCITT((uint8_t*)&pkt.flags, frame->data_len+6);
    if (send(ins->fd_out, (void*)&pkt, frame->data_len+10, 0) <= 0) {
        return getErrorCode();
    }

    return 1;
}

int16_t mcastReceive(MCASTCANInstance* ins, CanardCANFrame* out_frame, int32_t timeout_msec)
{
    struct pollfd fds;
    memset(&fds, 0, sizeof(fds));
    fds.fd = ins->fd_in;
    fds.events |= POLLIN;

    memset(out_frame, 0, sizeof(*out_frame));

    const int poll_result = poll(&fds, 1, timeout_msec);
    if (poll_result < 0) {
        return getErrorCode();
    }
    if (poll_result == 0) {
        return 0;
    }
    if (((uint32_t)fds.revents & (uint32_t)POLLIN) == 0) {
        return -EIO;
    }
    struct mcast_pkt pkt;
    ssize_t ret = recv(ins->fd_in, (void*)&pkt, sizeof(pkt), 0);
    if (ret < 10) {
        return -EIO;
    }
    if (pkt.magic != MCAST_MAGIC) {
        return -EIO;
    }
    if (pkt.crc != crc16_CCITT((uint8_t*)&pkt.flags, ret-4)) {
        return -EIO;
    }

    out_frame->id = pkt.message_id;
#if CANARD_ENABLE_CANFD
    out_frame->canfd = (pkt.flags & MCAST_FLAG_CANFD) != 0;
#endif
    memcpy(out_frame->data, pkt.data, ret-10);
    out_frame->data_len = ret - 10;

    return 1;
}


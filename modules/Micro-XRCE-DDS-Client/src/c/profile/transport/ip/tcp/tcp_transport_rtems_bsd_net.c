// Copyright 2018 eSOL Co.,Ltd.
// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <uxr/client/profile/transport/ip/tcp/tcp_transport_rtems_bsd_net.h>
#include "tcp_transport_internal.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

bool uxr_init_tcp_platform(
        struct uxrTCPPlatform* platform,
        uxrIpProtocol ip_protocol,
        const char* ip,
        const char* port)
{
    bool rv = false;
    uint16_t iport;

    (void) ip_protocol;
    iport = (uint16_t)atoi(port);

    /* Socket initialization. */
    platform->fd = socket(AF_INET, SOCK_STREAM, 0);
    if (-1 != platform->fd)
    {
        /* Remote IP setup. */
        platform->remote_addr.sin_family = AF_INET;
        platform->remote_addr.sin_port = htons(iport);
        platform->remote_addr.sin_addr.s_addr = inet_addr(ip);

        /* Poll setup. */
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wsign-conversion"
        FD_ZERO(&platform->select_fd);
        FD_SET(platform->fd, &platform->select_fd);
        #pragma GCC diagnostic pop

        /* Server connection. */
        int connected = connect(platform->fd,
                        (struct sockaddr*)&platform->remote_addr,
                        sizeof(platform->remote_addr));
        rv = (0 == connected);
    }
    return rv;
}

bool uxr_close_tcp_platform(
        struct uxrTCPPlatform* platform)
{
    (void) shutdown(platform->fd, SHUT_RDWR);

    (void) close(platform->fd);

    return true;
}

size_t uxr_write_tcp_data_platform(
        struct uxrTCPPlatform* platform,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode)
{
    size_t rv = 0;

    int32_t bytes_sent = send(platform->fd, (void*)buf, len, 0);

    if (0 <= bytes_sent)
    {
        rv = (size_t)bytes_sent;
        *errcode = 0;
    }
    else
    {
        *errcode = 1;
    }

    return rv;
}

size_t uxr_read_tcp_data_platform(
        struct uxrTCPPlatform* platform,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode)
{
    size_t rv = 0;

    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

    fd_set fds = platform->select_fd;
    int32_t poll_rv = select(platform->fd + 1, &fds, NULL, NULL, &tv);
    if (0 < poll_rv)
    {
        int32_t bytes_received = recv(platform->fd, (void*)buf, len, 0);
        if (0 <= bytes_received)
        {
            rv = (size_t)bytes_received;
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

void uxr_disconnect_tcp_platform(
        struct uxrTCPPlatform* platform)
{
    (void) shutdown(platform->fd, SHUT_RDWR);

    (void) close(platform->fd);
    platform->fd = -1;
    FD_ZERO(&platform->select_fd);
}

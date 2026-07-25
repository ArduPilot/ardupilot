#include <uxr/client/profile/transport/custom/custom_transport.h>

#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <sys/poll.h>

static struct pollfd poll_fd;

bool my_custom_transport_open(
        uxrCustomTransport* transport)
{
    (void) transport;

    printf("Micro XRCE-DDS Client Custom transport: opening\n");

    bool rv = false;

    poll_fd.fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (-1 != poll_fd.fd)
    {
        struct addrinfo hints;
        struct addrinfo* result;
        struct addrinfo* ptr;

        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;

        if (0 == getaddrinfo("localhost", "8888", &hints, &result))
        {
            for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
            {
                if (0 == connect(poll_fd.fd, ptr->ai_addr, ptr->ai_addrlen))
                {
                    poll_fd.events = POLLIN;
                    rv = true;
                    break;
                }
            }
        }
        freeaddrinfo(result);
    }
    return rv;
}

bool my_custom_transport_close(
        uxrCustomTransport* transport)
{
    (void) transport;

    printf("Micro XRCE-DDS Client Custom transport: closing\n");

    return (-1 == poll_fd.fd) ? true : (0 == close(poll_fd.fd));
}

size_t my_custom_transport_write(
        uxrCustomTransport* transport,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode)
{
    (void) transport;

    size_t rv = 0;
    ssize_t bytes_sent = send(poll_fd.fd, (void*)buf, len, 0);
    if (-1 != bytes_sent)
    {
        rv = (size_t)bytes_sent;
        *errcode = 0;
    }
    else
    {
        *errcode = 1;
    }

    printf("Micro XRCE-DDS Client Custom transport: wrote %ld B\n", rv);

    return rv;
}

size_t my_custom_transport_read(
        uxrCustomTransport* transport,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode)
{
    (void) transport;

    size_t rv = 0;
    int poll_rv = poll(&poll_fd, 1, timeout);
    if (0 < poll_rv)
    {
        ssize_t bytes_received = recv(poll_fd.fd, (void*)buf, len, 0);
        if (-1 != bytes_received)
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

    printf("Micro XRCE-DDS Client Custom transport: read %ld B\n", rv);

    return rv;
}

#include <uxr/client/profile/multithread/multithread.h>
#include "udp_transport_internal.h"

/*******************************************************************************
* Static members.
*******************************************************************************/
static uint8_t error_code;

/*******************************************************************************
* Private function declarations.
*******************************************************************************/
static bool send_udp_msg(
        void* instance,
        const uint8_t* buf,
        size_t len);
static bool recv_udp_msg(
        void* instance,
        uint8_t** buf,
        size_t* len,
        int timeout);
static uint8_t get_udp_error(
        void);

/*******************************************************************************
* Private function definitions.
*******************************************************************************/
static bool send_udp_msg(
        void* instance,
        const uint8_t* buf,
        size_t len)
{
    bool rv = false;
    uxrUDPTransport* transport = (uxrUDPTransport*)instance;
    UXR_LOCK_TRANSPORT((&transport->comm));

    uint8_t errcode;
    size_t bytes_sent = uxr_write_udp_data_platform(&transport->platform, buf, len, &errcode);
    if (0 < bytes_sent)
    {
        rv = (bytes_sent == len);
    }
    else
    {
        error_code = errcode;
    }

    UXR_UNLOCK_TRANSPORT((&transport->comm));
    return rv;
}

static bool recv_udp_msg(
        void* instance,
        uint8_t** buf,
        size_t* len,
        int timeout)
{
    bool rv = false;
    uxrUDPTransport* transport = (uxrUDPTransport*)instance;
    UXR_LOCK_TRANSPORT((&transport->comm));

    uint8_t errcode;
    size_t bytes_received = uxr_read_udp_data_platform(&transport->platform,
                    transport->buffer,
                    sizeof(transport->buffer),
                    timeout,
                    &errcode);
    if (0 < bytes_received)
    {
        *buf = transport->buffer;
        *len = bytes_received;
        rv = true;
    }
    else
    {
        error_code = errcode;
    }

    UXR_UNLOCK_TRANSPORT((&transport->comm));
    return rv;
}

static uint8_t get_udp_error(
        void)
{
    return error_code;
}

/*******************************************************************************
* Public function definitions.
*******************************************************************************/
bool uxr_init_udp_transport(
        uxrUDPTransport* transport,
        uxrIpProtocol ip_protocol,
        const char* ip,
        const char* port)
{
    bool rv = false;

    if (uxr_init_udp_platform(&transport->platform, ip_protocol, ip, port))
    {
        /* Setup interface. */
        transport->comm.instance = (void*)transport;
        transport->comm.send_msg = send_udp_msg;
        transport->comm.recv_msg = recv_udp_msg;
        transport->comm.comm_error = get_udp_error;
        transport->comm.mtu = UXR_CONFIG_UDP_TRANSPORT_MTU;
        UXR_INIT_LOCK(&transport->comm.mutex);
        rv = true;
    }
    return rv;
}

bool uxr_close_udp_transport(
        uxrUDPTransport* transport)
{
    return uxr_close_udp_platform(&transport->platform);
}

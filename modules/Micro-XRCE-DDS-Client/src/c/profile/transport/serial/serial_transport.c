#include <uxr/client/profile/transport/serial/serial_transport_platform.h>
#include <uxr/client/profile/multithread/multithread.h>
#include "../stream_framing/stream_framing_protocol.h"
#include <uxr/client/util/time.h>

/*******************************************************************************
* Static members.
*******************************************************************************/
static uint8_t error_code;

/*******************************************************************************
* Private function declarations.
*******************************************************************************/
static bool send_serial_msg(
        void* instance,
        const uint8_t* buf,
        size_t len);
static bool recv_serial_msg(
        void* instance,
        uint8_t** buf,
        size_t* len,
        int timeout);
static uint8_t get_serial_error(
        void);

/*******************************************************************************
* Private function definitions.
*******************************************************************************/
static bool send_serial_msg(
        void* instance,
        const uint8_t* buf,
        size_t len)
{
    bool rv = false;
    uxrSerialTransport* transport = (uxrSerialTransport*)instance;
    UXR_LOCK_TRANSPORT((&transport->comm));

    uint8_t errcode;
    size_t bytes_written = uxr_write_framed_msg(&transport->framing_io,
                    uxr_write_serial_data_platform,
                    &transport->platform,
                    buf,
                    len,
                    transport->remote_addr,
                    &errcode);
    if ((0 < bytes_written) && (bytes_written == len))
    {
        rv = true;
    }
    else
    {
        error_code = errcode;
    }

    UXR_UNLOCK_TRANSPORT((&transport->comm));
    return rv;
}

static bool recv_serial_msg(
        void* instance,
        uint8_t** buf,
        size_t* len,
        int timeout)
{
    bool rv = false;
    uxrSerialTransport* transport = (uxrSerialTransport*)instance;
    UXR_LOCK_TRANSPORT((&transport->comm));

    size_t bytes_read = 0;
    uint8_t remote_addr = 0x00;
    uint8_t errcode;

    do
    {
        bytes_read = uxr_read_framed_msg(&transport->framing_io,
                        uxr_read_serial_data_platform,
                        &transport->platform,
                        transport->buffer,
                        sizeof(transport->buffer),
                        &remote_addr,
                        &timeout,
                        &errcode);
    }
    while ((0 == bytes_read) && (0 < timeout));

    if ((0 < bytes_read) && (remote_addr == transport->remote_addr))
    {
        *len = bytes_read;
        *buf = transport->buffer;
        rv = true;
    }
    else
    {
        error_code = errcode;
    }

    UXR_UNLOCK_TRANSPORT((&transport->comm));
    return rv;
}

static uint8_t get_serial_error(
        void)
{
    return error_code;
}

/*******************************************************************************
* Public function definitions.
*******************************************************************************/
bool uxr_init_serial_transport(
        uxrSerialTransport* transport,
        const int fd,
        uint8_t remote_addr,
        uint8_t local_addr)
{
    bool rv = false;
    if (uxr_init_serial_platform(&transport->platform, fd, remote_addr, local_addr))
    {
        /* Setup address. */
        transport->remote_addr = remote_addr;

        /* Init FramingIO. */
        uxr_init_framing_io(&transport->framing_io, local_addr);

        /* Setup interface. */
        transport->comm.instance = (void*)transport;
        transport->comm.send_msg = send_serial_msg;
        transport->comm.recv_msg = recv_serial_msg;
        transport->comm.comm_error = get_serial_error;
        transport->comm.mtu = UXR_CONFIG_SERIAL_TRANSPORT_MTU;
        UXR_INIT_LOCK(&transport->comm.mutex);
        rv = true;
    }
    return rv;
}

bool uxr_close_serial_transport(
        uxrSerialTransport* transport)
{
    return uxr_close_serial_platform(&transport->platform);
}

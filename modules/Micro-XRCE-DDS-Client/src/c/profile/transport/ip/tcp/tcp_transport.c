#include <uxr/client/profile/multithread/multithread.h>
#include "tcp_transport_internal.h"
#include <uxr/client/util/time.h>

#define UXR_MAX_WRITE_TCP_ATTEMPS 16

/*******************************************************************************
* Static members.
*******************************************************************************/
static uint8_t error_code;

/*******************************************************************************
* Private function declarations.
*******************************************************************************/
static bool send_tcp_msg(
        void* instance,
        const uint8_t* buf,
        size_t len);
static bool recv_tcp_msg(
        void* instance,
        uint8_t** buf,
        size_t* len,
        int timeout);
static uint8_t get_tcp_error(
        void);
static size_t read_tcp_data(
        uxrTCPTransport* transport,
        int timeout);

/*******************************************************************************
* Private function definitions.
*******************************************************************************/
bool send_tcp_msg(
        void* instance,
        const uint8_t* buf,
        size_t len)
{
    bool rv = false;
    uxrTCPTransport* transport = (uxrTCPTransport*)instance;
    UXR_LOCK_TRANSPORT((&transport->comm));

    uint8_t msg_size_buf[2];
    msg_size_buf[0] = (uint8_t)(0x00FF & len);
    msg_size_buf[1] = (uint8_t)((0xFF00 & len) >> 8);
    uint8_t n_attemps = 0;
    size_t bytes_sent = 0;

    /* Send message size. */
    bool size_sent = false;
    do
    {
        uint8_t errcode;
        size_t send_rv = uxr_write_tcp_data_platform(&transport->platform, msg_size_buf, 2, &errcode);
        if (0 < send_rv)
        {
            bytes_sent = (size_t)(bytes_sent + send_rv);
            size_sent = (sizeof(msg_size_buf) == bytes_sent);
        }
        else
        {
            if (0 < errcode)
            {
                error_code = errcode;
                break;
            }
        }
        ++n_attemps;
    }
    while (!size_sent && n_attemps < UXR_MAX_WRITE_TCP_ATTEMPS);

    /* Send message payload. */
    bool payload_sent = false;
    if (size_sent)
    {
        n_attemps = 0;
        bytes_sent = 0;
        do
        {
            uint8_t errcode;
            size_t send_rv = uxr_write_tcp_data_platform(&transport->platform,
                            buf + bytes_sent,
                            len - bytes_sent,
                            &errcode);
            if (0 < send_rv)
            {
                bytes_sent = (size_t)(bytes_sent + send_rv);
                payload_sent = (bytes_sent == len);
            }
            else
            {
                if (0 < errcode)
                {
                    error_code = errcode;
                    break;
                }
            }
            ++n_attemps;
        }
        while (!payload_sent && n_attemps < UXR_MAX_WRITE_TCP_ATTEMPS);
    }

    if (payload_sent)
    {
        rv = true;
    }
    else
    {
        uxr_disconnect_tcp_platform(&transport->platform);
    }

    UXR_UNLOCK_TRANSPORT((&transport->comm));
    return rv;
}

bool recv_tcp_msg(
        void* instance,
        uint8_t** buf,
        size_t* len,
        int timeout)
{
    bool rv = false;
    uxrTCPTransport* transport = (uxrTCPTransport*)instance;
    UXR_LOCK_TRANSPORT((&transport->comm));

    size_t bytes_read = 0;
    do
    {
        int64_t time_init = uxr_millis();
        bytes_read = read_tcp_data(transport, timeout);
        if (0 < bytes_read)
        {
            *buf = transport->input_buffer.buffer;
            *len = bytes_read;
            rv = true;
        }
        timeout -= (int)(uxr_millis() - time_init);
    }
    while ((0 == bytes_read) && (0 < timeout));

    UXR_UNLOCK_TRANSPORT((&transport->comm));
    return rv;
}

uint8_t get_tcp_error(
        void)
{
    return error_code;
}

size_t read_tcp_data(
        uxrTCPTransport* transport,
        int timeout)
{
    size_t rv = 0;
    bool exit_flag = false;

    /* State Machine. */
    while (!exit_flag)
    {
        switch (transport->input_buffer.state)
        {
            case UXR_TCP_BUFFER_EMPTY:
            {
                transport->input_buffer.position = 0;
                uint8_t size_buf[2];
                uint8_t errcode;
                size_t bytes_received =
                        uxr_read_tcp_data_platform(&transport->platform, size_buf, 2, timeout, &errcode);
                if (0 < bytes_received)
                {
                    transport->input_buffer.msg_size = 0;
                    if (2 == bytes_received)
                    {
                        transport->input_buffer.msg_size = (size_t)(((uint16_t)size_buf[1] << 8) | size_buf[0]);
                        if (transport->input_buffer.msg_size != 0)
                        {
                            transport->input_buffer.state = UXR_TCP_SIZE_READ;
                        }
                    }
                    else
                    {
                        transport->input_buffer.msg_size = (size_t)size_buf[0];
                        transport->input_buffer.state = UXR_TCP_SIZE_INCOMPLETE;
                    }
                }
                else
                {
                    if (0 < errcode)
                    {
                        uxr_disconnect_tcp_platform(&transport->platform);
                    }
                    error_code = errcode;
                    exit_flag = true;
                }
                break;
            }
            case UXR_TCP_SIZE_INCOMPLETE:
            {
                uint8_t size_msb;
                uint8_t errcode;
                size_t bytes_received =
                        uxr_read_tcp_data_platform(&transport->platform, &size_msb, 1, timeout, &errcode);
                if (0 < bytes_received)
                {
                    transport->input_buffer.msg_size = (size_t)(size_msb << 8) | transport->input_buffer.msg_size;
                    if (transport->input_buffer.msg_size != 0)
                    {
                        transport->input_buffer.state = UXR_TCP_SIZE_READ;
                    }
                    else
                    {
                        transport->input_buffer.state = UXR_TCP_BUFFER_EMPTY;
                    }
                }
                else
                {
                    if (0 < errcode)
                    {
                        uxr_disconnect_tcp_platform(&transport->platform);
                    }
                    error_code = errcode;
                    exit_flag = true;
                }
                break;
            }
            case UXR_TCP_SIZE_READ:
            {
                uint8_t errcode;
                size_t bytes_received = uxr_read_tcp_data_platform(&transport->platform,
                                transport->input_buffer.buffer,
                                transport->input_buffer.msg_size,
                                timeout,
                                &errcode);
                if (0 < bytes_received)
                {
                    if (bytes_received == transport->input_buffer.msg_size)
                    {
                        transport->input_buffer.state = UXR_TCP_MESSAGE_AVAILABLE;
                    }
                    else
                    {
                        transport->input_buffer.position = bytes_received;
                        transport->input_buffer.state = UXR_TCP_MESSAGE_INCOMPLETE;
                        exit_flag = true;
                    }
                }
                else
                {
                    if (0 < errcode)
                    {
                        uxr_disconnect_tcp_platform(&transport->platform);
                    }
                    error_code = errcode;
                    exit_flag = true;
                }
                break;
            }
            case UXR_TCP_MESSAGE_INCOMPLETE:
            {
                uint8_t errcode;
                size_t bytes_received = uxr_read_tcp_data_platform(&transport->platform,
                                transport->input_buffer.buffer +
                                transport->input_buffer.position,
                                (size_t)(transport->input_buffer.msg_size -
                                transport->input_buffer.position),
                                timeout,
                                &errcode);
                if (0 < bytes_received)
                {
                    transport->input_buffer.position = (size_t)(transport->input_buffer.position +  bytes_received);
                    if (transport->input_buffer.position == transport->input_buffer.msg_size)
                    {
                        transport->input_buffer.state = UXR_TCP_MESSAGE_AVAILABLE;
                    }
                    else
                    {
                        exit_flag = true;
                    }
                }
                else
                {
                    if (0 < errcode)
                    {
                        uxr_disconnect_tcp_platform(&transport->platform);
                    }
                    error_code = errcode;
                    exit_flag = true;
                }
                break;
            }
            case UXR_TCP_MESSAGE_AVAILABLE:
            {
                rv = transport->input_buffer.msg_size;
                transport->input_buffer.state = UXR_TCP_BUFFER_EMPTY;
                exit_flag = true;
                break;
            }
            default:
                rv = 0;
                exit_flag = true;
                break;
        }
    }

    return rv;
}

/*******************************************************************************
* Public function definitions.
*******************************************************************************/
bool uxr_init_tcp_transport(
        uxrTCPTransport* transport,
        uxrIpProtocol ip_protocol,
        const char* ip,
        const char* port)
{
    bool rv = false;

    if (uxr_init_tcp_platform(&transport->platform, ip_protocol, ip, port))
    {
        /* Interface setup. */
        transport->comm.instance = (void*)transport;
        transport->comm.send_msg = send_tcp_msg;
        transport->comm.recv_msg = recv_tcp_msg;
        transport->comm.comm_error = get_tcp_error;
        transport->comm.mtu = UXR_CONFIG_TCP_TRANSPORT_MTU;
        transport->input_buffer.state = UXR_TCP_BUFFER_EMPTY;
        UXR_INIT_LOCK(&transport->comm.mutex);
        rv = true;
    }

    return rv;
}

bool uxr_close_tcp_transport(
        uxrTCPTransport* transport)
{
    return uxr_close_tcp_platform(&transport->platform);
}

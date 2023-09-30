#include "AP_UROS_Client.h"

#if AP_UROS_UDP_ENABLED

#include <rmw_microros/custom_transport.h>

#include <errno.h>

/*
  open connection on UDP
 */
bool AP_UROS_Client::udp_transport_open(uxrCustomTransport *t)
{
    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    auto *sock = new SocketAPM(true);
    if (sock == nullptr) {
        return false;
    }
    if (!sock->connect(uros->udp.ip, uros->udp.port.get())) {
        return false;
    }
    uros->udp.socket = sock;
    return true;
}

/*
  close UDP connection
 */
bool AP_UROS_Client::udp_transport_close(uxrCustomTransport *t)
{
    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    delete uros->udp.socket;
    uros->udp.socket = nullptr;
    return true;
}

/*
  write on UDP
 */
size_t AP_UROS_Client::udp_transport_write(uxrCustomTransport *t,
    const uint8_t* buf, size_t len, uint8_t* error)
{
    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    if (uros->udp.socket == nullptr) {
        *error = EINVAL;
        return 0;
    }
    const ssize_t ret = uros->udp.socket->send(buf, len);
    if (ret <= 0) {
        *error = errno;
        return 0;
    }
    return ret;
}

/*
  read from UDP
 */
size_t AP_UROS_Client::udp_transport_read(uxrCustomTransport *t,
    uint8_t* buf, size_t len, int timeout_ms, uint8_t* error)
{
    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    if (uros->udp.socket == nullptr) {
        *error = EINVAL;
        return 0;
    }
    const ssize_t ret = uros->udp.socket->recv(buf, len, timeout_ms);
    if (ret <= 0) {
        *error = errno;
        return 0;
    }
    return ret;
}

/*
  initialise UDP connection
 */
bool AP_UROS_Client::urosUdpInit()
{
    // setup a non-framed transport for UDP
    rmw_ret_t rcl_ret = rmw_uros_set_custom_transport(
            false,
            (void*)this,
            udp_transport_open,
            udp_transport_close,
            udp_transport_write,
            udp_transport_read);

    return (rcl_ret == RCL_RET_OK);
}

#endif // AP_UROS_UDP_ENABLED

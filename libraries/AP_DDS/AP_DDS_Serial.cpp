#include "AP_DDS_Client.h"

#if AP_DDS_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>

#include <errno.h>

/*
  open connection on a serial port
 */
bool AP_DDS_Client::serial_transport_open(uxrCustomTransport *t)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    auto *dds_port = serial_manager->find_serial(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (dds_port == nullptr) {
        return false;
    }
    // ensure we own the UART
    dds_port->begin(0);
    dds->serial.port = dds_port;
    return true;
}

/*
  close serial transport
 */
bool AP_DDS_Client::serial_transport_close(uxrCustomTransport *t)
{
    // we don't actually close the UART
    return true;
}

/*
  write on serial transport
 */
size_t AP_DDS_Client::serial_transport_write(uxrCustomTransport *t, const uint8_t* buf, size_t len, uint8_t* error)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    if (dds->serial.port == nullptr) {
        *error = EINVAL;
        return 0;
    }
    ssize_t bytes_written = dds->serial.port->write(buf, len);
    if (bytes_written <= 0) {
        *error = 1;
        return 0;
    }
    //! @todo populate the error code correctly
    *error = 0;
    return bytes_written;
}

/*
  read from a serial transport
 */
size_t AP_DDS_Client::serial_transport_read(uxrCustomTransport *t, uint8_t* buf, size_t len, int timeout_ms, uint8_t* error)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    if (dds->serial.port == nullptr) {
        *error = EINVAL;
        return 0;
    }
    const uint32_t tstart = AP_HAL::millis();
    while (AP_HAL::millis() - tstart < uint32_t(timeout_ms) &&
           dds->serial.port->available() < len) {
        hal.scheduler->delay_microseconds(100); // TODO select or poll this is limiting speed (100us)
    }
    ssize_t bytes_read = dds->serial.port->read(buf, len);
    if (bytes_read <= 0) {
        *error = 1;
        return 0;
    }
    //! @todo Add error reporting
    *error = 0;
    return bytes_read;
}

/*
  initialise serial connection
 */
bool AP_DDS_Client::ddsSerialInit()
{
    // setup a framed transport for serial
    uxr_set_custom_transport_callbacks(&serial.transport, true,
                                       serial_transport_open,
                                       serial_transport_close,
                                       serial_transport_write,
                                       serial_transport_read);

    if (!uxr_init_custom_transport(&serial.transport, (void*)this)) {
        return false;
    }
    comm = &serial.transport.comm;
    return true;
}
#endif // AP_DDS_ENABLED
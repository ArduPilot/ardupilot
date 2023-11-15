#include "AP_UROS_Client.h"

#include <AP_SerialManager/AP_SerialManager.h>

#include <rmw_microros/custom_transport.h>

#include <errno.h>

/*
  open connection on a serial port
 */
bool AP_UROS_Client::serial_transport_open(uxrCustomTransport *t)
{
    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    AP_HAL::UARTDriver *uros_port = serial_manager->find_serial(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (uros_port == nullptr) {
        hal.console->printf("UROS: serial transport open... FAIL\n");
        return false;
    }

    // ensure we own the UART
    uros_port->begin(0);
    uros->serial.port = uros_port;

    return true;
}

/*
  close serial transport
 */
bool AP_UROS_Client::serial_transport_close(uxrCustomTransport *t)
{
    // we don't actually close the UART
    return true;
}

/*
  write on serial transport
 */
size_t AP_UROS_Client::serial_transport_write(uxrCustomTransport *t, const uint8_t* buf, size_t len, uint8_t* error)
{
    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    if (uros->serial.port == nullptr) {
        *error = EINVAL;
        return 0;
    }
    ssize_t bytes_written = uros->serial.port->write(buf, len);
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
size_t AP_UROS_Client::serial_transport_read(uxrCustomTransport *t, uint8_t* buf, size_t len, int timeout_ms, uint8_t* error)
{
    AP_UROS_Client *uros = (AP_UROS_Client *)t->args;
    if (uros->serial.port == nullptr) {
        *error = EINVAL;
        return 0;
    }
    const uint32_t tstart = AP_HAL::millis();
    while (AP_HAL::millis() - tstart < uint32_t(timeout_ms) &&
           uros->serial.port->available() < len) {
        hal.scheduler->delay_microseconds(100); // TODO select or poll this is limiting speed (100us)
    }
    ssize_t bytes_read = uros->serial.port->read(buf, len);
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
bool AP_UROS_Client::urosSerialInit()
{
    // fail transport initialisation if port not configured
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    auto *uros_port = serial_manager->find_serial(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (uros_port == nullptr) {
        hal.console->printf("UROS: serial port not configured\n");
        return false;
    }

    // setup a framed transport for serial
    hal.console->printf("UROS: initialising custom transport\n");
    rmw_ret_t rcl_ret = rmw_uros_set_custom_transport(
        true,
        (void *) &serial.transport,
        serial_transport_open,
        serial_transport_close,
        serial_transport_write,
        serial_transport_read
    );

    return (rcl_ret == RCL_RET_OK);
}

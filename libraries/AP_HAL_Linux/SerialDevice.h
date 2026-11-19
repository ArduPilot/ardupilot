#pragma once

#include <stdint.h>
#include <stdlib.h>

#include <AP_HAL/utility/RingBuffer.h>

#include "AP_HAL_Linux.h"

class SerialDevice {
public: 
    virtual ~SerialDevice() {}

    virtual bool open() = 0;
    virtual bool close() = 0;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) = 0;
    virtual ssize_t read(uint8_t *buf, uint16_t n) = 0;

    // true if read() delivers whole datagrams (UDP): a datagram that does
    // not fit into the buffer given to read() is cut and the rest is lost
    virtual bool is_datagram() const { return false; }

    /*
      read whole datagrams into the free space of a ring buffer, which can
      be split in two at its end (vec[0], then vec[1]), so that a datagram
      is never cut there. max_space is the free space of the empty ring
      buffer: a larger datagram can never fit and is read cut. Returns the
      number of bytes read or -1. Only used if is_datagram()
     */
    virtual ssize_t read_datagrams(const ByteBuffer::IoVec *vec, uint8_t n_vec, uint32_t max_space) { return -1; }

    virtual void set_blocking(bool blocking) = 0;
    virtual void set_speed(uint32_t speed) = 0;
    virtual AP_HAL::UARTDriver::flow_control get_flow_control(void) { return AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE; }
    virtual void set_flow_control(AP_HAL::UARTDriver::flow_control flow_control_setting)
    {
        /* most devices simply ignore this setting */
    };

    /* Depends on lower level to implement, most devices are fine with defaults */
    virtual void set_parity(int v) { }
};

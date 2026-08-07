/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "AP_HAL_QURT.h"
#include "Semaphores.h"
#include <AP_HAL/utility/RingBuffer.h>
#include "ap_host/src/protocol.h"

class QURT::UARTDriver : public AP_HAL::UARTDriver
{
public:
    bool is_initialized() override;
    bool tx_pending() override;

    /* Empty implementations of Stream virtual methods */
    uint32_t txspace() override;

    virtual bool _write_pending_bytes(void)
    {
        return false;
    }
    virtual void _timer_tick(void) override;
    virtual void _fill_read_buffer(void) {}

    virtual uint32_t bw_in_bytes_per_second() const override
    {
        return 5760;
    }
    virtual enum AP_HAL::UARTDriver::flow_control get_flow_control(void) override
    {
        return AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
    }

protected:
    virtual void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t size) override WARN_IF_UNUSED;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;
    volatile bool _initialised;

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};

    QURT::Semaphore _read_mutex;
    QURT::Semaphore _write_mutex;
};

/*
  subclass for console output, maps to HAP_PRINTF
*/
class QURT::UARTDriver_Console : public QURT::UARTDriver
{
public:
    using UARTDriver::UARTDriver;
    virtual void printf(const char *fmt, ...) override;
};

/*
  subclass for MAVLink UDP communications
*/

class QURT::UARTDriver_MAVLinkUDP : public QURT::UARTDriver
{
public:
    UARTDriver_MAVLinkUDP(uint8_t instance);

    bool _write_pending_bytes(void) override;

    void check_rx_seq(uint32_t seq);

    uint32_t bw_in_bytes_per_second() const override
    {
        return 250000 * 3;
    }
    enum AP_HAL::UARTDriver::flow_control get_flow_control(void) override
    {
        return AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE;
    }

private:
    static void _mavlink_data_cb(const struct qurt_rpc_msg *msg, void *p);
    uint8_t inst;
    uint32_t tx_seq;
    uint32_t rx_seq;
};

/*
  subclass for local UART communications
*/
class QURT::UARTDriver_Local : public QURT::UARTDriver
{
public:
    UARTDriver_Local(uint8_t _port_id) : port_id(_port_id) {}

    uint32_t bw_in_bytes_per_second() const override
    {
        return baudrate?baudrate/10:5760;
    }

    bool _write_pending_bytes(void) override;
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void _fill_read_buffer(void) override;

    uint32_t get_baud_rate() const override
    {
        return baudrate;
    }

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart.
    */
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

private:
    const uint8_t port_id;
    int fd = -1;
    uint32_t baudrate;
    uint64_t receive_timestamp_us;
};

#pragma once

#include <stdint.h>

#include "AP_HAL_Namespace.h"
#include "utility/BetterStream.h"

#ifndef HAL_UART_STATS_ENABLED
#define HAL_UART_STATS_ENABLED !defined(HAL_NO_UARTDRIVER)
#endif

#ifndef AP_UART_MONITOR_ENABLED
#define AP_UART_MONITOR_ENABLED 0
#endif

class ExpandingString;
class ByteBuffer;

/* Pure virtual UARTDriver class */
class AP_HAL::UARTDriver : public AP_HAL::BetterStream {
public:
    UARTDriver() {}
    /* Do not allow copies */
    CLASS_NO_COPY(UARTDriver);

    /*
      the individual HALs need to implement the protected versions of these calls, _begin(), _write(), _read()
      port locking is checked by the top level AP_HAL functions
     */
    void begin(uint32_t baud);

    /*
      begin with specified minimum tx and rx buffer sizes
     */
    void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace);

    /*
      begin for use when the port is write locked. Note that this does
      not lock the port, an existing write_key from lock_port() must
      be used
     */
    void begin_locked(uint32_t baud, uint16_t rxSpace, uint16_t txSpace, uint32_t write_key);

    /*
      single and multi-byte write methods
     */
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    size_t write(const char *str) override;

    /*
      single and multi-byte read methods
     */
    int16_t read(void) override;
    bool read(uint8_t &b) override WARN_IF_UNUSED;
    ssize_t read(uint8_t *buffer, uint16_t count) override;
    
    void end();
    void flush();

    virtual bool is_initialized() = 0;
    virtual bool tx_pending() = 0;

    // lock a port for exclusive use. Use a key of 0 to unlock
    bool lock_port(uint32_t write_key, uint32_t read_key);

    // check data available for read, return 0 is not available
    uint32_t available() override;
    uint32_t available_locked(uint32_t key);

    // discard any pending input
    bool discard_input() override;

    // write to a locked port. If port is locked and key is not correct then 0 is returned
    // and write is discarded
    size_t write_locked(const uint8_t *buffer, size_t size, uint32_t key);

    // read buffer from a locked port. If port is locked and key is not correct then -1 is returned
    ssize_t read_locked(uint8_t *buf, size_t count, uint32_t key) WARN_IF_UNUSED;
    
    // control optional features
    virtual bool set_options(uint16_t options) { _last_options = options; return options==0; }
    virtual uint16_t get_options(void) const { return _last_options; }

    enum {
        OPTION_RXINV              = (1U<<0),  // invert RX line
        OPTION_TXINV              = (1U<<1),  // invert TX line
        OPTION_HDPLEX             = (1U<<2), // half-duplex (one-wire) mode
        OPTION_SWAP               = (1U<<3), // swap RX and TX pins
        OPTION_PULLDOWN_RX        = (1U<<4), // apply pulldown to RX
        OPTION_PULLUP_RX          = (1U<<5), // apply pullup to RX
        OPTION_PULLDOWN_TX        = (1U<<6), // apply pulldown to TX
        OPTION_PULLUP_TX          = (1U<<7), // apply pullup to TX
        OPTION_NODMA_RX           = (1U<<8), // don't use DMA for RX
        OPTION_NODMA_TX           = (1U<<9), // don't use DMA for TX
        OPTION_MAVLINK_NO_FORWARD = (1U<<10), // don't forward MAVLink data to or from this device
        OPTION_NOFIFO             = (1U<<11), // disable hardware FIFO
        OPTION_NOSTREAMOVERRIDE   = (1U<<12), // don't allow GCS to override streamrates
    };

    enum flow_control {
        FLOW_CONTROL_DISABLE=0,
        FLOW_CONTROL_ENABLE=1,
        FLOW_CONTROL_AUTO=2,
    };
    virtual void set_flow_control(enum flow_control flow_control_setting) {};
    virtual enum flow_control get_flow_control(void) { return FLOW_CONTROL_DISABLE; }

    virtual void configure_parity(uint8_t v){};
    virtual void set_stop_bits(int n){};

    /* unbuffered writes bypass the ringbuffer and go straight to the
     * file descriptor
     */
    virtual bool set_unbuffered_writes(bool on){ return false; };

    /*
      wait for at least n bytes of incoming data, with timeout in
      milliseconds. Return true if n bytes are available, false if
      timeout
     */
    virtual bool wait_timeout(uint16_t n, uint32_t timeout_ms) { return false; }

    /*
     * Optional method to control the update of the motors. Derived classes
     * can implement it if their HAL layer requires.
     */
    virtual void _timer_tick(void) { }

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart. This should be treated as a
      time constraint, not an exact time. It is guaranteed that the
      packet did not start being received after this time, but it
      could have been in a system buffer before the returned time.

      This takes account of the baudrate of the link. For transports
      that have no baudrate (such as USB) the time estimate may be
      less accurate.
     */
    virtual uint64_t receive_time_constraint_us(uint16_t nbytes);

    virtual uint32_t bw_in_bytes_per_second() const {
        return 5760;
    }

    virtual uint32_t get_baud_rate() const { return 0; }

    /*
      return true if this UART has DMA enabled on both RX and TX
     */
    virtual bool is_dma_enabled() const { return false; }

#if HAL_UART_STATS_ENABLED
    // request information on uart I/O for this uart, for @SYS/uarts.txt
    virtual void uart_info(ExpandingString &str) {}
#endif

    /*
      software control of the CTS/RTS pins if available. Return false if
      not available
     */
    virtual bool set_RTS_pin(bool high) { return false; };
    virtual bool set_CTS_pin(bool high) { return false; };

    // return true requested baud on USB port
    virtual uint32_t get_usb_baud(void) const { return 0; }

    // disable TX/RX pins for unusued uart
    virtual void disable_rxtx(void) const {}

#if AP_UART_MONITOR_ENABLED
    // a way to monitor all reads from the UART, putting them in a
    // buffer. Used by AP_Periph for GPS debug
    bool set_monitor_read_buffer(ByteBuffer *buffer) {
        _monitor_read_buffer = buffer;
        return true;
    }
#endif

    /*
      return true if the port is currently locked for writing
     */
    bool is_write_locked(void) const {
        return lock_write_key != 0;
    }

protected:
    // key for a locked port
    uint32_t lock_write_key;
    uint32_t lock_read_key;

    /*
      backend begin method
     */
    virtual void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) = 0;

    /*
      backend write method
     */
    virtual size_t _write(const uint8_t *buffer, size_t size) = 0;

    /*
      backend read method
     */
    virtual ssize_t _read(uint8_t *buffer, uint16_t count)  WARN_IF_UNUSED = 0;

    /*
      end control of the port, freeing buffers
     */
    virtual void _end() = 0;

    /*
      flush any pending data
     */
    virtual void _flush() = 0;

    // check available data on the port
    virtual uint32_t _available() = 0;

    // discard incoming data on the port
    virtual bool _discard_input(void) = 0;
    
private:
    // option bits for port
    uint16_t _last_options;

#if AP_UART_MONITOR_ENABLED
    ByteBuffer *_monitor_read_buffer;
#endif
};

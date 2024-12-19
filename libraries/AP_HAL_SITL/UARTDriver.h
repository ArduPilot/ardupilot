#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <stdint.h>
#include <stdarg.h>
#include <deque>
#include <vector>
#include "AP_HAL_SITL_Namespace.h"
#include <AP_HAL/utility/Socket_native.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_CSVReader/AP_CSVReader.h>
#include <AP_HAL/utility/DataRateLimit.h>

#include <SITL/SIM_SerialDevice.h>

// Timestamped packets to push into a double-ended queue to simulate latency
struct TimestampedData {
    uint64_t timestamp_us;
    std::vector<uint8_t> data;
};

class HALSITL::UARTDriver : public AP_HAL::UARTDriver {
public:
    friend class HALSITL::SITL_State;

    UARTDriver(const uint8_t portNumber, SITL_State *sitlState) {
        _portNumber = portNumber;
        _sitlState = sitlState;

        _fd = -1;
        _mc_fd = -1;
        _listen_fd = -1;
    }

    bool is_initialized() override {
        return true;
    }

    ssize_t get_system_outqueue_length() const;

    bool tx_pending() override {
        return false;
    }

    /* Implementations of Stream virtual methods */
    uint32_t txspace() override;

    bool _unbuffered_writes;

    enum flow_control get_flow_control(void) override { return FLOW_CONTROL_ENABLE; }

    void configure_parity(uint8_t v) override;
    void set_stop_bits(int n) override;
    bool set_unbuffered_writes(bool on) override;

    uint32_t bw_in_bytes_per_second() const override;

    void _timer_tick(void) override;

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart. This should be treated as a
      time constraint, not an exact time. It is guaranteed that the
      packet did not start being received after this time, but it
      could have been in a system buffer before the returned time.

      This takes account of the baudrate of the link. For transports
      that have no baudrate (such as USB) the time estimate may be
      less accurate.

      A return value of zero means the HAL does not support this API
     */
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

    uint32_t get_baud_rate() const override { return _uart_baudrate; }

#if HAL_UART_STATS_ENABLED
    // request information on uart I/O
    void uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms) override;
#endif

private:

    int _fd;

    // file descriptor for reading multicast packets
    int _mc_fd;

    uint8_t _portNumber;
    bool _connected = false; // true if a client has connected
    bool _use_send_recv = false;
    int _listen_fd;  // socket we are listening on
    int _serial_port;
    static bool _console;
    ByteBuffer _readbuffer{16384};
    ByteBuffer _writebuffer{16384};

    // default multicast IP and port
    const char *mcast_ip_default = "239.255.145.50";
    const uint16_t mcast_port_default = 14550;

    const char *_uart_path;
    uint32_t _uart_baudrate;

    void _tcp_start_connection(uint16_t port, bool wait_for_connection);
    void _uart_start_connection(void);
    void _check_reconnect();
    void _tcp_start_client(const char *address, uint16_t port);
    void _udp_start_client(const char *address, uint16_t port);
    void _udp_start_multicast(const char *address, uint16_t port);
    void _check_connection(void);
    static bool _select_check(int );
    static void _set_nonblocking(int );
    bool set_speed(int speed) const;

    SITL_State *_sitlState;
    uint64_t _receive_timestamp;
    bool _is_udp;
    bool _packetise;
    uint16_t _mc_myport;

    // for baud-rate limiting:
    struct {
        DataRateLimit write;
        DataRateLimit read;
    } baud_limits;

    HAL_Semaphore write_mtx;

    SITL::SerialDevice *_sim_serial_device;

    // Double-ended queues to simulate latency
    std::deque<TimestampedData> latencyQueueWrite;
    std::deque<TimestampedData> latencyQueueRead;

    struct {
        bool active;
        uint8_t term[20];
        AP_CSVReader csvreader{term, sizeof(term), ','};
        struct {
            uint32_t timestamp_us;
            uint8_t b;  // the byte
        } loaded_data;
        bool loaded;  // true if data is all valid
        bool done_first_line = false;
        uint8_t terms_seen;
        uint32_t first_timestamp_us;
        uint32_t first_emit_micros_us;
    } logic_async_csv;
    uint16_t read_from_async_csv(uint8_t *buffer, uint16_t space);

protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    uint32_t _available() override;
    void _end() override;
    void _flush() override;
    bool _discard_input() override;

#if HAL_UART_STATS_ENABLED
    // Getters for cumulative tx and rx counts
    uint32_t get_total_tx_bytes() const override { return _tx_stats_bytes; }
    uint32_t get_total_rx_bytes() const override { return _rx_stats_bytes; }
#endif

private:
    void handle_writing_from_writebuffer_to_device();
    void handle_reading_from_device_to_readbuffer();

    // statistics
    uint32_t _tx_stats_bytes;
    uint32_t _rx_stats_bytes;

};

#endif

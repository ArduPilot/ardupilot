#pragma once

#include <AP_SerialManager/AP_SerialManager.h>

#ifndef AP_DRONECAN_SERIAL_NUM_PORTS
#define AP_DRONECAN_SERIAL_NUM_PORTS 3
#endif

class AP_DroneCAN;

class AP_DroneCAN_Serial
{
public:
    /* Do not allow copies */
    CLASS_NO_COPY(AP_DroneCAN_Serial);

    AP_DroneCAN_Serial() {}

    AP_Int8 enable;

    void init(AP_DroneCAN *dronecan);
    void update(void);

public:
    class Port : public AP_SerialManager::RegisteredPort {
    public:
        friend class AP_DroneCAN_Serial;
        void init(void);

        AP_Int8 node;
        AP_Int8 idx;

    private:
        bool is_initialized() override {
            return true;
        }
        bool tx_pending() override {
            return false;
        }

        bool init_buffers(const uint32_t size_rx, const uint32_t size_tx);

        uint32_t txspace() override;
        void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
        size_t _write(const uint8_t *buffer, size_t size) override;
        ssize_t _read(uint8_t *buffer, uint16_t count) override;
        uint32_t _available() override;
        void _end() override {}
        void _flush() override {}
        bool _discard_input() override;
        uint64_t receive_time_constraint_us(uint16_t nbytes) override;

        ByteBuffer *readbuffer;
        ByteBuffer *writebuffer;
        uint32_t baudrate;
        uint32_t last_send_ms;
        uint32_t last_size_tx;
        uint32_t last_size_rx;
        uint64_t last_recv_us;

        // statistics
        uint32_t tx_stats_bytes;
        uint32_t rx_stats_bytes;
        uint32_t rx_stats_dropped_bytes;

        HAL_Semaphore sem;

    protected:

#if HAL_UART_STATS_ENABLED
        // Getters for cumulative tx and rx counts
        uint32_t get_total_tx_bytes() const override { return tx_stats_bytes; }
        uint32_t get_total_rx_bytes() const override { return rx_stats_bytes; }
        uint32_t get_total_dropped_rx_bytes() const override { return rx_stats_dropped_bytes; }
#endif
    };

    Port ports[AP_DRONECAN_SERIAL_NUM_PORTS];

private:
    AP_DroneCAN *dronecan;

    Canard::Publisher<uavcan_tunnel_Targetted> *targetted;
    static void handle_tunnel_targetted(AP_DroneCAN *dronecan,
                                        const CanardRxTransfer& transfer,
                                        const uavcan_tunnel_Targetted &msg);

    static AP_DroneCAN_Serial *serial[HAL_MAX_CAN_PROTOCOL_DRIVERS];
};

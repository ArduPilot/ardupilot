#pragma once

#include <AP_SerialManager/AP_SerialManager.h>

#ifndef AP_DRONECAN_SERIAL_NUM_PORTS
#define AP_DRONECAN_SERIAL_NUM_PORTS 3
#endif

class AP_UAVCAN;
class TunnelTargettedCb;

class AP_DroneCAN_Serial
{
public:
    /* Do not allow copies */
    CLASS_NO_COPY(AP_DroneCAN_Serial);

    AP_DroneCAN_Serial() {}

    AP_Int8 enable;

    void init(AP_UAVCAN *dronecan);
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
        void begin(uint32_t b) override {
            begin(b, 0, 0);
        }
        void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
        enum flow_control get_flow_control(void) override { return FLOW_CONTROL_ENABLE; }
        size_t write(uint8_t c) override;
        size_t write(const uint8_t *buffer, size_t size) override;
        int16_t read() override;
        ssize_t read(uint8_t *buffer, uint16_t count) override;
        uint32_t available() override;
        void end() override {}
        void flush() override {}
        bool discard_input() override;
        uint64_t receive_time_constraint_us(uint16_t nbytes) override;
        void set_blocking_writes(bool blocking) override {}

        ByteBuffer *readbuffer;
        ByteBuffer *writebuffer;
        uint32_t baudrate;
        uint32_t last_send_ms;
        uint32_t last_size_tx;
        uint32_t last_size_rx;
        uint64_t last_recv_us;

        HAL_Semaphore sem;
    };

    Port ports[AP_DRONECAN_SERIAL_NUM_PORTS];

private:
    AP_UAVCAN *dronecan;

    static void handle_tunnel_targetted(AP_UAVCAN* ap_uavcan, uint8_t node_id, const TunnelTargettedCb &cb);
    static AP_DroneCAN_Serial *serial[HAL_MAX_CAN_PROTOCOL_DRIVERS];
};

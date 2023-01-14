#pragma once

#include "AP_ESC_Telem_config.h"

class AP_HobbyWing_ESC {
public:
    AP_HobbyWing_ESC(AP_HAL::UARTDriver &_uart)
        : uart(_uart) {}

    void init();
    virtual void update() = 0;

    bool read_uart_v345(uint8_t *packet, uint8_t packet_len, uint16_t frame_gap_us);

    struct HWESC {
        float rpm;
        float voltage;
        float phase_current;
        float current;
        float temperature;
        uint32_t error_count;
    };

    // thread-safe method to get telemetry.  This "consumes" the
    // current telemetry packet, so unless another packet is received
    // subsequent calls will return false.
    bool get_telem(HWESC &hwesc);

    void set_poles(uint8_t _poles) { motor_poles = _poles; }

protected:

    void check_seq(uint32_t this_seq);

    struct HWESC decoded;
    HAL_Semaphore decoded_sem;
    uint32_t decoded_received_us;

    uint8_t motor_poles;

private:
    AP_HAL::UARTDriver &uart;

    uint64_t last_frame_us;

    uint32_t last_seq;
    uint32_t lost_count;
};

#pragma once

#include "AP_ESC_Telem_config.h"

#include "AP_ESC_Telem_Backend.h"

class AP_HobbyWing_ESC : public AP_ESC_Telem_Backend {
public:
    AP_HobbyWing_ESC(AP_HAL::UARTDriver &_uart, uint8_t _servo_channel, uint8_t poles)
        : uart(_uart),
          servo_channel(_servo_channel),
          motor_poles{poles}
        {}

    void init();
    virtual void update() = 0;

    bool read_uart_v345(uint8_t *packet, uint8_t packet_len, uint16_t frame_gap_us);

    // thread-safe method to get telemetry.  This "consumes" the
    // current telemetry packet, so unless another packet is received
    // subsequent calls will return false.
    bool get_telem(HWESC &hwesc);
#if HAL_WITH_ESC_TELEM
    void update_telemetry();
#endif

    void set_poles(uint8_t _poles) { motor_poles = _poles; }

    static uint8_t temperature_decode(uint8_t temp_raw);


protected:

    void check_seq(uint32_t this_seq);

    struct HWESC decoded;
    HAL_Semaphore decoded_sem;
    uint32_t decoded_received_us;

    uint8_t motor_poles;
    uint8_t servo_channel;

private:
    AP_HAL::UARTDriver &uart;

    uint64_t last_frame_us;

    uint32_t last_seq;
    uint32_t lost_count;
};

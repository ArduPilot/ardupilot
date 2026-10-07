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

#include "SIM_config.h"

#if AP_SIM_SERIALPLANT_ENABLED

#include "SIM_Aircraft.h"
#include <AP_HAL/AP_HAL.h>

namespace SITL {

class SerialPlant : public Aircraft {
public:
    SerialPlant(const char *frame_str);

    void update(const struct sitl_input &input) override;

    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW SerialPlant(frame_str);
    }

private:
    // protocol constants
    static constexpr uint8_t SYNC1 = 0xA5;
    static constexpr uint8_t SYNC2 = 0x5A;
    static constexpr uint8_t MSG_SERVO = 0x01;
    static constexpr uint8_t MSG_STATE = 0x02;
    static constexpr uint8_t FRAME_OVERHEAD = 5; // sync(2) + id(1) + len(2)

    // state mask bits (first byte of state payload)
    static constexpr uint8_t MASK_USE_QUATERNION   = (1 << 0); // bit 0: quaternion(16) vs attitude(12)
    static constexpr uint8_t MASK_RANGEFINDER      = (1 << 1); // bit 1: rangefinder float*6 (24 bytes)
    static constexpr uint8_t MASK_AIRSPEED         = (1 << 2); // bit 2: airspeed float (4 bytes)
    static constexpr uint8_t MASK_WINDVANE         = (1 << 3); // bit 3: windvane float*2 (8 bytes)

    // mandatory payload after mask byte: timestamp(8)+gyro(12)+accel(12)+position(24)+velocity(12) = 68
    static constexpr uint16_t MANDATORY_LEN = 69; // 1 (mask) + 68 (mandatory fields)

    // maximum state payload: mask(1) + mandatory(68) + quat(16) + rangefinder(24) + airspeed(4) + windvane(8) = 121
    static constexpr uint16_t STATE_BUF_MAX = 121;

    // servo packets sent to Simulink plant (16 or 32 channels)
    struct PACKED servo_payload_16 {
        uint32_t frame_count;
        uint16_t frame_rate_hz;
        uint16_t pwm[16];
    };

    struct PACKED servo_payload_32 {
        uint32_t frame_count;
        uint16_t frame_rate_hz;
        uint16_t pwm[32];
    };

    static constexpr uint16_t SERVO_PAYLOAD_16_LEN = sizeof(servo_payload_16);
    static constexpr uint16_t SERVO_PAYLOAD_32_LEN = sizeof(servo_payload_32);

    // Maximum dt between Simulator samples
    static constexpr float MAX_DT = 0.05;

    AP_HAL::UARTDriver *uart;
    uint8_t serial_port;
    uint32_t baud_rate;
    bool baud_from_frame;
    uint8_t parity;      // 0=None, 1=Even, 2=Odd
    uint8_t stop_bits;   // 1 or 2
    bool hw_flow_control;

    uint32_t frame_counter;
    double last_timestamp_s;
    uint8_t num_servo_channels;

    // RX parser state machine
    enum class RxState : uint8_t {
        WAIT_SYNC1,
        WAIT_SYNC2,
        WAIT_ID,
        WAIT_LEN_LO,
        WAIT_LEN_HI,
        WAIT_PAYLOAD,
    };

    RxState rx_state;
    uint8_t rx_msg_id;
    uint16_t rx_payload_len;
    uint16_t rx_payload_idx;
    uint8_t rx_buf[STATE_BUF_MAX];

    bool state_received;
    uint16_t state_len;

    void init_uart();
    void output_servos(const struct sitl_input &input);
    void poll_incoming();
    void process_state_packet(uint16_t len);
    void send_frame(uint8_t msg_id, const uint8_t *payload, uint16_t len);
    void apply_state_to_fdm();
};

} // namespace SITL

#endif // AP_SIM_SERIALPLANT_ENABLED

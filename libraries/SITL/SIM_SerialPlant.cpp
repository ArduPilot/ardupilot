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
/*
    Simulator connector for serial-based external plant models (HITL).
    Sends servo PWMs, receives sensor state over a HAL UART port.

    Wire format:
    [SYNC1=0xA5][SYNC2=0x5A][MSG_ID][LEN_LO][LEN_HI][PAYLOAD...]

    State packet layout:
      byte 0:    mask byte (bit 0=quaternion, bit 1=rangefinder, bit 2=airspeed, bit 3=windvane)
      bytes 1-8: timestamp (double, 8 bytes)
      bytes 9-20: gyro (float*3, 12 bytes)
      bytes 21-32: accel_body (float*3, 12 bytes)
      bytes 33-56: position (double*3, 24 bytes)
      bytes 57-68: velocity (float*3, 12 bytes)
      bytes 69+:  attitude (float*3, 12) or quaternion (float*4, 16) based on mask bit 0
      then if mask bit 1: rangefinder (float*6, 24 bytes)
      then if mask bit 2: airspeed (float, 4 bytes)
      then if mask bit 3: windvane (float*2, 8 bytes)
*/

#include "SIM_config.h"

#if AP_SIM_SERIALPLANT_ENABLED

#include "SIM_SerialPlant.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

/*
    Frame string format: "serial:<port>:<baud>"
    Examples: "serial:5:921600", "serial:2:230400"
    Defaults: port=5, baud=921600
*/
SerialPlant::SerialPlant(const char *frame_str) :
    Aircraft(frame_str),
    uart(nullptr),
    serial_port(5),
    baud_rate(921600),
    baud_from_frame(false),
    parity(0),
    stop_bits(1),
    hw_flow_control(false),
    frame_counter(0),
    last_timestamp_s(0),
    num_servo_channels(16),
    rx_state(RxState::WAIT_SYNC1),
    state_received(false),
    state_len(0)
{
    printf("Starting SITL: SerialPlant\n");

    // parse "serial:<port>:<baud>:<parity>:<stopbits>:<hwflow>" from frame string
    const char *colon = strchr(frame_str, ':');
    if (colon) {
        serial_port = (uint8_t)atoi(colon + 1);
        const char *colon2 = strchr(colon + 1, ':');
        if (colon2) {
            baud_rate = (uint32_t)strtoul(colon2 + 1, nullptr, 10);
            baud_from_frame = true;
            const char *colon3 = strchr(colon2 + 1, ':');
            if (colon3) {
                // parity: None=0, Even=1, Odd=2
                if (strncasecmp(colon3 + 1, "Even", 4) == 0) {
                    parity = 1;
                } else if (strncasecmp(colon3 + 1, "Odd", 3) == 0) {
                    parity = 2;
                }
                const char *colon4 = strchr(colon3 + 1, ':');
                if (colon4) {
                    stop_bits = (uint8_t)atoi(colon4 + 1);
                    const char *colon5 = strchr(colon4 + 1, ':');
                    if (colon5) {
                        hw_flow_control = (atoi(colon5 + 1) != 0);
                    }
                }
            }
        }
    }

    printf("SerialPlant: port=SERIAL%u baud=%u parity=%u stopbits=%u hwflow=%u\n",
           serial_port, (unsigned)baud_rate, parity, stop_bits, (unsigned)hw_flow_control);

    memset(rx_buf, 0, sizeof(rx_buf));

    // on ChibiOS the scheduler is paced by the real HW timer, not the
    // simulation clock; disable sync_frame_time so it does not stall
    use_time_sync = false;
}

void SerialPlant::init_uart()
{
    uart = hal.serial(serial_port);
    if (uart == nullptr) {
        printf("SerialPlant: SERIAL%u not available!\n", serial_port);
        return;
    }

    const auto *sm = AP_SerialManager::get_singleton();
    if (sm != nullptr) {
        const auto *st = sm->get_state_by_id(serial_port);
        if (st == nullptr ||
            st->get_protocol() != AP_SerialManager::SerialProtocol_SerialPlant) {
            printf("SerialPlant: SERIAL%u_PROTOCOL is not set to %d (SerialPlant)\n",
                   serial_port, (int)AP_SerialManager::SerialProtocol_SerialPlant);
            uart = nullptr;
            return;
        }
        if (!baud_from_frame) {
            baud_rate = st->baudrate();
            printf("SerialPlant: baud rate from SERIAL%u_BAUD = %u\n",
                   serial_port, (unsigned)baud_rate);
        }
    }

    if (serial_port != 0) {
        uart->set_unbuffered_writes(true);
        uart->begin(baud_rate, 2048, 2048);
    } else {
        uart->begin(baud_rate);
    }
    uart->configure_parity(parity);
    uart->set_stop_bits(stop_bits);
    if (hw_flow_control) {
        uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE);
    }
    uart->flush();
    printf("SerialPlant: SERIAL%u initialized at %u baud parity=%u stopbits=%u hwflow=%u\n",
           serial_port, (unsigned)baud_rate, parity, stop_bits, (unsigned)hw_flow_control);
}

void SerialPlant::send_frame(uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    if (uart == nullptr) {
        return;
    }

    uint8_t header[FRAME_OVERHEAD];
    header[0] = SYNC1;
    header[1] = SYNC2;
    header[2] = msg_id;
    header[3] = (uint8_t)(len & 0xFF);
    header[4] = (uint8_t)(len >> 8);

    uart->write(header, sizeof(header));
    uart->write(payload, len);
}

void SerialPlant::output_servos(const struct sitl_input &input)
{
    num_servo_channels = SRV_Channels::have_32_channels() ? 32 : 16;

    uint8_t buf[SERVO_PAYLOAD_32_LEN];
    uint16_t offset = 0;

    memcpy(buf + offset, &frame_counter, 4);
    offset += 4;

    const uint16_t hz = (uint16_t)rate_hz;
    memcpy(buf + offset, &hz, 2);
    offset += 2;

    for (uint8_t i = 0; i < num_servo_channels; i++) {
        const uint16_t pwm = input.servos[i];
        memcpy(buf + offset, &pwm, 2);
        offset += 2;
    }

    send_frame(MSG_SERVO, buf, offset);
}

void SerialPlant::poll_incoming()
{
    if (uart == nullptr) {
        return;
    }

    uint32_t available = uart->available();
    while (available-- > 0) {
        uint8_t b;
        if (uart->read(&b, 1) != 1) {
            break;
        }

        switch (rx_state) {
        case RxState::WAIT_SYNC1:
            if (b == SYNC1) {
                rx_state = RxState::WAIT_SYNC2;
            }
            break;

        case RxState::WAIT_SYNC2:
            if (b == SYNC2) {
                rx_state = RxState::WAIT_ID;
            } else {
                rx_state = RxState::WAIT_SYNC1;
            }
            break;

        case RxState::WAIT_ID:
            rx_msg_id = b;
            rx_state = RxState::WAIT_LEN_LO;
            break;

        case RxState::WAIT_LEN_LO:
            rx_payload_len = b;
            rx_state = RxState::WAIT_LEN_HI;
            break;

        case RxState::WAIT_LEN_HI:
            rx_payload_len |= ((uint16_t)b) << 8;
            if (rx_payload_len < MANDATORY_LEN || rx_payload_len > STATE_BUF_MAX) {
                rx_state = RxState::WAIT_SYNC1;
            } else {
                rx_payload_idx = 0;
                rx_state = RxState::WAIT_PAYLOAD;
            }
            break;

        case RxState::WAIT_PAYLOAD:
            rx_buf[rx_payload_idx++] = b;
            if (rx_payload_idx >= rx_payload_len) {
                if (rx_msg_id == MSG_STATE) {
                    process_state_packet(rx_payload_len);
                }
                rx_state = RxState::WAIT_SYNC1;
            }
            break;
        }
    }
}

void SerialPlant::process_state_packet(uint16_t len)
{
    state_len = len;
    state_received = true;
}

void SerialPlant::apply_state_to_fdm()
{
    if (!state_received) {
        return;
    }

    const uint8_t *buf = rx_buf;
    const uint16_t len = state_len;

    const uint8_t mask = buf[0];

    double timestamp_s;
    memcpy(&timestamp_s, buf + 1, 8);

    float gyro_f[3], accel_f[3], vel_f[3];
    double pos_d[3];
    memcpy(gyro_f,  buf + 9,  12);
    memcpy(accel_f, buf + 21, 12);
    memcpy(pos_d,   buf + 33, 24);
    memcpy(vel_f,   buf + 57, 12);

    accel_body = Vector3f(accel_f[0], accel_f[1], accel_f[2]);
    gyro = Vector3f(gyro_f[0], gyro_f[1], gyro_f[2]);
    velocity_ef = Vector3f(vel_f[0], vel_f[1], vel_f[2]);
    position = Vector3d(pos_d[0], pos_d[1], pos_d[2]);
    position.xy() += origin.get_distance_NE_double(home);

    uint16_t cursor = 69; // past mask(1) + mandatory(68)
    if (mask & MASK_USE_QUATERNION) {
        if (cursor + 16 > len) { return; }
        float q[4];
        memcpy(q, buf + cursor, 16);
        Quaternion quat(q[0], q[1], q[2], q[3]);
        quat.rotation_matrix(dcm);
        cursor += 16;
    } else {
        if (cursor + 12 > len) { return; }
        float att[3];
        memcpy(att, buf + cursor, 12);
        dcm.from_euler(att[0], att[1], att[2]);
        cursor += 12;
    }

    if (mask & MASK_RANGEFINDER) {
        if (cursor + 24 > len) { return; }
        float rangefinder_f[6];
        memcpy(rangefinder_f, buf + cursor, 24);
        for (uint8_t i = 0; i < 6; i++) {
            rangefinder_m[i] = rangefinder_f[i];
        }
        cursor += 24;
    }

    if (mask & MASK_AIRSPEED) {
        if (cursor + 4 > len) { return; }
        float airspeed_f;
        memcpy(&airspeed_f, buf + cursor, 4);
        airspeed = airspeed_f;
        airspeed_pitot = airspeed_f;
        cursor += 4;
    } else {
        velocity_air_ef = velocity_ef;
        velocity_air_bf = dcm.transposed() * velocity_air_ef;
        update_eas_airspeed();
    }

    if (mask & MASK_WINDVANE) {
        if (cursor + 8 > len) { return; }
        float windvane_f[2];
        memcpy(windvane_f, buf + cursor, 8);
        wind_vane_apparent.direction = windvane_f[0];
        wind_vane_apparent.speed = windvane_f[1];
        cursor += 8;
    }

    update_position();

    if (timestamp_s < last_timestamp_s) {
        printf("SerialPlant: old timestamp received\n");
        last_timestamp_s = timestamp_s;
        return;
    }

    const double dt = timestamp_s - last_timestamp_s;
    last_timestamp_s = timestamp_s;
    time_now_us += dt * 1.0e6;

    if (!is_positive(dt) || dt >= MAX_DT) {
        return;
    }
    adjust_frame_time(1.0 / dt);
    time_advance();
}

void SerialPlant::update(const struct sitl_input &input)
{
    if (uart == nullptr) {
        init_uart();
        if (uart == nullptr) {
            return;
        }
    }

    output_servos(input);
    frame_counter++;

    poll_incoming();
    apply_state_to_fdm();
    update_mag_field_bf();
}

#endif // AP_SIM_SERIALPLANT_ENABLED

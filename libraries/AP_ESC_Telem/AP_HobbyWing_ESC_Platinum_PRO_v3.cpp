/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC.

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.

  See https://github.com/dgatf/msrc#9-annex

 */
#include "AP_HobbyWing_ESC_Platinum_PRO_v3.h"

#if AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/AP_HAL.h>

#include <stdio.h>  // removeme

void AP_HobbyWing_Platinum_PRO_v3::update()
{
    struct PACKED Packet {
        uint8_t header; // 0x9B
        uint8_t seqno_high;  // 24-bit "package number"
        uint8_t seqno_mid;
        uint8_t seqno_low;
        uint16_t throttle_req_pwm;
        uint16_t throttle_out_pwm;
        uint16_t rpm;
    } packet;

    if (!read_uart_v345((uint8_t*)&packet, sizeof(packet), 10000)) {
        return;
    }

    // check for valid frame header
    if (packet.header != 0x9b) {
        // bad header byte after a frame gap
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Bad header");
        return;
    }

    // extract packet sequence number and update count of dropped frames
    check_seq(packet.seqno_high << 16 |
              packet.seqno_mid << 8 |
              packet.seqno_low);

    // decoded.throttle_req = be16toh(packet.throttle_req_pwm);
    // decoded.throttle = be16toh(packet.throttle_req_pwm);

    const float rpm = rpm_from_commutation_time(be16toh(packet.rpm));

    WITH_SEMAPHORE(decoded_sem);
    decoded.rpm = rpm;
    decoded_received_us = AP_HAL::micros();
}

float AP_HobbyWing_Platinum_PRO_v3::rpm_from_commutation_time(uint32_t commutation_time_us) const
{
    if (commutation_time_us == 0) {
        return 0;
    }

    // convert from a commutation interval (in microseconds) to commutations/minute
    float rpm = 60000000.0 / commutation_time_us;
    rpm /= motor_poles * 0.5;  // convert from commutations/minute to RPM
    return rpm;
}

#endif  // AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED


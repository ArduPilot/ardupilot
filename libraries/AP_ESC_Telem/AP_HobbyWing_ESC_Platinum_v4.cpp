/*
  ESC Telemetry for Hobbywing Platinum v4 ESC

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.

  See https://github.com/dgatf/msrc#9-annex

 */
#include "AP_HobbyWing_ESC_Platinum_v4.h"

#if AP_HOBBYWING_PLATINUM_V4_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/AP_HAL.h>

static float get_voltage(uint16_t voltage_raw);
static float get_temperature(uint16_t temperature_raw);
static float get_current(uint16_t current_raw);

void AP_HobbyWing_Platinum_v4::update()
{
    struct PACKED Packet {
        uint8_t header; // 0x9B
        uint8_t seqno_high;  // 24-bit "package number"
        uint8_t seqno_mid;
        uint8_t seqno_low;
        uint16_t throttle_req_pwm;
        uint16_t throttle_out_pwm;
        uint8_t rpm_high;
        uint8_t rpm_mid;
        uint8_t rpm_low;
        uint16_t voltage;
        uint16_t current;
        uint16_t tempfet;
        uint16_t temp;
    } packet;

    if (!read_uart_v345((uint8_t*)&packet, sizeof(packet), 20000)) {
        return;
    }

    // check for valid frame header
    if (packet.header != 0x9b) {
        // bad header byte after a frame gap
        // gcs().send_text(MAV_SEVERITY_INFO, "Bad header");
        return;
    }

    // extract packet sequence number and update count of dropped frames
    check_seq(packet.seqno_high << 16 |
              packet.seqno_mid << 8 |
              packet.seqno_low);

    // decoded.throttle_req = be16toh(packet.throttle_req_pwm);
    // decoded.throttle = be16toh(packet.throttle_req_pwm);

    const float rpm = (packet.rpm_high << 16 | packet.rpm_mid << 8 | packet.rpm_low) / motor_poles * 2;

    WITH_SEMAPHORE(decoded_sem);
    decoded.rpm = rpm;

    decoded.temperature = get_temperature(be16toh(packet.temp));
    decoded.current = get_current(be16toh(packet.current));
    decoded.voltage = get_voltage(be16toh(packet.voltage));

    decoded_received_us = AP_HAL::micros();
}

// the following code swiped from
// https://github.com/dgatf/msrc/blob/master/board/project/esc_hw4.c#L107
// - many thanks to dgatf for this work.

#define ESC_HW4_NTC_BETA 3950.0
#define ESC_HW4_NTC_R1 10000.0
#define ESC_HW4_NTC_R_REF 47000.0
#define ESC_HW4_DIFFAMP_SHUNT (0.25 / 1000)
#define ESC_HW4_V_REF 3.3
#define ESC_HW4_ADC_RES 4096.0

static float get_voltage(uint16_t voltage_raw)
{
    const float divisor = 11.1;  // by experimentation

    return ESC_HW4_V_REF * voltage_raw / ESC_HW4_ADC_RES * divisor;
}

static float get_temperature(uint16_t temperature_raw)
{
    float voltage = temperature_raw * ESC_HW4_V_REF / ESC_HW4_ADC_RES;
    float ntcR_Rref = (voltage * ESC_HW4_NTC_R1 / (ESC_HW4_V_REF - voltage)) / ESC_HW4_NTC_R_REF;
    if (ntcR_Rref < 0.001)
        return 0;
    float temperature = 1 / (log(ntcR_Rref) / ESC_HW4_NTC_BETA + 1 / 298.15) - 273.15;

    return temperature;
}

static float get_current(uint16_t current_raw)
{
    const float ampgain = 1;
    const float current_offset = 0;
    float current = current_raw * ESC_HW4_V_REF / (ampgain * ESC_HW4_DIFFAMP_SHUNT * ESC_HW4_ADC_RES) - current_offset;
    if (current < 0)
        return 0;
    return current;
}

#endif  // AP_HOBBYWING_PLATINUM_PRO_V4_ENABLED

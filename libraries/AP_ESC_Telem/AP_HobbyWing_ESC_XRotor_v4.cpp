/*
  ESC Telemetry for Hobbywing XRotor v4 ESC

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.

  See https://github.com/dgatf/msrc#9-annex

 */
#include "AP_HobbyWing_ESC_XRotor_v4.h"

#if AP_HOBBYWING_PLATINUM_V4_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

void AP_HobbyWing_XRotor_v4::update()
{
    struct PACKED {
        uint8_t header; // 0x9B
        uint8_t pkt_len; // 0x16
        uint32_t counter;
        uint16_t throttle_req;
        uint16_t throttle;
        uint16_t rpm;
        uint16_t voltage;
        int16_t current;
        int16_t phase_current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
        uint16_t crc;

        uint16_t calc_checksum() const {
            uint16_t ret = 0;
            for (uint8_t i=0; i<sizeof(*this)-2; i++) {
                ret += ((uint8_t*)this)[i];
            }
            return ret;
        };

    } packet;

    if (!read_uart_v345((uint8_t*)&packet, sizeof(packet), 10000)) {
        return;
    }

    // check for valid frame header - and valid packet length
    if (packet.header != 0x9b || packet.pkt_len != 0x16) {
        // bad header byte after a frame gap
        // gcs().send_text(MAV_SEVERITY_INFO, "Bad header");
        return;
    }

    if (packet.calc_checksum() != packet.crc) {
        // checksum failure
        return;
    }

    // extract packet sequence number and update count of dropped frames
    check_seq(packet.counter);

    WITH_SEMAPHORE(decoded_sem);

    // decoded.counter = be32toh(pkt.counter);
    // decoded.throttle_req = be16toh(pkt.throttle_req);
    // decoded.throttle = be16toh(pkt.throttle);
    decoded.rpm = packet.rpm / motor_poles;
    decoded.voltage = be16toh(packet.voltage) * 0.1;
    decoded.phase_current = int16_t(be16toh(packet.phase_current)) * 0.01;
    decoded.current = int16_t(be16toh(packet.current)) * 0.01;
    decoded.temperature = MAX(temperature_decode(packet.mos_temperature),
                              temperature_decode(packet.cap_temperature));
    const uint16_t status = be16toh(packet.status);
    if (status != 0) {
        decoded.error_count++;
    }

    decoded_received_us = AP_HAL::micros();
}

static const struct {
    uint8_t adc_temp;
    uint8_t temp_C;
} temp_table[] = {
    { 241, 	0}, 	{ 240, 	1}, 	{ 239, 	2}, 	{ 238, 	3}, 	{ 237, 	4}, 	{ 236, 	5}, 	{ 235, 	6}, 	{ 234, 	7}, 	{ 233, 	8}, 	{ 232, 	9},
    { 231, 	10}, 	{ 230, 	11}, 	{ 229, 	12}, 	{ 228, 	13}, 	{ 227, 	14}, 	{ 226, 	15}, 	{ 224, 	16}, 	{ 223, 	17}, 	{ 222, 	18}, 	{ 220, 	19},
    { 219, 	20}, 	{ 217, 	21}, 	{ 216, 	22}, 	{ 214, 	23}, 	{ 213, 	24}, 	{ 211, 	25}, 	{ 209, 	26}, 	{ 208, 	27}, 	{ 206, 	28}, 	{ 204, 	29},
    { 202, 	30}, 	{ 201, 	31}, 	{ 199, 	32}, 	{ 197, 	33}, 	{ 195, 	34}, 	{ 193, 	35}, 	{ 191, 	36}, 	{ 189, 	37}, 	{ 187, 	38}, 	{ 185, 	39},
    { 183, 	40}, 	{ 181, 	41}, 	{ 179, 	42}, 	{ 177, 	43}, 	{ 174, 	44}, 	{ 172, 	45}, 	{ 170, 	46}, 	{ 168, 	47}, 	{ 166, 	48}, 	{ 164, 	49},
    { 161, 	50}, 	{ 159, 	51}, 	{ 157, 	52}, 	{ 154, 	53}, 	{ 152, 	54}, 	{ 150, 	55}, 	{ 148, 	56}, 	{ 146, 	57}, 	{ 143, 	58}, 	{ 141, 	59},
    { 139, 	60}, 	{ 136, 	61}, 	{ 134, 	62}, 	{ 132, 	63}, 	{ 130, 	64}, 	{ 128, 	65}, 	{ 125, 	66}, 	{ 123, 	67}, 	{ 121, 	68}, 	{ 119, 	69},
    { 117, 	70}, 	{ 115, 	71}, 	{ 113, 	72}, 	{ 111, 	73}, 	{ 109, 	74}, 	{ 106, 	75}, 	{ 105, 	76}, 	{ 103, 	77}, 	{ 101, 	78}, 	{ 99, 	79},
    { 97, 	80}, 	{ 95, 	81}, 	{ 93, 	82}, 	{ 91, 	83}, 	{ 90, 	84}, 	{ 88, 	85}, 	{ 85, 	86}, 	{ 84, 	87}, 	{ 82, 	88}, 	{ 81, 	89},
    { 79, 	90}, 	{ 77, 	91}, 	{ 76, 	92}, 	{ 74, 	93}, 	{ 73, 	94}, 	{ 72, 	95}, 	{ 69, 	96}, 	{ 68, 	97}, 	{ 66, 	98}, 	{ 65, 	99},
    { 64, 	100}, 	{ 62, 	101}, 	{ 62, 	102}, 	{ 61, 	103}, 	{ 59, 	104}, 	{ 58, 	105}, 	{ 56, 	106}, 	{ 54, 	107}, 	{ 54, 	108}, 	{ 53, 	109},
    { 51, 	110}, 	{ 51, 	111}, 	{ 50, 	112}, 	{ 48, 	113}, 	{ 48, 	114}, 	{ 46, 	115}, 	{ 46, 	116}, 	{ 44, 	117}, 	{ 43, 	118}, 	{ 43, 	119},
    { 41, 	120}, 	{ 41, 	121}, 	{ 39, 	122}, 	{ 39, 	123}, 	{ 39, 	124}, 	{ 37, 	125}, 	{ 37, 	126}, 	{ 35, 	127}, 	{ 35, 	128}, 	{ 33, 	129},
};

uint8_t AP_HobbyWing_XRotor_v4::temperature_decode(uint8_t temp_raw) const
{
    for (uint8_t i=0; i<ARRAY_SIZE(temp_table); i++) {
        if (temp_table[i].adc_temp <= temp_raw) {
            return temp_table[i].temp_C;
        }
    }
    return 130U;
}

#endif  // AP_HOBBYWING_PLATINUM_PRO_V4_ENABLED


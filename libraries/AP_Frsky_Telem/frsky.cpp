#include <AP_HAL/AP_HAL.h>

#include "frsky.h"

/*
   State machine to process incoming Frsky SPort bytes
*/
bool FRSky::parse_sport_telemetry_data(uint8_t data, sport_parse_state_t &parse_state)
{
    switch (parse_state.state) {
        case ParseState::START:
            if (data == FRAME_HEAD) {
                parse_state.state = ParseState::IN_FRAME ;
                parse_state.telemetry_rx_buffer_count = 0;
            } else {
                if (parse_state.telemetry_rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
                    parse_state.telemetry_rx_buffer[parse_state.telemetry_rx_buffer_count++] = data;
                }
                parse_state.state = ParseState::IN_FRAME;
            }
            break;

        case ParseState::IN_FRAME:
            if (data == FRAME_DLE) {
                parse_state.state = ParseState::XOR; // XOR next byte
            } else if (data == FRAME_HEAD) {
                parse_state.state = ParseState::IN_FRAME ;
                parse_state.telemetry_rx_buffer_count = 0;
                break;
            } else if (parse_state.telemetry_rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
                parse_state.telemetry_rx_buffer[parse_state.telemetry_rx_buffer_count++] = data;
            }
            break;

        case ParseState::XOR:
            if (parse_state.telemetry_rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
                parse_state.telemetry_rx_buffer[parse_state.telemetry_rx_buffer_count++] = data ^ STUFF_MASK;
            }
            parse_state.state = ParseState::IN_FRAME;
            break;

        case ParseState::IDLE:
            if (data == FRAME_HEAD) {
                parse_state.telemetry_rx_buffer_count = 0;
                parse_state.state = ParseState::START;
            }
            break;

    } // switch

    if (parse_state.telemetry_rx_buffer_count >= SPORT_PACKET_SIZE) {
        parse_state.telemetry_rx_buffer_count = 0;
        parse_state.state = ParseState::IDLE;
        return true;
    }
    return false;
}

/*
 * check sport packet against its crc
 * skip first byte (START_STOP)
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool FRSky::should_process_sport_packet(const uint8_t *packet)
{
    return should_process_sport_packet(packet, nullptr);
}

bool FRSky::should_process_sport_packet(const uint8_t *packet, const uint8_t *previous_packet)
{
#ifdef SPORT_RX_PACKET_DISCARD_DUPLICATE
    //discard duplicates
    if (previous_packet != nullptr) {
        // if they match -> discard this packet
        if (memcmp(&packet[1], &previous_packet[1], SPORT_PACKET_SIZE-1) == 0) {
            return false;
        } else { // if not -> update previous packet
            memcpy(previous_packet, packet, SPORT_PACKET_SIZE);
        }
    }
#endif
    //check CRC
    int16_t crc = 0;
    for (uint8_t i=1; i<SPORT_PACKET_SIZE; ++i) {
        crc += packet[i]; // 0-1FE
        crc += crc >> 8;  // 0-1FF
        crc &= 0x00ff;    // 0-FF
    }
    return (crc == 0x00ff);
}

/*
 * Calculates the sensor id from the physical sensor index [0-27]
        0x00, 	// Physical ID 0 - Vario2 (altimeter high precision)
        0xA1, 	// Physical ID 1 - FLVSS Lipo sensor
        0x22, 	// Physical ID 2 - FAS-40S current sensor
        0x83, 	// Physical ID 3 - GPS / altimeter (normal precision)
        0xE4, 	// Physical ID 4 - RPM
        0x45, 	// Physical ID 5 - SP2UART(Host)
        0xC6, 	// Physical ID 6 - SPUART(Remote)
        0x67, 	// Physical ID 7 - Ardupilot/Betaflight EXTRA DOWNLINK
        0x48, 	// Physical ID 8 -
        0xE9, 	// Physical ID 9 -
        0x6A, 	// Physical ID 10 -
        0xCB, 	// Physical ID 11 -
        0xAC, 	// Physical ID 12 -
        0x0D, 	// Physical ID 13 - Ardupilot/Betaflight UPLINK
        0x8E, 	// Physical ID 14 -
        0x2F, 	// Physical ID 15 -
        0xD0, 	// Physical ID 16 -
        0x71, 	// Physical ID 17 -
        0xF2, 	// Physical ID 18 -
        0x53, 	// Physical ID 19 -
        0x34, 	// Physical ID 20 - Ardupilot/Betaflight EXTRA DOWNLINK
        0x95, 	// Physical ID 21 -
        0x16, 	// Physical ID 22 - GAS Suite
        0xB7, 	// Physical ID 23 - IMU ACC (x,y,z)
        0x98, 	// Physical ID 24 -
        0x39, 	// Physical ID 25 - Power Box
        0xBA 	// Physical ID 26 - Temp
        0x1B	// Physical ID 27 - ArduPilot/Betaflight DEFAULT DOWNLINK
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
#define BIT(x, index) (((x) >> index) & 0x01)
uint8_t FRSky::get_sport_sensor_id(uint8_t physical_id)
{
    uint8_t result = physical_id;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 1) ^ BIT(physical_id, 2)) << 5;
    result += (BIT(physical_id, 2) ^ BIT(physical_id, 3) ^ BIT(physical_id, 4)) << 6;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 2) ^ BIT(physical_id, 4)) << 7;
    return result;
}

/*
  send 1 byte and do byte stuffing
*/
void FRSky::send_byte(AP_HAL::UARTDriver *_port, uint8_t byte)
{
    if (byte == START_STOP_D) {
        _port->write(0x5D);
        _port->write(0x3E);
    } else if (byte == BYTESTUFF_D) {
        _port->write(0x5D);
        _port->write(0x3D);
    } else {
        _port->write(byte);
    }
}

/*
 * send one uint16 frame of FrSky data - for FrSky D protocol (D-receivers)
 */
void  FRSky::send_uint16(AP_HAL::UARTDriver *_port, uint16_t id, uint16_t data)
{
    _port->write(START_STOP_D);    // send a 0x5E start byte
    uint8_t *bytes = (uint8_t*)&id;
    send_byte(_port, bytes[0]);
    bytes = (uint8_t*)&data;
    send_byte(_port, bytes[0]); // LSB
    send_byte(_port, bytes[1]); // MSB
}

/*
 * send an 8 bytes SPort frame of FrSky data - for FrSky SPort protocol (X-receivers)
 */
void  FRSky::send_sport_frame(AP_HAL::UARTDriver *port, uint8_t frame, uint16_t appid, uint32_t data)
{
    uint8_t buf[8];                 // packet buffer
    uint8_t buf2[sizeof(buf)*2+1];  // byte stuffing buffer

    if (port->txspace() < sizeof(buf2)) {
        return;
    }

    buf[0] = frame;
    buf[1] = appid & 0xFF;
    buf[2] = appid >> 8;
    memcpy(&buf[3], &data, 4);

    uint16_t sum = 0;
    for (uint8_t i=0; i<sizeof(buf)-1; i++) {
        sum += buf[i];
        sum += sum >> 8;
        sum &= 0xFF;
    }
    sum = 0xff - ((sum & 0xff) + (sum >> 8));
    buf[7] = (uint8_t)sum;

    // perform byte stuffing per SPort spec
    uint8_t len = 0;

    for (uint8_t i=0; i<sizeof(buf); i++) {
        uint8_t c = buf[i];
        if (c == FRAME_DLE || buf[i] == FRAME_HEAD) {
            buf2[len++] = FRAME_DLE;
            buf2[len++] = c ^ FRAME_XOR;
        } else {
            buf2[len++] = c;
        }
    }
#ifndef HAL_BOARD_SITL
    /*
      check that we haven't been too slow in responding to the new
      UART data. If we respond too late then we will overwrite the next
      polling frame.
      SPort poll-to-poll period is 11.65ms, a frame takes 1.38ms
      but specs require we release the bus before 8ms leaving us with 6500us
     */
    const uint64_t tend_us = port->receive_time_constraint_us(1);
    const uint64_t now_us = AP_HAL::micros64();
    const uint64_t tdelay_us = now_us - tend_us;
    if (tdelay_us > 6500) {
        // we've been too slow in responding
        return;
    }
#endif
    port->write(buf2, len);
}


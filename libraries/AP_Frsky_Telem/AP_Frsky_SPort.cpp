#include "AP_Frsky_SPort.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_Frsky_SPortParser.h"

#include <string.h>

/*
 * send telemetry data
 * for FrSky SPort protocol (X-receivers)
 */
void AP_Frsky_SPort::send(void)
{
    const uint16_t numc = MIN(_port->available(), 1024U);

    // this is the constant for hub data frame
    if (_port->txspace() < 19) {
        return;
    }

    if (numc == 0) {
        // no serial data to process do bg tasks
        if (_SPort.vario_refresh) {
            calc_nav_alt(); // nav altitude is not recalculated until all of it has been sent
            _SPort.vario_refresh = false;
        }
        if (_SPort.gps_refresh) {
            calc_gps_position(); // gps data is not recalculated until all of it has been sent
            _SPort.gps_refresh = false;
        }
        return;
    }

    for (int16_t i = 0; i < numc; i++) {
        int16_t readbyte = _port->read();
        if (_SPort.sport_status == false) {
            if  (readbyte == FRAME_HEAD) {
                _SPort.sport_status = true;
            }
        } else {
            const AP_BattMonitor &_battery = AP::battery();
            switch (readbyte) {
            case SENSOR_ID_VARIO:   // Sensor ID  0
                switch (_SPort.vario_call) {
                case 0:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_BARO_ALT_BP, _SPort_data.alt_nav_meters); // send altitude integer part
                    break;
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_BARO_ALT_AP, _SPort_data.alt_nav_cm); // send altitude decimal part
                    break;
                case 2:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_VARIO, _SPort_data.vario_vspd); // send vspeed m/s
                    _SPort.vario_refresh = true;
                    break;
                }
                if (++_SPort.vario_call > 2) {
                    _SPort.vario_call = 0;
                }
                break;
            case SENSOR_ID_FAS: // Sensor ID  2
                switch (_SPort.fas_call) {
                case 0:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_FUEL, (uint16_t)roundf(_battery.capacity_remaining_pct())); // send battery remaining
                    break;
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_VFAS, (uint16_t)roundf(_battery.voltage() * 10.0f)); // send battery voltage
                    break;
                case 2: {
                    float current;
                    if (!_battery.current_amps(current)) {
                        current = 0;
                    }
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_CURRENT, (uint16_t)roundf(current * 10.0f)); // send current consumption
                    break;
                }
                break;
                }
                if (++_SPort.fas_call > 2) {
                    _SPort.fas_call = 0;
                }
                break;
            case SENSOR_ID_GPS: // Sensor ID  3
                switch (_SPort.gps_call) {
                case 0:
                    send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(_passthrough.send_latitude)); // gps latitude or longitude
                    break;
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, calc_gps_latlng(_passthrough.send_latitude)); // gps latitude or longitude
                    break;
                case 2:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_SPEED_BP, _SPort_data.speed_in_meter); // send gps speed integer part
                    break;
                case 3:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_SPEED_AP, _SPort_data.speed_in_centimeter); // send gps speed decimal part
                    break;
                case 4:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_ALT_BP, _SPort_data.alt_gps_meters); // send gps altitude integer part
                    break;
                case 5:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_ALT_AP, _SPort_data.alt_gps_cm); // send gps altitude decimals
                    break;
                case 6:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_GPS_COURS_BP, _SPort_data.yaw); // send heading in degree based on AHRS and not GPS
                    _SPort.gps_refresh = true;
                    break;
                }
                if (++_SPort.gps_call > 6) {
                    _SPort.gps_call = 0;
                }
                break;
            case SENSOR_ID_SP2UR: // Sensor ID  6
                switch (_SPort.various_call) {
                case 0 :
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_TEMP2, (uint16_t)(AP::gps().num_sats() * 10 + AP::gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
                    break;
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, DATA_ID_TEMP1, gcs().custom_mode()); // send flight mode
                    break;
                }
                if (++_SPort.various_call > 1) {
                    _SPort.various_call = 0;
                }
                break;
            }
            _SPort.sport_status = false;
        }
    }
}

/*
 * prepare gps latitude/longitude data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort::calc_gps_latlng(bool &send_latitude)
{
    const Location &loc = AP::gps().location(0); // use the first gps instance (same as in send_mavlink_gps_raw)

    // alternate between latitude and longitude
    if (send_latitude == true) {
        send_latitude = false;
        if (loc.lat < 0) {
            return ((labs(loc.lat)/100)*6) | 0x40000000;
        } else {
            return ((labs(loc.lat)/100)*6);
        }
    } else {
        send_latitude = true;
        if (loc.lng < 0) {
            return ((labs(loc.lng)/100)*6) | 0xC0000000;
        } else {
            return ((labs(loc.lng)/100)*6) | 0x80000000;
        }
    }
}

/*
 * send an 8 bytes SPort frame of FrSky data - for FrSky SPort protocol (X-receivers)
 */
void  AP_Frsky_SPort::send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data)
{
    uint8_t buf[8];

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
    uint8_t buf2[sizeof(buf)*2+1];

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
    _port->write(buf2, len);
}

extern const AP_HAL::HAL& hal;

bool AP_Frsky_SPortParser::should_process_packet(const uint8_t *packet, bool discard_duplicates)
{
    // check for duplicate packets
    if (discard_duplicates && _parse_state.last_packet != nullptr) {
        /*
          Note: the polling byte packet[0] should be ignored in the comparison
          because we might get the same packet with different polling bytes
          We have 2 types of duplicate packets: ghost identical packets sent by the receiver
          and user duplicate packets sent to compensate for bad link and frame loss, this
          check should address both types.
        */
        if (memcmp(&packet[1], &_parse_state.last_packet[1], SPORT_PACKET_SIZE-1) == 0) {
            return false;
        }
        memcpy(_parse_state.last_packet, packet, SPORT_PACKET_SIZE);
    }
    //check CRC
    int16_t crc = 0;
    for (uint8_t i=1; i<SPORT_PACKET_SIZE; ++i) {
        crc += _parse_state.rx_buffer[i]; // 0-1FE
        crc += crc >> 8;  // 0-1FF
        crc &= 0x00ff;    // 0-FF
    }
    return (crc == 0x00ff);
}

bool AP_Frsky_SPortParser::process_byte(AP_Frsky_SPort::sport_packet_t &sport_packet, const uint8_t data)
{
    switch (_parse_state.state) {
    case ParseState::START:
        if (_parse_state.rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
            _parse_state.rx_buffer[_parse_state.rx_buffer_count++] = data;
        }
        _parse_state.state = ParseState::IN_FRAME;
        break;

    case ParseState::IN_FRAME:
        if (data == FRAME_DLE) {
            _parse_state.state = ParseState::XOR; // XOR next byte
        } else if (data == FRAME_HEAD) {
            _parse_state.state = ParseState::IN_FRAME ;
            _parse_state.rx_buffer_count = 0;
            break;
        } else if (_parse_state.rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
            _parse_state.rx_buffer[_parse_state.rx_buffer_count++] = data;
        }
        break;

    case ParseState::XOR:
        if (_parse_state.rx_buffer_count < TELEMETRY_RX_BUFFER_SIZE) {
            _parse_state.rx_buffer[_parse_state.rx_buffer_count++] = data ^ STUFF_MASK;
        }
        _parse_state.state = ParseState::IN_FRAME;
        break;

    case ParseState::IDLE:
        if (data == FRAME_HEAD) {
            _parse_state.rx_buffer_count = 0;
            _parse_state.state = ParseState::START;
        }
        break;

    } // switch

    if (_parse_state.rx_buffer_count >= SPORT_PACKET_SIZE) {
        _parse_state.rx_buffer_count = 0;
        _parse_state.state = ParseState::IDLE;
        // feed the packet only if it's not a duplicate
        return get_packet(sport_packet, true);
    }
    return false;
}

bool AP_Frsky_SPortParser::get_packet(AP_Frsky_SPort::sport_packet_t &sport_packet, bool discard_duplicates)
{
    if (!should_process_packet(_parse_state.rx_buffer, discard_duplicates)) {
        return false;
    }

    const AP_Frsky_SPort::sport_packet_t sp {
        _parse_state.rx_buffer[0],
        _parse_state.rx_buffer[1],
        le16toh_ptr(&_parse_state.rx_buffer[2]),
        le32toh_ptr(&_parse_state.rx_buffer[4])
    };

    sport_packet = sp;
    return true;
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
uint8_t AP_Frsky_SPort::calc_sensor_id(const uint8_t physical_id)
{
    uint8_t result = physical_id;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 1) ^ BIT(physical_id, 2)) << 5;
    result += (BIT(physical_id, 2) ^ BIT(physical_id, 3) ^ BIT(physical_id, 4)) << 6;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 2) ^ BIT(physical_id, 4)) << 7;
    return result;
}

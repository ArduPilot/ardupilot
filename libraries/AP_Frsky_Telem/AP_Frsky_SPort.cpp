#include "AP_Frsky_SPort.h"

#if AP_FRSKY_SPORT_TELEM_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RPM/AP_RPM.h>

#include "AP_Frsky_SPortParser.h"

#include <string.h>

extern const AP_HAL::HAL& hal;

AP_Frsky_SPort *AP_Frsky_SPort::singleton;

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
        uint8_t readbyte;
        if (!_port->read(readbyte)) {
            break;
        }
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
                    send_sport_frame(SPORT_DATA_FRAME, ALT_ID, _SPort_data.alt_nav_meters*100 + _SPort_data.alt_nav_cm); // send altitude in cm
                    break;
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, VARIO_ID, _SPort_data.vario_vspd); // send vspeed cm/s
                    _SPort.vario_refresh = true;
                    break;
                }
                if (++_SPort.vario_call > 1) {
                    _SPort.vario_call = 0;
                }
                break;
            case SENSOR_ID_FAS: // Sensor ID  2
                switch (_SPort.fas_call) {
                case 0:
                    {
                        uint8_t percentage = 0;
                        IGNORE_RETURN(_battery.capacity_remaining_pct(percentage));
                        send_sport_frame(SPORT_DATA_FRAME, FUEL_ID, (uint16_t)roundf(percentage)); // send battery remaining
                        break;
                    }
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, VFAS_ID, (uint16_t)roundf(_battery.voltage() * 100.0f)); // send battery voltage in cV
                    break;
                case 2: {
                    float current;
                    if (!_battery.current_amps(current)) {
                        current = 0;
                    }
                    send_sport_frame(SPORT_DATA_FRAME, CURR_ID, (uint16_t)roundf(current * 10.0f)); // send current consumption in dA
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
                    send_sport_frame(SPORT_DATA_FRAME, GPS_SPEED_ID, _SPort_data.speed_in_meter*1000 + _SPort_data.speed_in_centimeter*10); // send gps speed in mm/sec
                    break;
                case 3:
                    send_sport_frame(SPORT_DATA_FRAME, GPS_ALT_ID, _SPort_data.alt_gps_meters*100+_SPort_data.alt_gps_cm); // send gps altitude in cm
                    break;
                case 4:
                    send_sport_frame(SPORT_DATA_FRAME, GPS_COURS_ID, _SPort_data.yaw*100); // send heading in cd based on AHRS and not GPS
                    _SPort.gps_refresh = true;
                    break;
                }
                if (++_SPort.gps_call > 4) {
                    _SPort.gps_call = 0;
                }
                break;
            case SENSOR_ID_RPM: // Sensor ID 4
#if AP_RPM_ENABLED
                {
                    const AP_RPM* rpm = AP::rpm();
                    if (rpm == nullptr) {
                        break;
                    }
                    int32_t value;
                    if (calc_rpm(_SPort.rpm_call, value)) {
                        // use high numbered frsky sensor ids to leave low numbered free for externally attached physical frsky sensors
                        uint16_t id = RPM1_ID;
                        if (_SPort.rpm_call != 0) {
                            // only two sensors are currently supported
                            id = RPM2_ID;
                        }
                        send_sport_frame(SPORT_DATA_FRAME, id, value);
                    }
                    if (++_SPort.rpm_call > MIN(rpm->num_sensors()-1, 1)) {
                        _SPort.rpm_call = 0;
                    }
                }
#endif  // AP_RPM_ENABLED
                break;
            case SENSOR_ID_SP2UR: // Sensor ID  6
                switch (_SPort.various_call) {
                case 0 :
                    send_sport_frame(SPORT_DATA_FRAME, TEMP2_ID, (uint16_t)(AP::gps().num_sats() * 10 + AP::gps().status())); // send GPS status and number of satellites as num_sats*10 + status (to fit into a uint8_t)
                    break;
                case 1:
                    send_sport_frame(SPORT_DATA_FRAME, TEMP1_ID, gcs().custom_mode()); // send flight mode
                    break;
                }
                if (++_SPort.various_call > 1) {
                    _SPort.various_call = 0;
                }
                break;
            default:
                {
                    // respond to custom user data polling
                    WITH_SEMAPHORE(_sport_push_buffer.sem);
                    if (_sport_push_buffer.pending && readbyte == _sport_push_buffer.packet.sensor) {
                        send_sport_frame(_sport_push_buffer.packet.frame, _sport_push_buffer.packet.appid, _sport_push_buffer.packet.data);
                        _sport_push_buffer.pending = false;
                    }
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
    if (discard_duplicates) {
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
        { _parse_state.rx_buffer[0],
        _parse_state.rx_buffer[1],
        le16toh_ptr(&_parse_state.rx_buffer[2]),
        le32toh_ptr(&_parse_state.rx_buffer[4]) },
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
#undef BIT
#define BIT(x, index) (((x) >> index) & 0x01)
uint8_t AP_Frsky_SPort::calc_sensor_id(const uint8_t physical_id)
{
    uint8_t result = physical_id;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 1) ^ BIT(physical_id, 2)) << 5;
    result += (BIT(physical_id, 2) ^ BIT(physical_id, 3) ^ BIT(physical_id, 4)) << 6;
    result += (BIT(physical_id, 0) ^ BIT(physical_id, 2) ^ BIT(physical_id, 4)) << 7;
    return result;
}

/*
 * prepare value for transmission through FrSky link
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint16_t AP_Frsky_SPort::prep_number(int32_t number, uint8_t digits, uint8_t power)
{
    uint16_t res = 0;
    uint32_t abs_number = abs(number);

    if ((digits == 2) && (power == 0)) { // number encoded on 7 bits, client side needs to know if expected range is 0,127 or -63,63
        uint8_t max_value = number < 0 ? (0x1<<6)-1 : (0x1<<7)-1;
        res = constrain_int16(abs_number,0,max_value);
        if (number < 0) {   // if number is negative, add sign bit in front
            res |= 1U<<6;
        }
    } else if ((digits == 2) && (power == 1)) { // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
        if (abs_number < 100) {
            res = abs_number<<1;
        } else if (abs_number < 1270) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x7F x 10^1 = 1270)
            res = 0xFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<8;
        }
    } else if ((digits == 2) && (power == 2)) { // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
        if (abs_number < 100) {
            res = abs_number<<2;
        } else if (abs_number < 1000) {
            res = ((uint8_t)roundf(abs_number * 0.1f)<<2)|0x1;
        } else if (abs_number < 10000) {
            res = ((uint8_t)roundf(abs_number * 0.01f)<<2)|0x2;
        } else if (abs_number < 127000) {
            res = ((uint8_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x7F x 10^3 = 127000)
            res = 0x1FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<9;
        }
    } else if ((digits == 3) && (power == 1)) { // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<1;
        } else if (abs_number < 10240) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<1)|0x1;
        } else { // transmit max possible value (0x3FF x 10^1 = 10230)
            res = 0x7FF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<11;
        }
    } else if ((digits == 3) && (power == 2)) { // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
        if (abs_number < 1000) {
            res = abs_number<<2;
        } else if (abs_number < 10000) {
            res = ((uint16_t)roundf(abs_number * 0.1f)<<2)|0x1;
        } else if (abs_number < 100000) {
            res = ((uint16_t)roundf(abs_number * 0.01f)<<2)|0x2;
        } else if (abs_number < 1024000) {
            res = ((uint16_t)roundf(abs_number * 0.001f)<<2)|0x3;
        } else { // transmit max possible value (0x3FF x 10^3 = 1023000)
            res = 0xFFF;
        }
        if (number < 0) { // if number is negative, add sign bit in front
            res |= 0x1<<12;
        }
    }
    return res;
}

/*
 * Push user data down the telemetry link by responding to sensor polling (sport)
 * or by using dedicated slots in the scheduler (fport)
 * for SPort and FPort protocols (X-receivers)
 */
bool AP_Frsky_SPort::sport_telemetry_push(uint8_t sensor, uint8_t frame, uint16_t appid, int32_t data)
{
    WITH_SEMAPHORE(_sport_push_buffer.sem);
    if (_sport_push_buffer.pending) {
        return false;
    }
    _sport_push_buffer.packet.sensor = sensor;
    _sport_push_buffer.packet.frame = frame;
    _sport_push_buffer.packet.appid = appid;
    _sport_push_buffer.packet.data = static_cast<uint32_t>(data);
    _sport_push_buffer.pending = true;
    return true;
}

namespace AP {
    AP_Frsky_SPort *frsky_sport() {
        return AP_Frsky_SPort::get_singleton();
    }
};

#endif  // AP_FRSKY_SPORT_TELEM_ENABLED

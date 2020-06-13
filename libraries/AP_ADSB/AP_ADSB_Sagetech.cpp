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

#include <AP_HAL/AP_HAL.h>
#include "AP_ADSB_Sagetech.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_RTC/AP_RTC.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define SAGETECH_ENFORCE_ACKS               0
#define SAGETECH_DEBUG_ACK_TIMEOUTS         1
#define SAGETECH_DEBUG_TX                   1
#define SAGETECH_DEBUG_RX                   1
#define SAGETECH_DEBUG_RX_CRC_FAIL          1

// constructor
AP_ADSB_Sagetech::AP_ADSB_Sagetech(AP_ADSB &adsb) :
        AP_ADSB_Backend(adsb)
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Sagetech, 0);
    if (uart != nullptr) {
        baudrate = AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Sagetech, 0);
        uart->begin(baudrate);

        // no sagtech hardware have flow control pins exposed
        uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
}

void AP_ADSB_Sagetech::init()
{
    if (uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sInit failed, check SERIALx_PROTOCOL cfg", _GcsHeader);
//    } else {
//        gcs().send_text(MAV_SEVERITY_DEBUG, "%sInit", _GcsHeader);
    }
}

void AP_ADSB_Sagetech::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    // -----------------------------
    // read any available data on serial port
    // -----------------------------
    int32_t nbytes = 0;
    if (uart != nullptr) {
        nbytes = uart->available();
    }
    while (nbytes-- > 0) {
        const uint8_t data = (uint8_t)uart->read();

        switch (protocol) {
        default:
        case AP_ADSB_Sagetech::Protocol::NONE:
            // parse all protocols until we find one that works
            if (parse_byte_XP(data)) {
                protocol = AP_ADSB_Sagetech::Protocol::XP;
                parse_packet_XP(message_in_xp.packet);
            }
            if (parse_byte_MX(data)) {
                protocol = AP_ADSB_Sagetech::Protocol::MX;
                parse_packet_MX();
            }
            break;

        case AP_ADSB_Sagetech::Protocol::XP:
            if (parse_byte_XP(data)) {
                parse_packet_XP(message_in_xp.packet);
            }
            break;

        case AP_ADSB_Sagetech::Protocol::MX:
            if (parse_byte_MX(data)) {
                parse_packet_MX();
            }
            break;
        }
    }


    // -----------------------------
    // handle timers for generating data
    // -----------------------------
    if (last_packet_type_sent != MsgTypes_XP::INVALID) {
        // we're expecting an ACK
        if (now_ms - last_packet_send_ms >= 1000) {
            response_timeout_count++;
#if SAGETECH_DEBUG_ACK_TIMEOUTS
            gcs().send_text(MAV_SEVERITY_DEBUG, "%sACK Timeout type=%d, cnt=%d", _GcsHeader, last_packet_type_sent, (unsigned)response_timeout_count);
#endif
#if SAGETECH_ENFORCE_ACKS
            // retry forever until we get an ACK
            send_packet(last_packet_type_sent);
#endif
        }
    } else {


        if (!has_sent_initialize) {
            has_sent_initialize = true;
            send_packet(MsgTypes_XP::Installation_Set);

        } else if (now_ms - frontend.out_state.last_config_ms >= 10000) {
            frontend.out_state.last_config_ms = now_ms;
            send_packet(MsgTypes_XP::Preflight_Set);

//        } else if (now_ms - last_packet_GPS_ms >= 200 && !frontend._my_loc.is_zero()) {
//            last_packet_GPS_ms = now_ms;
//            // 5Hz GPS update
//            send_GPS();
        }


        //send_Operating();
        //send_Request();
    }
}

void AP_ADSB_Sagetech::send_packet(const MsgTypes_XP type)
{
    switch (type) {
    case MsgTypes_XP::Installation_Set:
        send_Installation();
        break;
    case MsgTypes_XP::Preflight_Set:
        send_PreFlight();
        break;
    case MsgTypes_XP::Operating_Set:
        send_Operating();
        break;
    case MsgTypes_XP::GPS_Set:
        send_GPS();
        break;
    default:
        break;
    }
}

void AP_ADSB_Sagetech::request_packet(const MsgTypes_XP type)
{
    Packet_XP pkt {};

    pkt.type = MsgTypes_XP::Request;
    pkt.id = 0;
    pkt.payload_length = 4;

    pkt.payload[0] = type;

    send_msg(pkt);

}


void AP_ADSB_Sagetech::parse_packet_XP(const Packet_XP &msg)
{
#if SAGETECH_DEBUG_RX
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sRX type=%d, id=%d", _GcsHeader, msg.type, msg.id);
#endif

    switch (msg.type) {
    case MsgTypes_XP::ACK: {
        // ACK received!
        const uint16_t squawk = (uint16_t)msg.payload[7] << 8 | (uint16_t)msg.payload[8];
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sACK type=%u, squawk=%u", _GcsHeader, msg.payload[0], squawk);
        if (last_packet_type_sent == msg.type) {
            last_packet_type_sent = MsgTypes_XP::INVALID;
        } else {
            gcs().send_text(MAV_SEVERITY_DEBUG, "%sACK - did not expect type=%u", _GcsHeader, msg.payload[0]);
        }
        }
        break;

    case MsgTypes_XP::Installatioin_Response:
    case MsgTypes_XP::Preflight_Response:
    case MsgTypes_XP::Status_Response:
    case MsgTypes_XP::ADSB_StateVector_Report:
    case MsgTypes_XP::ADSB_ModeStatus_Report:
    case MsgTypes_XP::TISB_StateVector_Report:
    case MsgTypes_XP::TISB_ModeStatus_Report:
    case MsgTypes_XP::TISB_CorasePos_Report:
    case MsgTypes_XP::TISB_ADSB_Mgr_Report:
        // TODO
        break;

    case MsgTypes_XP::Installation_Set:
    case MsgTypes_XP::Preflight_Set:
    case MsgTypes_XP::Operating_Set:
    case MsgTypes_XP::GPS_Set:
    case MsgTypes_XP::Request:
        // these are out-bound only

    default:
        last_packet_type_sent = INVALID;
        break;

    }
}


bool AP_ADSB_Sagetech::parse_byte_XP(const uint8_t data)
{
    switch (message_in_xp.state) {
        default:
        case ParseState::WaitingFor_Start:
            if (data == 0xA5) {
                message_in_xp.state = ParseState::WaitingFor_AssmAddr;
            }
            break;
        case ParseState::WaitingFor_AssmAddr:
            message_in_xp.state = (data == 0x01) ? ParseState::WaitingFor_MsgType : ParseState::WaitingFor_Start;
            break;
        case ParseState::WaitingFor_MsgType:
            message_in_xp.packet.type = (MsgTypes_XP)data;
            message_in_xp.state = ParseState::WaitingFor_MsgId;
            break;
        case ParseState::WaitingFor_MsgId:
            message_in_xp.packet.id = data;
            message_in_xp.state = ParseState::WaitingFor_PayloadLen;
            break;
        case ParseState::WaitingFor_PayloadLen:
            message_in_xp.packet.payload_length = data;
            message_in_xp.index = 0;
            message_in_xp.state = (data == 0) ? ParseState::WaitingFor_ChecksumFletcher : ParseState::WaitingFor_PayloadContents;
            break;

        case ParseState::WaitingFor_PayloadContents:
            message_in_xp.packet.payload[message_in_xp.index++] = data;
            if (message_in_xp.index >= message_in_xp.packet.payload_length) {
                message_in_xp.state = ParseState::WaitingFor_ChecksumFletcher;
                message_in_xp.index = 0;
            }
            break;

        case ParseState::WaitingFor_ChecksumFletcher:
            message_in_xp.packet.checksumFletcher = data;
            message_in_xp.state = ParseState::WaitingFor_Checksum;
            break;
        case ParseState::WaitingFor_Checksum:
            message_in_xp.packet.checksum = data;
            message_in_xp.state = ParseState::WaitingFor_End;
            if (checksum_XP(message_in_xp.packet)) {
                parse_packet_XP(message_in_xp.packet);
#if SAGETECH_DEBUG_RX_CRC_FAIL
            } else {
                const uint16_t checksum = ((uint16_t)message_in_xp.packet.checksumFletcher << 8) | (uint16_t)message_in_xp.packet.checksum;
                gcs().send_text(MAV_SEVERITY_DEBUG, "%sRX CRC bad type=%d, chksum=0x%04x", _GcsHeader, message_in_xp.packet.type, checksum);
#endif
            }
            break;

        case ParseState::WaitingFor_End:
            // we don't care if the end value matches
            message_in_xp.state = ParseState::WaitingFor_Start;
            break;
    }
    return false;
}

// compute Sum and FletcherSum and write them into msg.
// returns true if the values in msg were already the correct ones
//
// this allows a single function to tell you if inbound msg is correct wirh true result, and ignore result for outbound
bool AP_ADSB_Sagetech::checksum_XP(Packet_XP &msg)
{
    uint8_t sum = 0;
    uint8_t sumFletcher = 0;

    const uint8_t header_message_format[5] = {
            0xA5,   // start
            0x01,   // assembly address
            msg.type,
            msg.id,
            msg.payload_length
    };

    for (uint8_t i=0; i<5; i++) {
        sum += header_message_format[i];
        sumFletcher += sum;
    }
    for (uint8_t i=0; i<msg.payload_length; i++) {
        sum += msg.payload[i];
        sumFletcher += sum;
    }

    // the calculated sums already match the values in the msg
    if ((sum == msg.checksum) && (sumFletcher == msg.checksumFletcher)) {
        return true;
    }

    msg.checksum = sum;
    msg.checksumFletcher = sumFletcher;
    return false;
}


void AP_ADSB_Sagetech::send_msg(Packet_XP &msg)
{
    // generate and populate checksums.
    checksum_XP(msg);

    if (uart != nullptr) {
        uart->write(msg.start);
        uart->write(msg.assemAddr);
        uart->write(msg.type);
        uart->write(msg.id);
        uart->write(msg.payload_length);
        uart->write(msg.payload, msg.payload_length);
        uart->write(msg.checksumFletcher);
        uart->write(msg.checksum);
        uart->write(msg.end);
    }

    last_packet_type_sent = msg.type;
    last_packet_send_ms = AP_HAL::millis();

#if SAGETECH_DEBUG_TX
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sTx type=%d", _GcsHeader, msg.type);
#endif
}





void AP_ADSB_Sagetech::send_Installation()
{
    uint8_t adsb_in_vehicles_per_second = 0;
    uint8_t baud_value = 0;

    switch (baudrate) {
    case 2400:
        baud_value = 1;
        adsb_in_vehicles_per_second = 0;
        break;
    case 4800:
        baud_value = 2;
        adsb_in_vehicles_per_second = 0;
        break;
    case 9600:
        baud_value = 3;
        adsb_in_vehicles_per_second = 5;
        break;
    case 19200:
        baud_value = 4;
        adsb_in_vehicles_per_second = 10;
        break;
    case 38400:
        baud_value = 5;
        adsb_in_vehicles_per_second = 20;
        break;
    default:
    case 57600:
        baud_value = 0;
        adsb_in_vehicles_per_second = 30;
        break;
    case 115000:
    case 115200:
        baud_value = 6;
        adsb_in_vehicles_per_second = 60;
        break;
    case 230000:
    case 230400:
        baud_value = 7;
        adsb_in_vehicles_per_second = 100;
        break;
    case 460000:
    case 460800:
        baud_value = 8;
        adsb_in_vehicles_per_second = 100;
        break;
    }

    Packet_XP pkt {};

    pkt.type = MsgTypes_XP::Installation_Set;
    pkt.payload_length = 28;

//    // mode C version
//    pkt.id = 3;

    // mode S version
    pkt.id = 0;

    pkt.payload[0] = frontend.out_state.cfg.ICAO_id >> 16;
    pkt.payload[1] = frontend.out_state.cfg.ICAO_id >> 8;
    pkt.payload[2] = frontend.out_state.cfg.ICAO_id >> 0;

    memcpy(&frontend.out_state.cfg.callsign, &pkt.payload[3], 8);

    pkt.payload[11] = 0;   // airspeed MAX

    pkt.payload[12] = baud_value;   // COM Port 0 baud
    pkt.payload[13] = 0;            // COM Port 1 baud
    pkt.payload[14] = 0;            // COM Port 2 baud

    pkt.payload[15] = 1;   // GPS from COM port 0 (this port)
    pkt.payload[16] = 0;   // GPS Integrity

    pkt.payload[17] = frontend.out_state.cfg.emitterType / 8;   // Emitter Set
    pkt.payload[18] = frontend.out_state.cfg.emitterType % 8;   // Emitter Type

    pkt.payload[19] = frontend.out_state.cfg.lengthWidth;   // Aircraft Size

    pkt.payload[20] = 0;   // Altitude Encoder Offset
    pkt.payload[21] = 0;   // Altitude Encoder Offset

    pkt.payload[22] = 0x07;   // ADSB In Control, enable reading everything
    pkt.payload[23] = adsb_in_vehicles_per_second; // ADSB In Report max length COM Port 0 (this one)
    pkt.payload[24] = 0;   // ADSB In Report max length COM Port 1

    send_msg(pkt);
}

void AP_ADSB_Sagetech::send_PreFlight()
{
    Packet_XP pkt {};

    pkt.type = MsgTypes_XP::Preflight_Set;
    pkt.id = 0;
    pkt.payload_length = 10;

    memcpy(&frontend.out_state.cfg.callsign, &pkt.payload[0], 8);
    memset(&pkt.payload[8], 0, 2);

    send_msg(pkt);
}

void AP_ADSB_Sagetech::send_Operating()
{
    Packet_XP pkt {};

    pkt.type = MsgTypes_XP::Operating_Set;
    pkt.id = 0;
    pkt.payload_length = 8;

    memcpy(&frontend.out_state.cfg.squawk_octal, &pkt.payload[0], 2);

    const int32_t alt_m = frontend._my_loc.alt * 0.01f;
    int16_t alt_feet = alt_m / FEET_TO_METERS;
    memcpy(&alt_feet, &pkt.payload[2], 2);

    uint8_t mode = 0;
    if (frontend.out_state.cfg.rfSelect & 0x02) {
        // enable TX
        mode |= 1;
    }
    if (frontend.out_state.cfg.rfSelect & 0x04) {
        // enable Ident
        mode |= 2;
    }
    pkt.payload[4] = mode;

    send_msg(pkt);
}

void AP_ADSB_Sagetech::send_GPS()
{
    Packet_XP pkt {};
    float deg;

    pkt.type = MsgTypes_XP::GPS_Set;
    pkt.payload_length = 52;
    pkt.id = 0;

    const int32_t latitude =  58.565247 * 1.0e7f;
    const int32_t longitude = 49.285211 * 1.0e7f;
//    const int32_t latitude =  frontend._my_loc.lat;
//    const int32_t longitude = frontend._my_loc.lng;

    deg = fabsf(longitude * 1.0e-7f);
    snprintf((char*)&pkt.payload[0], 11+1, "%03u.%08.5f", (unsigned)deg, double((deg - int(deg)) * 60));

    deg = fabsf(latitude * 1.0e-7f);
    snprintf((char*)&pkt.payload[11], 10+1, "%02u.%08.5f", (unsigned)deg, double((deg - int(deg)) * 60));



    const Vector2f speed = AP::ahrs().groundspeed_vector();

    const float speed_knots = norm(speed.x, speed.y) * M_PER_SEC_TO_KNOTS;
    snprintf((char*)&pkt.payload[21], 6+1, "%.2f", double(speed_knots));

    const float heading = wrap_360(degrees(atan2f(speed.x, speed.y)));
    snprintf((char*)&pkt.payload[27], 8+1, "%.4f", double(heading));

    // hemisphere
    uint8_t hemisphere = 0;
    hemisphere |= (latitude >= 0) ? 0x01 : 0;   // isNorth
    hemisphere |= (longitude >= 0) ? 0x02 : 0;  // isEast
    hemisphere |= (AP::gps().status() < AP_GPS::GPS_OK_FIX_2D) ? 0x80 : 0;  // isInvalid
    pkt.payload[35] = hemisphere;


    uint64_t time_usec;
    if (!AP::rtc().get_utc_usec(time_usec)) {
        memset(&pkt.payload[36],' ', 10);
    } else {
        // not completely accurate, our time includes leap seconds and time_t should be without
        const time_t time_sec = time_usec / 1000000;
        struct tm* tm = gmtime(&time_sec);

        // format time string
        snprintf((char*)&pkt.payload[36], 10, "%06.3f", tm->tm_sec + (time_usec % 1000000) * 1.0e-6);
    }

    send_msg(pkt);
}







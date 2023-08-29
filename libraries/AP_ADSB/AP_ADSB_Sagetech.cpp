/*
   Copyright (C) 2020  Kraus Hamdani Aerospace Inc. All rights reserved.

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

#include "AP_ADSB_Sagetech.h"

#if HAL_ADSB_SAGETECH_ENABLED
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <math.h>

#define SAGETECH_SCALER_LATLNG              (1.0f/2.145767E-5f)     //   180/(2^23)
#define SAGETECH_SCALER_KNOTS_TO_CMS        ((KNOTS_TO_M_PER_SEC/0.125f) * 100.0f)
#define SAGETECH_SCALER_ALTITUDE            (1.0f/0.015625f)
#define SAGETECH_SCALER_HEADING_CM          ((360.0f/256.0f) * 100.0f)

#define SAGETECH_VALIDFLAG_LATLNG           (1U<<0)
#define SAGETECH_VALIDFLAG_ALTITUDE         (1U<<1)
#define SAGETECH_VALIDFLAG_VELOCITY         (1U<<2)
#define SAGETECH_VALIDFLAG_GND_SPEED        (1U<<3)
#define SAGETECH_VALIDFLAG_HEADING          (1U<<4)
#define SAGETECH_VALIDFLAG_V_RATE_GEO       (1U<<5)
#define SAGETECH_VALIDFLAG_V_RATE_BARO      (1U<<6)
#define SAGETECH_VALIDFLAG_EST_LATLNG       (1U<<7)
#define SAGETECH_VALIDFLAG_EST_VELOCITY     (1U<<8)

// detect if any port is configured as Sagetech
bool AP_ADSB_Sagetech::detect()
{
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_ADSB, 0);
}

// Init, called once after class is constructed
bool AP_ADSB_Sagetech::init()
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0);

    return (_port != nullptr);
}

void AP_ADSB_Sagetech::update()
{
    if (_port == nullptr) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // -----------------------------
    // read any available data on serial port
    // -----------------------------
    uint32_t nbytes = MIN(_port->available(), 10 * PAYLOAD_XP_MAX_SIZE);
    while (nbytes-- > 0) {
        uint8_t data;
        if (!_port->read(data)) {
            break;
        }
        if (parse_byte_XP(data)) {
            handle_packet_XP(message_in.packet);
        }
    } // while nbytes


    // -----------------------------
    // handle timers for generating data
    // -----------------------------
    if (!last_packet_initialize_ms || (now_ms - last_packet_initialize_ms >= 5000)) {
        last_packet_initialize_ms = now_ms;
        send_packet(MsgType_XP::Installation_Set);

    } else if (!last_packet_PreFlight_ms || (now_ms - last_packet_PreFlight_ms >= 8200)) {
        last_packet_PreFlight_ms = now_ms;
        // TODO: allow callsign to not require a reboot
        send_packet(MsgType_XP::Preflight_Set);

    } else if (now_ms - last_packet_Operating_ms >= 1000 && (
            last_packet_Operating_ms == 0 || // send once at boot
            // send as data changes
            last_operating_squawk != _frontend.out_state.cfg.squawk_octal ||
            abs(last_operating_alt - _frontend._my_loc.alt) > 1555 ||      // 1493cm == 49ft. The output resolution is 100ft per bit
            last_operating_rf_select != _frontend.out_state.cfg.rfSelect))
    {
        last_packet_Operating_ms = now_ms;
        last_operating_squawk = _frontend.out_state.cfg.squawk_octal;
        last_operating_alt = _frontend._my_loc.alt;
        last_operating_rf_select = _frontend.out_state.cfg.rfSelect;
        send_packet(MsgType_XP::Operating_Set);

    } else if (now_ms - last_packet_GPS_ms >= (_frontend.out_state.is_flying ? 200 : 1000)) {
        // 1Hz when not flying, 5Hz when flying
        last_packet_GPS_ms = now_ms;
        send_packet(MsgType_XP::GPS_Set);
    }
}

void AP_ADSB_Sagetech::send_packet(const MsgType_XP type)
{
    switch (type) {
    case MsgType_XP::Installation_Set:
        send_msg_Installation();
        break;
    case MsgType_XP::Preflight_Set:
        send_msg_PreFlight();
        break;
    case MsgType_XP::Operating_Set:
        send_msg_Operating();
        break;
    case MsgType_XP::GPS_Set:
        send_msg_GPS();
        break;
    default:
        break;
    }
}

void AP_ADSB_Sagetech::request_packet(const MsgType_XP type)
{
    // set all bytes in packet to 0 via {} so we only need to set the ones we need to
    Packet_XP pkt {};

    pkt.type = MsgType_XP::Request;
    pkt.id = 0;
    pkt.payload_length = 4;

    pkt.payload[0] = static_cast<uint8_t>(type);

    send_msg(pkt);
}


void AP_ADSB_Sagetech::handle_packet_XP(const Packet_XP &msg)
{
    switch (msg.type) {
    case MsgType_XP::ACK:
        handle_ack(msg);
        break;

    case MsgType_XP::Installation_Response:
    case MsgType_XP::Preflight_Response:
    case MsgType_XP::Status_Response:
        // TODO add support for these
        break;

    case MsgType_XP::ADSB_StateVector_Report:
    case MsgType_XP::ADSB_ModeStatus_Report:
    case MsgType_XP::TISB_StateVector_Report:
    case MsgType_XP::TISB_ModeStatus_Report:
    case MsgType_XP::TISB_CorasePos_Report:
    case MsgType_XP::TISB_ADSB_Mgr_Report:
        handle_adsb_in_msg(msg);
        break;

    case MsgType_XP::Installation_Set:
    case MsgType_XP::Preflight_Set:
    case MsgType_XP::Operating_Set:
    case MsgType_XP::GPS_Set:
    case MsgType_XP::Request:
        // these are out-bound only and are not expected to be received
    case MsgType_XP::INVALID:
        break;
    }
}

void AP_ADSB_Sagetech::handle_ack(const Packet_XP &msg)
{
    // ACK received!
    const uint8_t system_state = msg.payload[2];
    transponder_type = (Transponder_Type)msg.payload[6];

    const uint8_t prev_transponder_mode = last_ack_transponder_mode;
    last_ack_transponder_mode = (system_state >> 6) & 0x03;
    if (prev_transponder_mode != last_ack_transponder_mode) {
        static const char *mode_names[] = {"OFF", "STBY", "ON", "ON-ALT"};
        if (last_ack_transponder_mode < ARRAY_SIZE(mode_names)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADSB: RF Mode: %s", mode_names[last_ack_transponder_mode]);
        }
    }
}

void AP_ADSB_Sagetech::handle_adsb_in_msg(const Packet_XP &msg)
{
    AP_ADSB::adsb_vehicle_t vehicle {};

    vehicle.last_update_ms = AP_HAL::millis();

    switch (msg.type) {
    case MsgType_XP::ADSB_StateVector_Report: { // 0x91
        const uint16_t validFlags = le16toh_ptr(&msg.payload[8]);
        vehicle.info.ICAO_address = le24toh_ptr(&msg.payload[10]);

        if (validFlags & SAGETECH_VALIDFLAG_LATLNG) {
            vehicle.info.lat = ((int32_t)le24toh_ptr(&msg.payload[20])) * SAGETECH_SCALER_LATLNG;
            vehicle.info.lon = ((int32_t)le24toh_ptr(&msg.payload[23])) * SAGETECH_SCALER_LATLNG;
            vehicle.info.flags |= ADSB_FLAGS_VALID_COORDS;
        }

        if (validFlags & SAGETECH_VALIDFLAG_ALTITUDE) {
            vehicle.info.altitude = (int32_t)le24toh_ptr(&msg.payload[26]);
            vehicle.info.flags |= ADSB_FLAGS_VALID_ALTITUDE;
        }

        if (validFlags & SAGETECH_VALIDFLAG_VELOCITY) {
            const float velNS = ((int32_t)le16toh_ptr(&msg.payload[29])) * SAGETECH_SCALER_KNOTS_TO_CMS;
            const float velEW = ((int32_t)le16toh_ptr(&msg.payload[31])) * SAGETECH_SCALER_KNOTS_TO_CMS;
            vehicle.info.hor_velocity = Vector2f(velEW, velNS).length();
            vehicle.info.flags |= ADSB_FLAGS_VALID_VELOCITY;
        }

        if (validFlags & SAGETECH_VALIDFLAG_HEADING) {
            vehicle.info.heading = ((float)msg.payload[29]) * SAGETECH_SCALER_HEADING_CM;
            vehicle.info.flags |= ADSB_FLAGS_VALID_HEADING;
        }

        if ((validFlags & SAGETECH_VALIDFLAG_V_RATE_GEO) || (validFlags & SAGETECH_VALIDFLAG_V_RATE_BARO)) {
            vehicle.info.ver_velocity = (int16_t)le16toh_ptr(&msg.payload[38]);
            vehicle.info.flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID;
        }

        _frontend.handle_adsb_vehicle(vehicle);
        break;
    }
    case MsgType_XP::ADSB_ModeStatus_Report:   // 0x92
        vehicle.info.ICAO_address = le24toh_ptr(&msg.payload[9]);

        if (msg.payload[16] != 0) {
            // if string is non-null, consider it valid
            memcpy(&vehicle.info, &msg.payload[16], 8);
            vehicle.info.flags |= ADSB_FLAGS_VALID_CALLSIGN;
        }

        _frontend.handle_adsb_vehicle(vehicle);
        break;
    case MsgType_XP::TISB_StateVector_Report:
    case MsgType_XP::TISB_ModeStatus_Report:
    case MsgType_XP::TISB_CorasePos_Report:
    case MsgType_XP::TISB_ADSB_Mgr_Report:
        // TODO
        return;

    default:
        return;
    }

}

// handling inbound byte and process it in the state machine
// return true when a full packet has been received
bool AP_ADSB_Sagetech::parse_byte_XP(const uint8_t data)
{
    switch (message_in.state) {
        default:
        case ParseState::WaitingFor_Start:
            if (data == 0xA5) {
                message_in.state = ParseState::WaitingFor_AssmAddr;
            }
            break;
        case ParseState::WaitingFor_AssmAddr:
            message_in.state = (data == 0x01) ? ParseState::WaitingFor_MsgType : ParseState::WaitingFor_Start;
            break;
        case ParseState::WaitingFor_MsgType:
            message_in.packet.type = static_cast<MsgType_XP>(data);
            message_in.state = ParseState::WaitingFor_MsgId;
            break;
        case ParseState::WaitingFor_MsgId:
            message_in.packet.id = data;
            message_in.state = ParseState::WaitingFor_PayloadLen;
            break;
        case ParseState::WaitingFor_PayloadLen:
            message_in.packet.payload_length = data;
            message_in.index = 0;
            message_in.state = (data == 0) ? ParseState::WaitingFor_ChecksumFletcher : ParseState::WaitingFor_PayloadContents;
            break;

        case ParseState::WaitingFor_PayloadContents:
            message_in.packet.payload[message_in.index++] = data;
            if (message_in.index >= message_in.packet.payload_length) {
                message_in.state = ParseState::WaitingFor_ChecksumFletcher;
                message_in.index = 0;
            }
            break;

        case ParseState::WaitingFor_ChecksumFletcher:
            message_in.packet.checksumFletcher = data;
            message_in.state = ParseState::WaitingFor_Checksum;
            break;
        case ParseState::WaitingFor_Checksum:
            message_in.packet.checksum = data;
            message_in.state = ParseState::WaitingFor_End;
            if (checksum_verify_XP(message_in.packet)) {
                handle_packet_XP(message_in.packet);
            }
            break;

        case ParseState::WaitingFor_End:
            // we don't care if the end value matches
            message_in.state = ParseState::WaitingFor_Start;
            break;
    }
    return false;
}

// compute Sum and FletcherSum values into a single value
// returns uint16_t with MSByte as Sum and LSByte FletcherSum
uint16_t AP_ADSB_Sagetech::checksum_generate_XP(Packet_XP &msg) const
{
    uint8_t sum = 0;
    uint8_t sumFletcher = 0;

    const uint8_t header_message_format[5] {
            0xA5,   // start
            0x01,   // assembly address
            static_cast<uint8_t>(msg.type),
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

    return UINT16_VALUE(sum, sumFletcher);
}

// computes checksum and returns true if it matches msg checksum
bool AP_ADSB_Sagetech::checksum_verify_XP(Packet_XP &msg) const
{
    const uint16_t checksum = checksum_generate_XP(msg);
    return (HIGHBYTE(checksum) == msg.checksum) && (LOWBYTE(checksum) == msg.checksumFletcher);
}

// computes checksum and assigns checksum values to msg
void AP_ADSB_Sagetech::checksum_assign_XP(Packet_XP &msg)
{
    const uint16_t checksum = checksum_generate_XP(msg);
    msg.checksum = HIGHBYTE(checksum);
    msg.checksumFletcher = LOWBYTE(checksum);
}

// send message to serial port
void AP_ADSB_Sagetech::send_msg(Packet_XP &msg)
{
    // generate and populate checksums.
    checksum_assign_XP(msg);

    const uint8_t message_format_header[5] {
            0xA5,   // start
            0x01,   // assembly address
            static_cast<uint8_t>(msg.type),
            msg.id,
            msg.payload_length
    };
    const uint8_t message_format_tail[3] {
            msg.checksumFletcher,
            msg.checksum,
            0x5A    // end
    };

    if (_port != nullptr) {
        _port->write(message_format_header, sizeof(message_format_header));
        _port->write(msg.payload, msg.payload_length);
        _port->write(message_format_tail, sizeof(message_format_tail));
    }
}

void AP_ADSB_Sagetech::send_msg_Installation()
{
    Packet_XP pkt {};

    pkt.type = MsgType_XP::Installation_Set;
    pkt.payload_length = 28; // 28== 0x1C

    // Mode C = 3, Mode S = 0
    pkt.id = (transponder_type == Transponder_Type::Mode_C) ? 3 : 0;

//    // convert a decimal 123456 to 0x123456
    // TODO: do a proper conversion. The param contains "131313" but what gets transmitted over the air is 0x200F1.
    const uint32_t icao_hex = AP_ADSB::convert_base_to_decimal(16, _frontend.out_state.cfg.ICAO_id_param);
    put_le24_ptr(&pkt.payload[0], icao_hex);

    memcpy(&pkt.payload[3], &_frontend.out_state.cfg.callsign, 8);

    pkt.payload[11] = 0;        // airspeed MAX

    pkt.payload[12] = 0;        // COM Port 0 baud, fixed at 57600
    pkt.payload[13] = 0;        // COM Port 1 baud, fixed at 57600
    pkt.payload[14] = 0;        // COM Port 2 baud, fixed at 57600

    pkt.payload[15] = 1;        // GPS from COM port 0 (this port)
    pkt.payload[16] = 1;        // GPS Integrity

    pkt.payload[17] = _frontend.out_state.cfg.emitterType / 8;      // Emitter Set
    pkt.payload[18] = _frontend.out_state.cfg.emitterType & 0x0F;   // Emitter Type

    pkt.payload[19] = _frontend.out_state.cfg.lengthWidth;          // Aircraft Size

    pkt.payload[20] = 0;        // Altitude Encoder Offset
    pkt.payload[21] = 0;        // Altitude Encoder Offset

    pkt.payload[22] = 0x07;     // ADSB In Control, enable reading everything
    pkt.payload[23] = 30;       // ADSB In Report max length COM Port 0 (this one)
    pkt.payload[24] = 0;        // ADSB In Report max length COM Port 1

    send_msg(pkt);
}

void AP_ADSB_Sagetech::send_msg_PreFlight()
{
    Packet_XP pkt {};

    pkt.type = MsgType_XP::Preflight_Set;
    pkt.id = 0;
    pkt.payload_length = 10;

    memcpy(&pkt.payload[0], &_frontend.out_state.cfg.callsign, 8);

    memset(&pkt.payload[8], 0, 2);

    send_msg(pkt);
}

void AP_ADSB_Sagetech::send_msg_Operating()
{
    Packet_XP pkt {};

    pkt.type = MsgType_XP::Operating_Set;
    pkt.id = 0;
    pkt.payload_length = 8;

    // squawk
    // param is saved as native octal so we need convert back to
    // decimal because Sagetech will convert it back to octal
    const uint16_t squawk = AP_ADSB::convert_base_to_decimal(8, last_operating_squawk);
    put_le16_ptr(&pkt.payload[0], squawk);

    // altitude
    if (_frontend.out_state.cfg.rf_capable & 0x01) {
        const float alt_meters = last_operating_alt * 0.01f;
        const int32_t alt_feet = (int32_t)(alt_meters * FEET_TO_METERS);
        const int16_t alt_feet_adj = (alt_feet + 50) / 100; // 1 = 100 feet, 1 = 149 feet, 5 = 500 feet
        put_le16_ptr(&pkt.payload[2], alt_feet_adj);

    } else {
        // use integrated altitude - recommend by sagetech
        pkt.payload[2] = 0x80;
        pkt.payload[3] = 0x00;
    }

    // RF mode
    pkt.payload[4] = last_operating_rf_select;
    send_msg(pkt);
}

void AP_ADSB_Sagetech::send_msg_GPS()
{
    Packet_XP pkt {};

    pkt.type = MsgType_XP::GPS_Set;
    pkt.payload_length = 52;
    pkt.id = 0;

    const int32_t longitude = _frontend._my_loc.lng;
    const int32_t latitude =  _frontend._my_loc.lat;

    // longitude and latitude
    // NOTE: these MUST be done in double or else we get roundoff in the maths
    const double lon_deg = longitude * (double)1.0e-7 * (longitude < 0 ? -1 : 1);
    const double lon_minutes = (lon_deg - int(lon_deg)) * 60;
    snprintf((char*)&pkt.payload[0], 12, "%03u%02u.%05u", (unsigned)lon_deg, (unsigned)lon_minutes, unsigned((lon_minutes - (int)lon_minutes) * 1.0E5));

    const double lat_deg = latitude * (double)1.0e-7 * (latitude < 0 ? -1 : 1);
    const double lat_minutes = (lat_deg - int(lat_deg)) * 60;
    snprintf((char*)&pkt.payload[11], 11, "%02u%02u.%05u", (unsigned)lat_deg, (unsigned)lat_minutes, unsigned((lat_minutes - (int)lat_minutes) * 1.0E5));

    // ground speed
    const Vector2f speed = AP::ahrs().groundspeed_vector();
    float speed_knots = speed.length() * M_PER_SEC_TO_KNOTS;
    snprintf((char*)&pkt.payload[21], 7, "%03u.%02u", (unsigned)speed_knots, unsigned((speed_knots - (int)speed_knots) * 1.0E2));

    // heading
    float heading = wrap_360(degrees(speed.angle()));
    snprintf((char*)&pkt.payload[27], 10, "%03u.%04u", unsigned(heading), unsigned((heading - (int)heading) * 1.0E4));

    // hemisphere
    uint8_t hemisphere = 0;
    hemisphere |= (latitude >= 0) ? 0x01 : 0;   // isNorth
    hemisphere |= (longitude >= 0) ? 0x02 : 0;  // isEast
    hemisphere |= (AP::gps().status() < AP_GPS::GPS_OK_FIX_2D) ? 0x80 : 0;  // isInvalid
    pkt.payload[35] = hemisphere;

    // time
    uint64_t time_usec;
    if (AP::rtc().get_utc_usec(time_usec)) {
        // not completely accurate, our time includes leap seconds and time_t should be without
        const time_t time_sec = time_usec / 1000000;
        struct tm* tm = gmtime(&time_sec);

        // format time string
        snprintf((char*)&pkt.payload[36], 11, "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + (time_usec % 1000000) * 1.0e-6);
    } else {
        memset(&pkt.payload[36],' ', 10);
    }

    send_msg(pkt);
}

#endif // HAL_ADSB_SAGETECH_ENABLED


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
#include <ctype.h>
#include <time.h>
#include <string.h>
#include <AP_Math/bitwise.h>

extern const AP_HAL::HAL& hal;

#define SAGETECH_HARDCODE_CALLSIGN          "K1000ULE"  // comment out to disable
#define SAGETECH_ENFORCE_ACKS               0
#define SAGETECH_DEBUG_ACK_TIMEOUTS         0
#define SAGETECH_DEBUG_TX_ID_ONLY           0
#define SAGETECH_DEBUG_TX_ID_PAYLOAD        0
#define SAGETECH_DEBUG_TX_Operating         0
#define SAGETECH_DEBUG_TX_GPS               0
#define SAGETECH_DEBUG_TX_ALL_RAW           0
#define SAGETECH_DEBUG_RX                   0
#define SAGETECH_DEBUG_RX_CRC_FAIL          0
#define SAGETECH_DEBUG_RX_ACK               0

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
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (uart != nullptr) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sUART Init successful", _GcsHeader);
    }
#endif

#ifdef SAGETECH_HARDCODE_CALLSIGN
    memcpy(&frontend.out_state.cfg.callsign, SAGETECH_HARDCODE_CALLSIGN, 8);
#endif

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
#if SAGETECH_ENFORCE_ACKS
    if (last_packet_type_sent != MsgTypes_XP::INVALID) {
        // we're expecting an ACK
        if (now_ms - last_packet_send_ms >= 1000) {
            response_timeout_count++;
#if SAGETECH_DEBUG_ACK_TIMEOUTS
            gcs().send_text(MAV_SEVERITY_DEBUG, "%sACK Timeout type=%u, cnt=%u", _GcsHeader, (unsigned)last_packet_type_sent, (unsigned)response_timeout_count);
#endif
            // retry forever until we get an ACK
            send_packet(last_packet_type_sent);
        }
    } else
#endif
    {

        //if (!last_packet_initialize_ms) {
        if (!last_packet_initialize_ms || (now_ms - last_packet_initialize_ms >= 5000)) {
            last_packet_initialize_ms = now_ms;
            send_packet(MsgTypes_XP::Installation_Set);


        //} else if (last_packet_PreFlight_ms == 0) {
        } else if (!last_packet_PreFlight_ms || (now_ms - last_packet_PreFlight_ms >= 8200)) {
            // send once, for now..
            last_packet_PreFlight_ms = now_ms;
            // TODO: allow callsign to not require a reboot
            send_packet(MsgTypes_XP::Preflight_Set);

        } else if (now_ms - last_packet_Operating_ms >= 1000 && (
                last_packet_Operating_ms == 0 || // send once at boot
                // send as data changes
                last_operating_squawk != frontend.out_state.cfg.squawk_octal ||
                abs(last_operating_alt - frontend._my_loc.alt) > 1555 ||      // 1493cm == 49ft. The output resolution is 100ft per bit
                last_operating_rf_select != frontend.out_state.cfg.rfSelect))
        {
            last_packet_Operating_ms = now_ms;
            last_operating_squawk = frontend.out_state.cfg.squawk_octal;
            last_operating_alt = frontend._my_loc.alt;

            frontend.out_state.cfg.rfSelect &= 0x06; // apply mask for valid bits
            last_operating_rf_select = frontend.out_state.cfg.rfSelect;

            send_packet(MsgTypes_XP::Operating_Set);
            if (last_operating_rf_select != frontend.out_state.cfg.rfSelect) {
                frontend.out_state.cfg.rfSelect.set_and_notify(last_operating_rf_select);
                frontend.out_state.cfg.rfSelect.save();
            }

        } else if (now_ms - last_packet_GPS_ms >= (frontend.out_state.is_flying ? 200 : 1000)) {
            // 1Hz when not flying, 5Hz when flying
            last_packet_GPS_ms = now_ms;
            send_packet(MsgTypes_XP::GPS_Set);
        }
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
        const uint8_t system_state = msg.payload[2];
        transponder_type = (Transponder_Type)msg.payload[6];

        const char* rfmode = "RF mode: ";
        const uint8_t prev_transponder_mode = last_ack_transponder_mode;
        last_ack_transponder_mode = (system_state >> 6) & 0x03;
        if (prev_transponder_mode != last_ack_transponder_mode) {
            switch (last_ack_transponder_mode) {
            case 0: gcs().send_text(MAV_SEVERITY_INFO, "%s%sOFF",   _GcsHeader, rfmode); break;
            case 1: gcs().send_text(MAV_SEVERITY_INFO, "%s%sSTBY",  _GcsHeader, rfmode); break;
            case 2: gcs().send_text(MAV_SEVERITY_INFO, "%s%sON",    _GcsHeader, rfmode); break;
            case 3: gcs().send_text(MAV_SEVERITY_INFO, "%s%sON-ALT",_GcsHeader, rfmode); break;
            default:  break;
            }
        }

#if SAGETECH_DEBUG_RX_ACK
        const uint8_t acked_type = msg.payload[0];
        const uint8_t acked_id = msg.payload[1];
        const int32_t pres_altitude = (int32_t)fetchU32(&msg.payload[3]);
        const uint16_t squawk = fetchU16(&msg.payload[7]);

        gcs().send_text(MAV_SEVERITY_DEBUG, "%sACK %u, %u, 0x%02X, %d, %u, %u %u", _GcsHeader,
                acked_type,
                acked_id,
                system_state,
                pres_altitude,
                transponder_type,
                squawk,
                last_ack_transponder_mode); // mode

        for (uint8_t i=0; i<6; i++) {
            const uint8_t stateBits = (system_state & (1<< i));
            if (stateBits != 0 && stateBits != SystemStateBits::Altitidue_Source) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "%sACK status: %s", _GcsHeader, systemStatsBits_to_str((SystemStateBits)stateBits));
            }
        }
        //gcs().send_text(MAV_SEVERITY_DEBUG, "%sACK mode: %u", _GcsHeader, system_state >> 6);

#endif
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

    const uint8_t message_format_header[5] = {
            0xA5,   // start
            0x01,   // assembly address
            msg.type,
            msg.id,
            msg.payload_length
    };
    const uint8_t message_format_tail[3] = {
            msg.checksumFletcher,
            msg.checksum,
            0x5A    // end
    };

    if (uart != nullptr) {
        uart->write(message_format_header, sizeof(message_format_header));
        uart->write(msg.payload, msg.payload_length);
        uart->write(message_format_tail, sizeof(message_format_tail));
    }

    last_packet_type_sent = msg.type;
    last_packet_send_ms = AP_HAL::millis();


#if SAGETECH_DEBUG_TX_ALL_RAW
    gcs().send_text(MAV_SEVERITY_DEBUG, "%s-------", _GcsHeader);
    for (uint8_t i=0; i<sizeof(message_format_header); i++) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "%shead[%2u] = 0x%02x", _GcsHeader, i, message_format_header[i]);
    }
#endif
#if SAGETECH_DEBUG_TX_ID_ONLY
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sTx type=%d %s", _GcsHeader, msg.type, type_to_str(msg.type));
#endif
#if SAGETECH_DEBUG_TX_ID_PAYLOAD || SAGETECH_DEBUG_TX_ALL_RAW
    #if SAGETECH_DEBUG_TX_ID_PAYLOAD
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sTx type=%u, payload.len=%u", _GcsHeader, (unsigned)msg.type, (unsigned)msg.payload_length);
    #endif
    for (uint8_t i=0; i<msg.payload_length; i++) {
        const uint8_t data = msg.payload[i];
        if (isalnum(data) || data == '.' || data == '-' || data == ' ') {
            gcs().send_text(MAV_SEVERITY_DEBUG, "%sdata[%2u] = 0x%02x  %c", _GcsHeader, i, data, (char)data);
        } else {
            gcs().send_text(MAV_SEVERITY_DEBUG, "%sdata[%2u] = 0x%02x", _GcsHeader, i, data);
        }
    }
#endif

#if SAGETECH_DEBUG_TX_ALL_RAW
    for (uint8_t i=0; i<sizeof(message_format_tail); i++) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "%stail[%2u] = 0x%02x", _GcsHeader, (unsigned)(i + sizeof(message_format_header) + msg.payload_length), message_format_tail[i]);
    }
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
    case 38:
    case 38400:
        baud_value = 5;
        adsb_in_vehicles_per_second = 20;
        break;
    default:
    case 57:
    case 57600:
        baud_value = 0;
        adsb_in_vehicles_per_second = 30;
        break;
    case 115:
    case 115000:
    case 115200:
        baud_value = 6;
        adsb_in_vehicles_per_second = 60;
        break;
    case 230:
    case 230000:
    case 230400:
        baud_value = 7;
        adsb_in_vehicles_per_second = 100;
        break;
    case 460:
    case 460000:
    case 460800:
        baud_value = 8;
        adsb_in_vehicles_per_second = 100;
        break;
    }

    Packet_XP pkt {};

    pkt.type = MsgTypes_XP::Installation_Set;
    pkt.payload_length = 28; // 28== 0x1C

    // Mode C = 3, Mode S = 0
    pkt.id = (transponder_type == Transponder_Type::Mode_C) ? 3 : 0;


    // convert a decimal 123456 to 0x123456
    char hex_str[15];
    snprintf(hex_str, sizeof(hex_str), "%X", (unsigned)frontend.out_state.cfg.ICAO_id_param);
    const uint32_t icao_hex = strtol(hex_str, nullptr, 10);

    // TODO: do a proper conversion. The param contains "131313" but what gets transmitted over the air is 0x200F1.
    pkt.payload[0] = (uint32_t(icao_hex) >> 16) & 0xFF;
    pkt.payload[1] = (uint32_t(icao_hex) >> 8) & 0xFF;
    pkt.payload[2] = (uint32_t(icao_hex) >> 0) & 0xFF;


    memcpy(&pkt.payload[3], &frontend.out_state.cfg.callsign, 8);

    pkt.payload[11] = 0;   // airspeed MAX

    pkt.payload[12] = baud_value;   // COM Port 0 baud
    pkt.payload[13] = 0;            // COM Port 1 baud, fixed at 57600
    pkt.payload[14] = 0;            // COM Port 2 baud, fixed at 57600

    pkt.payload[15] = 1;   // GPS from COM port 0 (this port)
    pkt.payload[16] = 1;   // GPS Integrity

    pkt.payload[17] = frontend.out_state.cfg.emitterType / 8;   // Emitter Set
    pkt.payload[18] = frontend.out_state.cfg.emitterType & 0x0F;   // Emitter Type

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

    memcpy(&pkt.payload[0], &frontend.out_state.cfg.callsign, 8);

    memset(&pkt.payload[8], 0, 2);

    send_msg(pkt);
}

void AP_ADSB_Sagetech::send_Operating()
{
    Packet_XP pkt {};

    pkt.type = MsgTypes_XP::Operating_Set;
    pkt.id = 0;
    pkt.payload_length = 8;

    // squawk
    loadUint(&pkt.payload[0], last_operating_squawk, 16);

    // altitude
    if (frontend.out_state.cfg.rf_capable & 0x01) {
        const float alt_meters = last_operating_alt * 0.01f;
        const int32_t alt_feet = (int32_t)(alt_meters * FEET_TO_METERS);
        //int32_t alt_feet = 126700;       // TEST
        const int16_t alt_feet_adj = (alt_feet + 50) / 100; // 1 = 100 feet, 1 = 149 feet, 5 = 500 feet
        loadUint(&pkt.payload[2], alt_feet_adj, 16);

    } else {
        // use integrated altitude - recommend by sagetech
        pkt.payload[2] = 0x80;
        pkt.payload[3] = 0x00;
    }


    // RF mode
    last_operating_rf_select &= 7; // mask param to param bits
    pkt.payload[4] = last_operating_rf_select & 0x06;   // mask to useful sagetech bits

    if (last_operating_rf_select & 0x04) {
        // Ident should only be sent once. Its held on in the hw for 18 seconds
        gcs().send_text(MAV_SEVERITY_DEBUG, "%sIdent!", _GcsHeader);
        last_operating_rf_select &= ~0x04;
    }


#if SAGETECH_DEBUG_TX_Operating
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sOper squawk:%d, %dft, %d", _GcsHeader, fetchU16(&pkt.payload[0]), (int16_t)fetchU16(&pkt.payload[2])*100, pkt.payload[4]);
#endif

    send_msg(pkt);
}

void AP_ADSB_Sagetech::send_GPS()
{
    Packet_XP pkt {};

    pkt.type = MsgTypes_XP::GPS_Set;
    pkt.payload_length = 52;
    pkt.id = 0;

//    const int32_t longitude = -122.3291670 * 1.0e7f;    // Seattle
//    const int32_t latitude =   47.6204000 * 1.0e7f;

//    const int32_t longitude =  58.5652470 * 1.0e7f;
//    const int32_t latitude =   49.2852110 * 1.0e7f;

//    const int32_t longitude =  9 * 1.0e7f;
//    const int32_t latitude =   9 * 1.0e7f;

//    const int32_t longitude =  -9 * 1.0e7f;
//    const int32_t latitude =   -9 * 1.0e7f;

//    const int32_t longitude =   149.1652300 * 1.0e7f;       // CMAC
//    const int32_t latitude =    -35.3632000 * 1.0e7f;

    const int32_t longitude = frontend._my_loc.lng;
    const int32_t latitude =  frontend._my_loc.lat;


    // longitude and latitude
    // NOTE: these MUST be done in double or else we get roundoff in the maths
    const double lon_deg = longitude * 1.0e-7d * (longitude < 0 ? -1 : 1);
    const double lon_minutes = (lon_deg - int(lon_deg)) * 60;
    snprintf((char*)&pkt.payload[0], 12, "%03u%02u.%05u", (unsigned)lon_deg, (unsigned)lon_minutes, unsigned((lon_minutes - (int)lon_minutes) * 1.0E5));

#if SAGETECH_DEBUG_TX_GPS
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sGPS lon: %s", _GcsHeader, &pkt.payload[0]);
#endif


    const double lat_deg = latitude * 1.0e-7d * (latitude < 0 ? -1 : 1);
    const double lat_minutes = (lat_deg - int(lat_deg)) * 60;
    snprintf((char*)&pkt.payload[11], 11, "%02u%02u.%05u", (unsigned)lat_deg, (unsigned)lat_minutes, unsigned((lat_minutes - (int)lat_minutes) * 1.0E5));

#if SAGETECH_DEBUG_TX_GPS
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sGPS lat: %s", _GcsHeader, &pkt.payload[11]);
#endif


    // ground speed
    const Vector2f speed = AP::ahrs().groundspeed_vector();
    float speed_knots = norm(speed.x, speed.y) * M_PER_SEC_TO_KNOTS;
    snprintf((char*)&pkt.payload[21], 7, "%03u.%02u", (unsigned)speed_knots, unsigned((speed_knots - (int)speed_knots) * 1.0E2));


#if SAGETECH_DEBUG_TX_GPS
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sGPS speed: %s", _GcsHeader, &pkt.payload[21]);
#endif

    // heading
    //float heading = wrap_360(degrees(atan2f(speed.x, speed.y)));
    float heading = wrap_360(degrees(speed.angle()));
    snprintf((char*)&pkt.payload[27], 10, "%03u.%04u", unsigned(heading), unsigned((heading - (int)heading) * 1.0E4));

#if SAGETECH_DEBUG_TX_GPS
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sGPS heading: %s", _GcsHeader, &pkt.payload[27]);
#endif

    // hemisphere
    uint8_t hemisphere = 0;
    hemisphere |= (latitude >= 0) ? 0x01 : 0;   // isNorth
    hemisphere |= (longitude >= 0) ? 0x02 : 0;  // isEast
    hemisphere |= (AP::gps().status() < AP_GPS::GPS_OK_FIX_2D) ? 0x80 : 0;  // isInvalid
    pkt.payload[35] = hemisphere;

#if SAGETECH_DEBUG_TX_GPS
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sGPS hemisphere: 0x%02X", _GcsHeader, pkt.payload[35]);
#endif

    // time
    uint64_t time_usec;
    if (!AP::rtc().get_utc_usec(time_usec)) {
        memset(&pkt.payload[36],' ', 10);
    } else {
        // not completely accurate, our time includes leap seconds and time_t should be without
        const time_t time_sec = time_usec / 1000000;
        struct tm* tm = gmtime(&time_sec);

        // format time string
        snprintf((char*)&pkt.payload[36], 11, "%02u%02u%06.3f", tm->tm_hour, tm->tm_min, tm->tm_sec + (time_usec % 1000000) * 1.0e-6);

    }

#if SAGETECH_DEBUG_TX_GPS
    gcs().send_text(MAV_SEVERITY_DEBUG, "%sGPS time: %s", _GcsHeader, &pkt.payload[36]);
#endif

    send_msg(pkt);
}

const char* AP_ADSB_Sagetech::type_to_str(const uint8_t type)
{
    switch (type) {
    case 0x01: return "Install";
    case 0x02: return "PreFlight";
    case 0x03: return "Operate";
    case 0x04: return "GPS Data";
    case 0x05: return "DataReq";
    default: return "Unknown";
    }
}

const char* AP_ADSB_Sagetech::systemStatsBits_to_str(const SystemStateBits systemStateBits)
{
    switch (systemStateBits) {
    case SystemStateBits::Error_Transponder: return "Error_Transponder";
    case SystemStateBits::Altitidue_Source: return "Altitidue_Source";
    case SystemStateBits::Error_GPS: return "Error_GPS";
    case SystemStateBits::Error_ICAO: return "Error_ICAO";
    case SystemStateBits::Error_Over_Temperature: return "Error_Over_Temperature";
    case SystemStateBits::Error_Extended_Squitter: return "Error_Extended_Squitter";
     default: return "Unknown";
    }
}






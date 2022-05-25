#pragma once

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

#include "AP_ADSB_Backend.h"

#ifndef HAL_ADSB_SAGETECH_ENABLED
#define HAL_ADSB_SAGETECH_ENABLED HAL_ADSB_ENABLED
#endif

#if HAL_ADSB_SAGETECH_ENABLED
class AP_ADSB_Sagetech : public AP_ADSB_Backend {
public:
    using AP_ADSB_Backend::AP_ADSB_Backend;

    // init - performs any required initialisation for this instance
    bool init() override;

    // update - should be called periodically
    void update() override;

    // static detection function
    static bool detect();

private:

    static const uint32_t PAYLOAD_XP_MAX_SIZE  = 52;

    enum class SystemStateBits {
        Error_Transponder       = (1U<<0),
        Altitidue_Source        = (1U<<1),
        Error_GPS               = (1U<<2),
        Error_ICAO              = (1U<<3),
        Error_Over_Temperature  = (1U<<4),
        Error_Extended_Squitter = (1U<<5),
        Mode_Transponder        = (3U<<6),   // 2 bit status:
    };

    enum class Transponder_Type {
        Mode_C                  = 0x00,
        Mode_S_ADSB_OUT         = 0x01,
        Mode_S_ADSB_OUT_and_IN  = 0x02,
        Unknown                 = 0xFF,
    };

    enum class MsgType_XP {
        INVALID                 = 0,
        Installation_Set        = 0x01,
        Preflight_Set           = 0x02,
        Operating_Set           = 0x03,
        GPS_Set                 = 0x04,
        Request                 = 0x05,

        ACK                     = 0x80,
        Installation_Response   = 0x81,
        Preflight_Response      = 0x82,
        Status_Response         = 0x83,
        ADSB_StateVector_Report = 0x91,
        ADSB_ModeStatus_Report  = 0x92,
        TISB_StateVector_Report = 0x93,
        TISB_ModeStatus_Report  = 0x94,
        TISB_CorasePos_Report   = 0x95,
        TISB_ADSB_Mgr_Report    = 0x96,
    };

    enum class ParseState {
        WaitingFor_Start,
        WaitingFor_AssmAddr,
        WaitingFor_MsgType,
        WaitingFor_MsgId,
        WaitingFor_PayloadLen,
        WaitingFor_PayloadContents,
        WaitingFor_ChecksumFletcher,
        WaitingFor_Checksum,
        WaitingFor_End,
    };

    struct Packet_XP {
        const uint8_t   start = 0xA5;
        const uint8_t   assemAddr = 0x01;
        MsgType_XP     type;
        uint8_t         id;
        uint8_t         payload_length;
        uint8_t         payload[PAYLOAD_XP_MAX_SIZE];
        uint8_t         checksumFletcher;
        uint8_t         checksum;
        const uint8_t   end = 0x5A;
    };

    struct {
        ParseState      state;
        uint8_t         index;
        Packet_XP       packet;
    } message_in;

    // compute Sum and FletcherSum values
    uint16_t checksum_generate_XP(Packet_XP &msg) const;
    bool checksum_verify_XP(Packet_XP &msg) const;
    void checksum_assign_XP(Packet_XP &msg);


    // handling inbound byte and process it in the state machine
    bool parse_byte_XP(const uint8_t data);

    // handle inbound packet
    void handle_packet_XP(const Packet_XP &msg);

    // send message to serial port
    void send_msg(Packet_XP &msg);

    // handle inbound msgs
    void handle_adsb_in_msg(const Packet_XP &msg);
    void handle_ack(const Packet_XP &msg);

    // send messages to transceiver
    void send_msg_Installation();
    void send_msg_PreFlight();
    void send_msg_Operating();
    void send_msg_GPS();

    // send packet by type
    void send_packet(const MsgType_XP type);

    // send msg to request a packet by type
    void request_packet(const MsgType_XP type);

    // Convert base 8 or 16 to decimal. Used to convert an octal/hexadecimal value
    // stored on a GCS as a string field in different format, but then transmitted
    // over mavlink as a float which is always a decimal.
    uint32_t convert_base_to_decimal(const uint8_t baseIn, uint32_t inputNumber);

    // timers for each out-bound packet
    uint32_t        last_packet_initialize_ms;
    uint32_t        last_packet_PreFlight_ms;
    uint32_t        last_packet_GPS_ms;
    uint32_t        last_packet_Operating_ms;

    // cached variables to compare against params so we can send msg on param change.
    uint16_t        last_operating_squawk;
    int32_t         last_operating_alt;
    uint8_t         last_operating_rf_select;

    // track status changes in acks
    uint8_t         last_ack_transponder_mode;
    Transponder_Type transponder_type = Transponder_Type::Unknown;
};
#endif // HAL_ADSB_SAGETECH_ENABLED


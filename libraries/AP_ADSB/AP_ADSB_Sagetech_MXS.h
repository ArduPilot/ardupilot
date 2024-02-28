/*
 * Copyright (C)  2022 Sagetech Avionics Inc. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * SDK specification
 * https://github.com/Sagetech-Avionics/sagetech-mxs-sdk/blob/main/doc/pdf/ICD02373_MXS_Host_ICD.pdf
 * 
 * Authors: Chuck Faber, Tom Pittenger
*/

#pragma once

#include "AP_ADSB_Backend.h"

#if HAL_ADSB_SAGETECH_MXS_ENABLED

#include "sagetech-sdk/sagetech_mxs.h"

class AP_ADSB_Sagetech_MXS : public AP_ADSB_Backend {
public:
    using AP_ADSB_Backend::AP_ADSB_Backend;

    /**
     * @brief Performs required initialization for this instance
     * 
     * @return true if initialization successful
     */
    bool init() override;

    /**
     * @brief The main callback function (Called with freq of 10Hz) that sends 
     * appropriate message types at specific times.
     * 
     * Read Byte from Serial Port Buffer (10Hz)
     * Send installation message (every 5 seconds)
     * Send Flight ID (every 8.2 s)
     * Send Operating Message (every second)
     * Send GPS data (flying: 5Hz, not flying: 1Hz)
     * 
     */
    void update() override;

    /**
     * @brief Detect if a port is configured as Sagetech
     * 
     * @return true 
     * @return false 
     */
    static bool detect();

private:

    static const uint32_t PAYLOAD_MXS_MAX_SIZE  = 255;
    static const uint8_t  START_BYTE = 0xAA;
    static const uint8_t rf_capable_flags_default =
                    ADSB_BITBASK_RF_CAPABILITIES_1090ES_IN |
                    ADSB_BITBASK_RF_CAPABILITIES_1090ES_OUT;


    enum class MsgType : uint8_t {
        Installation            = SG_MSG_TYPE_HOST_INSTALL,
        FlightID                = SG_MSG_TYPE_HOST_FLIGHT,
        Operating               = SG_MSG_TYPE_HOST_OPMSG,
        GPS_Data                = SG_MSG_TYPE_HOST_GPS,
        Data_Request            = SG_MSG_TYPE_HOST_DATAREQ,
        // RESERVED 0x06 - 0x0A
        Target_Request          = SG_MSG_TYPE_HOST_TARGETREQ,
        Mode                    = SG_MSG_TYPE_HOST_MODE,
        // RESERVED 0x0D - 0xC1
        ACK                     = SG_MSG_TYPE_XPNDR_ACK,
        Installation_Response   = SG_MSG_TYPE_XPNDR_INSTALL,
        FlightID_Response       = SG_MSG_TYPE_XPNDR_FLIGHT,
        Status_Response         = SG_MSG_TYPE_XPNDR_STATUS,
        RESERVED_0x84           = 0x84,
        RESERVED_0x85           = 0x85,
        Mode_Settings           = SG_MSG_TYPE_XPNDR_MODE,
        RESERVED_0x8D           = 0x8D,
        Version_Response        = SG_MSG_TYPE_XPNDR_VERSION,
        Serial_Number_Response  = SG_MSG_TYPE_XPNDR_SERIALNUM,
        Target_Summary_Report   = SG_MSG_TYPE_ADSB_TSUMMARY,

        ADSB_StateVector_Report = SG_MSG_TYPE_ADSB_SVR,
        ADSB_ModeStatus_Report  = SG_MSG_TYPE_ADSB_MSR,
        ADSB_Target_State_Report= SG_MSG_TYPE_ADSB_TSTATE,
        ADSB_Air_Ref_Vel_Report = SG_MSG_TYPE_ADSB_ARVR,
    };

    enum class ParseState {
        WaitingFor_Start,
        WaitingFor_MsgType,
        WaitingFor_MsgId,
        WaitingFor_PayloadLen,
        WaitingFor_PayloadContents,
        WaitingFor_Checksum,
    };

    struct __attribute__((packed)) Packet {
        const uint8_t   start = SG_MSG_START_BYTE;
        MsgType         type;
        uint8_t         id;
        uint8_t         payload_length;
        uint8_t         payload[PAYLOAD_MXS_MAX_SIZE];
    };

    struct {
        ParseState      state;
        uint8_t         index;
        Packet          packet;
        uint8_t         checksum;
    } message_in;

    /**
     * @brief Given the dataReqType, send the appropriate data request message
     * 
     * @param dataReqType 
     */
    void send_data_req(const sg_datatype_t dataReqType);

    /**
     * @brief Takes incoming packets, gets their message type, and 
     * appropriately handles them with the correct callbacks.
     * 
     * @param msg Message packet received, cast into Packet type.
     */
    void handle_packet(const Packet &msg);

    /**
     * @brief Sends data received from ADSB State Vector Report to AutoPilot
     * 
     * @param svr 
     */
    void handle_svr(const sg_svr_t svr);

    /**
     * @brief Handle a received ADSB mode status report and updates the vehicle list
     * 
     * @param msr Sagetech SDK Mode Status Report type
     */
    void handle_msr(const sg_msr_t msr);


    /**
     * @brief Handles an incoming byte and processes it through the state
     * machine to determine if end of message is reached.
     * 
     * @param data : incoming byte
     * @return false : if not yet reached packet termination
     */
    bool parse_byte(const uint8_t data);

    /**
     * @brief Takes a raw buffer and writes it out to the device port.
     * 
     * @param data : pointer to data buffer
     * @param len : number of bytes to write
     */
    void msg_write(const uint8_t *data, const uint16_t len) const;


    /**
     * @brief Callback for sending an installation message.
     * 
     */
    void send_install_msg();

    /**
     * @brief Callback for sending a FlightID message
     * 
     */
    void send_flight_id_msg();

    /**
     * @brief Callback for sending an operating message.
     * 
     */
    void send_operating_msg();

    /**
     * @brief Callback for sending a GPS data message
     * 
     */
    void send_gps_msg();

    /**
     * @brief Callback for sending a Target Request message
     * 
     */
    void send_targetreq_msg();

    /**
     * @brief Convert the AP_ADSB uint8_t Emitter Type to the Sagetech Emitter Type definition
     * 
     * @param emitterType 
     * @return sg_emitter_t 
     */
    sg_emitter_t convert_emitter_type_to_sg(const uint8_t emitterType) const;

    /**
     * @brief Convert the float maxAirSpeed value to the Sagetech Max Airspeed Type
     * 
     * @param maxAirSpeed 
     * @return sg_airspeed_t 
     */
    sg_airspeed_t convert_airspeed_knots_to_sg(const float maxAirSpeed) const;

    /**
     * @brief Converts a Sagetech Emitter type value to the values used by ADSB.
     * 
     * @return uint8_t 
     */
    uint8_t convert_sg_emitter_type_to_adsb(const sg_emitter_t sgEmitterType) const;

    void auto_config_operating();
    void auto_config_installation();
    void auto_config_flightid();
    void handle_ack(const sg_ack_t ack);

    struct {
        // timers for each out-bound packet
        uint32_t        packet_initialize_ms;
        uint32_t        packet_PreFlight_ms;
        uint32_t        packet_GPS_ms;
        uint32_t        packet_Operating_ms;
        uint32_t        packet_targetReq;

        // cached variables to compare against params so we can send msg on param change.
        uint16_t        operating_squawk;
        int32_t         operating_alt;
        uint8_t         operating_rf_select;
        bool            modeAEnabled;
        bool            modeCEnabled;
        bool            modeSEnabled;
        bool            failXpdr;
        bool            failSystem;
        uint8_t         callsign[8];
        struct {
            uint8_t     id;
            uint8_t     type;
        } msg;
    } last;

    struct {
        bool init;
        bool init_failed;
        sg_operating_t op;
        sg_install_t inst;
        sg_targetreq_t treq;
        sg_flightid_t fid;
        sg_ack_t ack;
    } mxs_state;

    // helper functions for populating the operating message:
    void populate_op_altitude(const struct AP_ADSB::Loc &loc);
    void populate_op_climbrate(const struct AP_ADSB::Loc &loc);
    void populate_op_airspeed_and_heading(const struct AP_ADSB::Loc &loc);

};
#endif // HAL_ADSB_SAGETECH_MXS_ENABLED


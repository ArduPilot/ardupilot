/*
   Copyright (C) 2021  Kraus Hamdani Aerospace Inc. All rights reserved.

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

   Author: Tom Pittenger
 */

#pragma once
#include "AP_ADSB_Backend.h"

#if HAL_ADSB_UCP_ENABLED

#define AP_ADSB_UAVIONIX_UCP_CAPTURE_ALL_RX_PACKETS         1

#include "GDL90_protocol/GDL90_Message_Structs.h"
#include "GDL90_protocol/hostGDL90Support.h"

class AP_ADSB_uAvionix_UCP : public AP_ADSB_Backend {
public:
    using AP_ADSB_Backend::AP_ADSB_Backend;

    // init - performs any required initialisation for this instance
    bool init() override;

    // update - should be called periodically
    void update() override;

    // static detection function
    static bool detect();

private:

    void handle_msg(const GDL90_RX_MESSAGE &msg);
    bool request_msg(const GDL90_MESSAGE_ID msg_id);

    void send_GPS_Data();
    void send_Transponder_Control();
    const char* get_hardware_name(const uint8_t hwId);

    bool hostTransmit(uint8_t *buffer, uint16_t length);
    uint16_t gdl90Transmit(GDL90_TX_MESSAGE &message, const uint16_t length);
    static bool parseByte(const uint8_t data, GDL90_RX_MESSAGE &msg, GDL90_RX_STATUS &status);

    struct {
        uint32_t last_msg_ms;
        GDL90_RX_MESSAGE msg;
        GDL90_RX_STATUS status;

        // cache local copies so we always have the latest info of everything.
        struct {
            GDL90_IDENTIFICATION_V3 identification;
            GDL90_TRANSPONDER_CONFIG_MSG_V4_V5 transponder_config;
            GDL90_HEARTBEAT heartbeat;
            GDL90_TRANSPONDER_STATUS_MSG transponder_status;
            GDL90_TRANSPONDER_STATUS_MSG_V3 transponder_status_v3;
#if AP_ADSB_UAVIONIX_UCP_CAPTURE_ALL_RX_PACKETS
            GDL90_OWNSHIP_REPORT ownship_report;
            GDL90_OWNSHIP_GEO_ALTITUDE ownship_geometric_altitude;
            GDL90_SENSOR_BARO_MESSAGE sensor_message;
#endif
        } decoded;
    } rx;

    struct {
        uint32_t last_packet_GPS_ms; // out
        uint32_t last_packet_Transponder_Control_ms; // out
        uint32_t last_packet_Transponder_Status_ms; // in
        uint32_t last_packet_Transponder_Heartbeat_ms; // in
        uint32_t last_packet_Transponder_Ownship_ms; // in
        uint32_t last_gcs_send_message_Transponder_Status_ms; // out
        uint32_t last_packet_Request_Transponder_Config_ms;  // out
        uint32_t last_packet_Transponder_Config_ms; // in
        uint32_t request_Transponder_Config_tries;
        uint32_t last_packet_Request_Transponder_Id_ms; // out
        uint32_t last_packet_Transponder_Id_ms; // in
        uint32_t request_Transponder_Id_tries;

    } run_state;

};
#endif // HAL_ADSB_UCP_ENABLED


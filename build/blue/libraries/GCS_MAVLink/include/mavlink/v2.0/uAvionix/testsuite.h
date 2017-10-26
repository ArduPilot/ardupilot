/** @file
 *    @brief MAVLink comm protocol testsuite generated from uAvionix.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef UAVIONIX_TESTSUITE_H
#define UAVIONIX_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_uAvionix(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_uAvionix(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_uavionix_adsb_out_cfg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uavionix_adsb_out_cfg_t packet_in = {
        963497464,17443,"GHIJKLMN",242,53,120,187,254
    };
    mavlink_uavionix_adsb_out_cfg_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ICAO = packet_in.ICAO;
        packet1.stallSpeed = packet_in.stallSpeed;
        packet1.emitterType = packet_in.emitterType;
        packet1.aircraftSize = packet_in.aircraftSize;
        packet1.gpsOffsetLat = packet_in.gpsOffsetLat;
        packet1.gpsOffsetLon = packet_in.gpsOffsetLon;
        packet1.rfSelect = packet_in.rfSelect;
        
        mav_array_memcpy(packet1.callsign, packet_in.callsign, sizeof(char)*9);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_out_cfg_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uavionix_adsb_out_cfg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_out_cfg_pack(system_id, component_id, &msg , packet1.ICAO , packet1.callsign , packet1.emitterType , packet1.aircraftSize , packet1.gpsOffsetLat , packet1.gpsOffsetLon , packet1.stallSpeed , packet1.rfSelect );
    mavlink_msg_uavionix_adsb_out_cfg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_out_cfg_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ICAO , packet1.callsign , packet1.emitterType , packet1.aircraftSize , packet1.gpsOffsetLat , packet1.gpsOffsetLon , packet1.stallSpeed , packet1.rfSelect );
    mavlink_msg_uavionix_adsb_out_cfg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uavionix_adsb_out_cfg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_out_cfg_send(MAVLINK_COMM_1 , packet1.ICAO , packet1.callsign , packet1.emitterType , packet1.aircraftSize , packet1.gpsOffsetLat , packet1.gpsOffsetLon , packet1.stallSpeed , packet1.rfSelect );
    mavlink_msg_uavionix_adsb_out_cfg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_uavionix_adsb_out_dynamic(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uavionix_adsb_out_dynamic_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,963498504,18483,18587,18691,18795,18899,19003,19107,247,58,125
    };
    mavlink_uavionix_adsb_out_dynamic_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.utcTime = packet_in.utcTime;
        packet1.gpsLat = packet_in.gpsLat;
        packet1.gpsLon = packet_in.gpsLon;
        packet1.gpsAlt = packet_in.gpsAlt;
        packet1.baroAltMSL = packet_in.baroAltMSL;
        packet1.accuracyHor = packet_in.accuracyHor;
        packet1.accuracyVert = packet_in.accuracyVert;
        packet1.accuracyVel = packet_in.accuracyVel;
        packet1.velVert = packet_in.velVert;
        packet1.velNS = packet_in.velNS;
        packet1.VelEW = packet_in.VelEW;
        packet1.state = packet_in.state;
        packet1.squawk = packet_in.squawk;
        packet1.gpsFix = packet_in.gpsFix;
        packet1.numSats = packet_in.numSats;
        packet1.emergencyStatus = packet_in.emergencyStatus;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_out_dynamic_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uavionix_adsb_out_dynamic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_out_dynamic_pack(system_id, component_id, &msg , packet1.utcTime , packet1.gpsLat , packet1.gpsLon , packet1.gpsAlt , packet1.gpsFix , packet1.numSats , packet1.baroAltMSL , packet1.accuracyHor , packet1.accuracyVert , packet1.accuracyVel , packet1.velVert , packet1.velNS , packet1.VelEW , packet1.emergencyStatus , packet1.state , packet1.squawk );
    mavlink_msg_uavionix_adsb_out_dynamic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_out_dynamic_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.utcTime , packet1.gpsLat , packet1.gpsLon , packet1.gpsAlt , packet1.gpsFix , packet1.numSats , packet1.baroAltMSL , packet1.accuracyHor , packet1.accuracyVert , packet1.accuracyVel , packet1.velVert , packet1.velNS , packet1.VelEW , packet1.emergencyStatus , packet1.state , packet1.squawk );
    mavlink_msg_uavionix_adsb_out_dynamic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uavionix_adsb_out_dynamic_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_out_dynamic_send(MAVLINK_COMM_1 , packet1.utcTime , packet1.gpsLat , packet1.gpsLon , packet1.gpsAlt , packet1.gpsFix , packet1.numSats , packet1.baroAltMSL , packet1.accuracyHor , packet1.accuracyVert , packet1.accuracyVel , packet1.velVert , packet1.velNS , packet1.VelEW , packet1.emergencyStatus , packet1.state , packet1.squawk );
    mavlink_msg_uavionix_adsb_out_dynamic_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_uavionix_adsb_transceiver_health_report(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uavionix_adsb_transceiver_health_report_t packet_in = {
        5
    };
    mavlink_uavionix_adsb_transceiver_health_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rfHealth = packet_in.rfHealth;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_transceiver_health_report_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uavionix_adsb_transceiver_health_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_transceiver_health_report_pack(system_id, component_id, &msg , packet1.rfHealth );
    mavlink_msg_uavionix_adsb_transceiver_health_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_transceiver_health_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rfHealth );
    mavlink_msg_uavionix_adsb_transceiver_health_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uavionix_adsb_transceiver_health_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uavionix_adsb_transceiver_health_report_send(MAVLINK_COMM_1 , packet1.rfHealth );
    mavlink_msg_uavionix_adsb_transceiver_health_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_uAvionix(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_uavionix_adsb_out_cfg(system_id, component_id, last_msg);
    mavlink_test_uavionix_adsb_out_dynamic(system_id, component_id, last_msg);
    mavlink_test_uavionix_adsb_transceiver_health_report(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // UAVIONIX_TESTSUITE_H

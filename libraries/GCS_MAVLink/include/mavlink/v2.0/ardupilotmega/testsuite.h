/** @file
 *    @brief MAVLink comm protocol testsuite generated from ardupilotmega.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ARDUPILOTMEGA_TESTSUITE_H
#define ARDUPILOTMEGA_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_uAvionix(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_icarous(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_loweheiser(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_cubepilot(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_csAirLink(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ardupilotmega(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_uAvionix(system_id, component_id, last_msg);
    mavlink_test_icarous(system_id, component_id, last_msg);
    mavlink_test_loweheiser(system_id, component_id, last_msg);
    mavlink_test_cubepilot(system_id, component_id, last_msg);
    mavlink_test_csAirLink(system_id, component_id, last_msg);
    mavlink_test_ardupilotmega(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"
#include "../uAvionix/testsuite.h"
#include "../icarous/testsuite.h"
#include "../loweheiser/testsuite.h"
#include "../cubepilot/testsuite.h"
#include "../csAirLink/testsuite.h"


static void mavlink_test_sensor_offsets(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENSOR_OFFSETS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sensor_offsets_t packet_in = {
        17.0,963497672,963497880,101.0,129.0,157.0,185.0,213.0,241.0,19107,19211,19315
    };
    mavlink_sensor_offsets_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mag_declination = packet_in.mag_declination;
        packet1.raw_press = packet_in.raw_press;
        packet1.raw_temp = packet_in.raw_temp;
        packet1.gyro_cal_x = packet_in.gyro_cal_x;
        packet1.gyro_cal_y = packet_in.gyro_cal_y;
        packet1.gyro_cal_z = packet_in.gyro_cal_z;
        packet1.accel_cal_x = packet_in.accel_cal_x;
        packet1.accel_cal_y = packet_in.accel_cal_y;
        packet1.accel_cal_z = packet_in.accel_cal_z;
        packet1.mag_ofs_x = packet_in.mag_ofs_x;
        packet1.mag_ofs_y = packet_in.mag_ofs_y;
        packet1.mag_ofs_z = packet_in.mag_ofs_z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENSOR_OFFSETS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENSOR_OFFSETS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_offsets_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sensor_offsets_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_offsets_pack(system_id, component_id, &msg , packet1.mag_ofs_x , packet1.mag_ofs_y , packet1.mag_ofs_z , packet1.mag_declination , packet1.raw_press , packet1.raw_temp , packet1.gyro_cal_x , packet1.gyro_cal_y , packet1.gyro_cal_z , packet1.accel_cal_x , packet1.accel_cal_y , packet1.accel_cal_z );
    mavlink_msg_sensor_offsets_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_offsets_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mag_ofs_x , packet1.mag_ofs_y , packet1.mag_ofs_z , packet1.mag_declination , packet1.raw_press , packet1.raw_temp , packet1.gyro_cal_x , packet1.gyro_cal_y , packet1.gyro_cal_z , packet1.accel_cal_x , packet1.accel_cal_y , packet1.accel_cal_z );
    mavlink_msg_sensor_offsets_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sensor_offsets_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_offsets_send(MAVLINK_COMM_1 , packet1.mag_ofs_x , packet1.mag_ofs_y , packet1.mag_ofs_z , packet1.mag_declination , packet1.raw_press , packet1.raw_temp , packet1.gyro_cal_x , packet1.gyro_cal_y , packet1.gyro_cal_z , packet1.accel_cal_x , packet1.accel_cal_y , packet1.accel_cal_z );
    mavlink_msg_sensor_offsets_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SENSOR_OFFSETS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SENSOR_OFFSETS) != NULL);
#endif
}

static void mavlink_test_set_mag_offsets(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_MAG_OFFSETS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_mag_offsets_t packet_in = {
        17235,17339,17443,151,218
    };
    mavlink_set_mag_offsets_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mag_ofs_x = packet_in.mag_ofs_x;
        packet1.mag_ofs_y = packet_in.mag_ofs_y;
        packet1.mag_ofs_z = packet_in.mag_ofs_z;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_MAG_OFFSETS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_MAG_OFFSETS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_mag_offsets_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_mag_offsets_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_mag_offsets_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.mag_ofs_x , packet1.mag_ofs_y , packet1.mag_ofs_z );
    mavlink_msg_set_mag_offsets_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_mag_offsets_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.mag_ofs_x , packet1.mag_ofs_y , packet1.mag_ofs_z );
    mavlink_msg_set_mag_offsets_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_mag_offsets_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_mag_offsets_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.mag_ofs_x , packet1.mag_ofs_y , packet1.mag_ofs_z );
    mavlink_msg_set_mag_offsets_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_MAG_OFFSETS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_MAG_OFFSETS) != NULL);
#endif
}

static void mavlink_test_meminfo(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MEMINFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_meminfo_t packet_in = {
        17235,17339,963497672
    };
    mavlink_meminfo_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.brkval = packet_in.brkval;
        packet1.freemem = packet_in.freemem;
        packet1.freemem32 = packet_in.freemem32;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MEMINFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MEMINFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_meminfo_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_meminfo_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_meminfo_pack(system_id, component_id, &msg , packet1.brkval , packet1.freemem , packet1.freemem32 );
    mavlink_msg_meminfo_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_meminfo_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.brkval , packet1.freemem , packet1.freemem32 );
    mavlink_msg_meminfo_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_meminfo_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_meminfo_send(MAVLINK_COMM_1 , packet1.brkval , packet1.freemem , packet1.freemem32 );
    mavlink_msg_meminfo_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MEMINFO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MEMINFO) != NULL);
#endif
}

static void mavlink_test_ap_adc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AP_ADC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ap_adc_t packet_in = {
        17235,17339,17443,17547,17651,17755
    };
    mavlink_ap_adc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.adc1 = packet_in.adc1;
        packet1.adc2 = packet_in.adc2;
        packet1.adc3 = packet_in.adc3;
        packet1.adc4 = packet_in.adc4;
        packet1.adc5 = packet_in.adc5;
        packet1.adc6 = packet_in.adc6;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AP_ADC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AP_ADC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ap_adc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ap_adc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ap_adc_pack(system_id, component_id, &msg , packet1.adc1 , packet1.adc2 , packet1.adc3 , packet1.adc4 , packet1.adc5 , packet1.adc6 );
    mavlink_msg_ap_adc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ap_adc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.adc1 , packet1.adc2 , packet1.adc3 , packet1.adc4 , packet1.adc5 , packet1.adc6 );
    mavlink_msg_ap_adc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ap_adc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ap_adc_send(MAVLINK_COMM_1 , packet1.adc1 , packet1.adc2 , packet1.adc3 , packet1.adc4 , packet1.adc5 , packet1.adc6 );
    mavlink_msg_ap_adc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AP_ADC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AP_ADC) != NULL);
#endif
}

static void mavlink_test_digicam_configure(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DIGICAM_CONFIGURE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_digicam_configure_t packet_in = {
        17.0,17443,151,218,29,96,163,230,41,108,175
    };
    mavlink_digicam_configure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.extra_value = packet_in.extra_value;
        packet1.shutter_speed = packet_in.shutter_speed;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mode = packet_in.mode;
        packet1.aperture = packet_in.aperture;
        packet1.iso = packet_in.iso;
        packet1.exposure_type = packet_in.exposure_type;
        packet1.command_id = packet_in.command_id;
        packet1.engine_cut_off = packet_in.engine_cut_off;
        packet1.extra_param = packet_in.extra_param;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DIGICAM_CONFIGURE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_digicam_configure_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_digicam_configure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_digicam_configure_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.mode , packet1.shutter_speed , packet1.aperture , packet1.iso , packet1.exposure_type , packet1.command_id , packet1.engine_cut_off , packet1.extra_param , packet1.extra_value );
    mavlink_msg_digicam_configure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_digicam_configure_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.mode , packet1.shutter_speed , packet1.aperture , packet1.iso , packet1.exposure_type , packet1.command_id , packet1.engine_cut_off , packet1.extra_param , packet1.extra_value );
    mavlink_msg_digicam_configure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_digicam_configure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_digicam_configure_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.mode , packet1.shutter_speed , packet1.aperture , packet1.iso , packet1.exposure_type , packet1.command_id , packet1.engine_cut_off , packet1.extra_param , packet1.extra_value );
    mavlink_msg_digicam_configure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DIGICAM_CONFIGURE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DIGICAM_CONFIGURE) != NULL);
#endif
}

static void mavlink_test_digicam_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DIGICAM_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_digicam_control_t packet_in = {
        17.0,17,84,151,218,29,96,163,230,41
    };
    mavlink_digicam_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.extra_value = packet_in.extra_value;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.session = packet_in.session;
        packet1.zoom_pos = packet_in.zoom_pos;
        packet1.zoom_step = packet_in.zoom_step;
        packet1.focus_lock = packet_in.focus_lock;
        packet1.shot = packet_in.shot;
        packet1.command_id = packet_in.command_id;
        packet1.extra_param = packet_in.extra_param;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DIGICAM_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DIGICAM_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_digicam_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_digicam_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_digicam_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.session , packet1.zoom_pos , packet1.zoom_step , packet1.focus_lock , packet1.shot , packet1.command_id , packet1.extra_param , packet1.extra_value );
    mavlink_msg_digicam_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_digicam_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.session , packet1.zoom_pos , packet1.zoom_step , packet1.focus_lock , packet1.shot , packet1.command_id , packet1.extra_param , packet1.extra_value );
    mavlink_msg_digicam_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_digicam_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_digicam_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.session , packet1.zoom_pos , packet1.zoom_step , packet1.focus_lock , packet1.shot , packet1.command_id , packet1.extra_param , packet1.extra_value );
    mavlink_msg_digicam_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DIGICAM_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DIGICAM_CONTROL) != NULL);
#endif
}

static void mavlink_test_mount_configure(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOUNT_CONFIGURE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mount_configure_t packet_in = {
        5,72,139,206,17,84
    };
    mavlink_mount_configure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mount_mode = packet_in.mount_mode;
        packet1.stab_roll = packet_in.stab_roll;
        packet1.stab_pitch = packet_in.stab_pitch;
        packet1.stab_yaw = packet_in.stab_yaw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOUNT_CONFIGURE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOUNT_CONFIGURE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_configure_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mount_configure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_configure_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.mount_mode , packet1.stab_roll , packet1.stab_pitch , packet1.stab_yaw );
    mavlink_msg_mount_configure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_configure_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.mount_mode , packet1.stab_roll , packet1.stab_pitch , packet1.stab_yaw );
    mavlink_msg_mount_configure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mount_configure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_configure_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.mount_mode , packet1.stab_roll , packet1.stab_pitch , packet1.stab_yaw );
    mavlink_msg_mount_configure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOUNT_CONFIGURE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOUNT_CONFIGURE) != NULL);
#endif
}

static void mavlink_test_mount_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOUNT_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mount_control_t packet_in = {
        963497464,963497672,963497880,41,108,175
    };
    mavlink_mount_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.input_a = packet_in.input_a;
        packet1.input_b = packet_in.input_b;
        packet1.input_c = packet_in.input_c;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.save_position = packet_in.save_position;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOUNT_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mount_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.input_a , packet1.input_b , packet1.input_c , packet1.save_position );
    mavlink_msg_mount_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.input_a , packet1.input_b , packet1.input_c , packet1.save_position );
    mavlink_msg_mount_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mount_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.input_a , packet1.input_b , packet1.input_c , packet1.save_position );
    mavlink_msg_mount_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOUNT_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOUNT_CONTROL) != NULL);
#endif
}

static void mavlink_test_mount_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOUNT_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mount_status_t packet_in = {
        963497464,963497672,963497880,41,108,175
    };
    mavlink_mount_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pointing_a = packet_in.pointing_a;
        packet1.pointing_b = packet_in.pointing_b;
        packet1.pointing_c = packet_in.pointing_c;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.mount_mode = packet_in.mount_mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOUNT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mount_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_status_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.pointing_a , packet1.pointing_b , packet1.pointing_c , packet1.mount_mode );
    mavlink_msg_mount_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.pointing_a , packet1.pointing_b , packet1.pointing_c , packet1.mount_mode );
    mavlink_msg_mount_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mount_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mount_status_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.pointing_a , packet1.pointing_b , packet1.pointing_c , packet1.mount_mode );
    mavlink_msg_mount_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOUNT_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOUNT_STATUS) != NULL);
#endif
}

static void mavlink_test_fence_point(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FENCE_POINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_fence_point_t packet_in = {
        17.0,45.0,29,96,163,230
    };
    mavlink_fence_point_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.idx = packet_in.idx;
        packet1.count = packet_in.count;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FENCE_POINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_point_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_fence_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_point_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.idx , packet1.count , packet1.lat , packet1.lng );
    mavlink_msg_fence_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_point_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.idx , packet1.count , packet1.lat , packet1.lng );
    mavlink_msg_fence_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_fence_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_point_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.idx , packet1.count , packet1.lat , packet1.lng );
    mavlink_msg_fence_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FENCE_POINT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FENCE_POINT) != NULL);
#endif
}

static void mavlink_test_fence_fetch_point(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FENCE_FETCH_POINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_fence_fetch_point_t packet_in = {
        5,72,139
    };
    mavlink_fence_fetch_point_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.idx = packet_in.idx;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FENCE_FETCH_POINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FENCE_FETCH_POINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_fetch_point_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_fence_fetch_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_fetch_point_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.idx );
    mavlink_msg_fence_fetch_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_fetch_point_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.idx );
    mavlink_msg_fence_fetch_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_fence_fetch_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fence_fetch_point_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.idx );
    mavlink_msg_fence_fetch_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FENCE_FETCH_POINT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FENCE_FETCH_POINT) != NULL);
#endif
}

static void mavlink_test_ahrs(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AHRS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ahrs_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0
    };
    mavlink_ahrs_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.omegaIx = packet_in.omegaIx;
        packet1.omegaIy = packet_in.omegaIy;
        packet1.omegaIz = packet_in.omegaIz;
        packet1.accel_weight = packet_in.accel_weight;
        packet1.renorm_val = packet_in.renorm_val;
        packet1.error_rp = packet_in.error_rp;
        packet1.error_yaw = packet_in.error_yaw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AHRS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AHRS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ahrs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs_pack(system_id, component_id, &msg , packet1.omegaIx , packet1.omegaIy , packet1.omegaIz , packet1.accel_weight , packet1.renorm_val , packet1.error_rp , packet1.error_yaw );
    mavlink_msg_ahrs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.omegaIx , packet1.omegaIy , packet1.omegaIz , packet1.accel_weight , packet1.renorm_val , packet1.error_rp , packet1.error_yaw );
    mavlink_msg_ahrs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ahrs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs_send(MAVLINK_COMM_1 , packet1.omegaIx , packet1.omegaIy , packet1.omegaIz , packet1.accel_weight , packet1.renorm_val , packet1.error_rp , packet1.error_yaw );
    mavlink_msg_ahrs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AHRS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AHRS) != NULL);
#endif
}

static void mavlink_test_simstate(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SIMSTATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_simstate_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,963499336,963499544
    };
    mavlink_simstate_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.xacc = packet_in.xacc;
        packet1.yacc = packet_in.yacc;
        packet1.zacc = packet_in.zacc;
        packet1.xgyro = packet_in.xgyro;
        packet1.ygyro = packet_in.ygyro;
        packet1.zgyro = packet_in.zgyro;
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SIMSTATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SIMSTATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_simstate_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_simstate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_simstate_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.lat , packet1.lng );
    mavlink_msg_simstate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_simstate_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.lat , packet1.lng );
    mavlink_msg_simstate_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_simstate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_simstate_send(MAVLINK_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw , packet1.xacc , packet1.yacc , packet1.zacc , packet1.xgyro , packet1.ygyro , packet1.zgyro , packet1.lat , packet1.lng );
    mavlink_msg_simstate_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SIMSTATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SIMSTATE) != NULL);
#endif
}

static void mavlink_test_hwstatus(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HWSTATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_hwstatus_t packet_in = {
        17235,139
    };
    mavlink_hwstatus_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.Vcc = packet_in.Vcc;
        packet1.I2Cerr = packet_in.I2Cerr;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HWSTATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HWSTATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hwstatus_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_hwstatus_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hwstatus_pack(system_id, component_id, &msg , packet1.Vcc , packet1.I2Cerr );
    mavlink_msg_hwstatus_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hwstatus_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Vcc , packet1.I2Cerr );
    mavlink_msg_hwstatus_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_hwstatus_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_hwstatus_send(MAVLINK_COMM_1 , packet1.Vcc , packet1.I2Cerr );
    mavlink_msg_hwstatus_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("HWSTATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_HWSTATUS) != NULL);
#endif
}

static void mavlink_test_radio(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RADIO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_radio_t packet_in = {
        17235,17339,17,84,151,218,29
    };
    mavlink_radio_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rxerrors = packet_in.rxerrors;
        packet1.fixed = packet_in.fixed;
        packet1.rssi = packet_in.rssi;
        packet1.remrssi = packet_in.remrssi;
        packet1.txbuf = packet_in.txbuf;
        packet1.noise = packet_in.noise;
        packet1.remnoise = packet_in.remnoise;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RADIO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RADIO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radio_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_radio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radio_pack(system_id, component_id, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
    mavlink_msg_radio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radio_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
    mavlink_msg_radio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_radio_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_radio_send(MAVLINK_COMM_1 , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
    mavlink_msg_radio_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RADIO") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RADIO) != NULL);
#endif
}

static void mavlink_test_limits_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LIMITS_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_limits_status_t packet_in = {
        963497464,963497672,963497880,963498088,18067,187,254,65,132
    };
    mavlink_limits_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.last_trigger = packet_in.last_trigger;
        packet1.last_action = packet_in.last_action;
        packet1.last_recovery = packet_in.last_recovery;
        packet1.last_clear = packet_in.last_clear;
        packet1.breach_count = packet_in.breach_count;
        packet1.limits_state = packet_in.limits_state;
        packet1.mods_enabled = packet_in.mods_enabled;
        packet1.mods_required = packet_in.mods_required;
        packet1.mods_triggered = packet_in.mods_triggered;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LIMITS_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LIMITS_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_limits_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_limits_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_limits_status_pack(system_id, component_id, &msg , packet1.limits_state , packet1.last_trigger , packet1.last_action , packet1.last_recovery , packet1.last_clear , packet1.breach_count , packet1.mods_enabled , packet1.mods_required , packet1.mods_triggered );
    mavlink_msg_limits_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_limits_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.limits_state , packet1.last_trigger , packet1.last_action , packet1.last_recovery , packet1.last_clear , packet1.breach_count , packet1.mods_enabled , packet1.mods_required , packet1.mods_triggered );
    mavlink_msg_limits_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_limits_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_limits_status_send(MAVLINK_COMM_1 , packet1.limits_state , packet1.last_trigger , packet1.last_action , packet1.last_recovery , packet1.last_clear , packet1.breach_count , packet1.mods_enabled , packet1.mods_required , packet1.mods_triggered );
    mavlink_msg_limits_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LIMITS_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LIMITS_STATUS) != NULL);
#endif
}

static void mavlink_test_wind(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WIND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_wind_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_wind_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.direction = packet_in.direction;
        packet1.speed = packet_in.speed;
        packet1.speed_z = packet_in.speed_z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WIND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WIND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_wind_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_pack(system_id, component_id, &msg , packet1.direction , packet1.speed , packet1.speed_z );
    mavlink_msg_wind_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.direction , packet1.speed , packet1.speed_z );
    mavlink_msg_wind_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_wind_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wind_send(MAVLINK_COMM_1 , packet1.direction , packet1.speed , packet1.speed_z );
    mavlink_msg_wind_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WIND") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WIND) != NULL);
#endif
}

static void mavlink_test_data16(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DATA16 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_data16_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154 }
    };
    mavlink_data16_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.type = packet_in.type;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DATA16_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DATA16_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data16_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_data16_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data16_pack(system_id, component_id, &msg , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data16_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data16_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data16_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_data16_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data16_send(MAVLINK_COMM_1 , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data16_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DATA16") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DATA16) != NULL);
#endif
}

static void mavlink_test_data32(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DATA32 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_data32_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170 }
    };
    mavlink_data32_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.type = packet_in.type;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DATA32_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DATA32_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data32_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_data32_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data32_pack(system_id, component_id, &msg , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data32_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data32_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data32_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_data32_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data32_send(MAVLINK_COMM_1 , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data32_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DATA32") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DATA32) != NULL);
#endif
}

static void mavlink_test_data64(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DATA64 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_data64_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }
    };
    mavlink_data64_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.type = packet_in.type;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DATA64_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DATA64_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data64_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_data64_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data64_pack(system_id, component_id, &msg , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data64_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data64_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data64_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_data64_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data64_send(MAVLINK_COMM_1 , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data64_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DATA64") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DATA64) != NULL);
#endif
}

static void mavlink_test_data96(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DATA96 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_data96_t packet_in = {
        5,72,{ 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234 }
    };
    mavlink_data96_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.type = packet_in.type;
        packet1.len = packet_in.len;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*96);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DATA96_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DATA96_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data96_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_data96_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data96_pack(system_id, component_id, &msg , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data96_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data96_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data96_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_data96_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data96_send(MAVLINK_COMM_1 , packet1.type , packet1.len , packet1.data );
    mavlink_msg_data96_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DATA96") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DATA96) != NULL);
#endif
}

static void mavlink_test_rangefinder(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RANGEFINDER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rangefinder_t packet_in = {
        17.0,45.0
    };
    mavlink_rangefinder_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.distance = packet_in.distance;
        packet1.voltage = packet_in.voltage;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RANGEFINDER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rangefinder_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rangefinder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rangefinder_pack(system_id, component_id, &msg , packet1.distance , packet1.voltage );
    mavlink_msg_rangefinder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rangefinder_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.distance , packet1.voltage );
    mavlink_msg_rangefinder_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rangefinder_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rangefinder_send(MAVLINK_COMM_1 , packet1.distance , packet1.voltage );
    mavlink_msg_rangefinder_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RANGEFINDER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RANGEFINDER) != NULL);
#endif
}

static void mavlink_test_airspeed_autocal(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AIRSPEED_AUTOCAL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_airspeed_autocal_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0
    };
    mavlink_airspeed_autocal_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vx = packet_in.vx;
        packet1.vy = packet_in.vy;
        packet1.vz = packet_in.vz;
        packet1.diff_pressure = packet_in.diff_pressure;
        packet1.EAS2TAS = packet_in.EAS2TAS;
        packet1.ratio = packet_in.ratio;
        packet1.state_x = packet_in.state_x;
        packet1.state_y = packet_in.state_y;
        packet1.state_z = packet_in.state_z;
        packet1.Pax = packet_in.Pax;
        packet1.Pby = packet_in.Pby;
        packet1.Pcz = packet_in.Pcz;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airspeed_autocal_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_airspeed_autocal_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airspeed_autocal_pack(system_id, component_id, &msg , packet1.vx , packet1.vy , packet1.vz , packet1.diff_pressure , packet1.EAS2TAS , packet1.ratio , packet1.state_x , packet1.state_y , packet1.state_z , packet1.Pax , packet1.Pby , packet1.Pcz );
    mavlink_msg_airspeed_autocal_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airspeed_autocal_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vx , packet1.vy , packet1.vz , packet1.diff_pressure , packet1.EAS2TAS , packet1.ratio , packet1.state_x , packet1.state_y , packet1.state_z , packet1.Pax , packet1.Pby , packet1.Pcz );
    mavlink_msg_airspeed_autocal_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_airspeed_autocal_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_airspeed_autocal_send(MAVLINK_COMM_1 , packet1.vx , packet1.vy , packet1.vz , packet1.diff_pressure , packet1.EAS2TAS , packet1.ratio , packet1.state_x , packet1.state_y , packet1.state_z , packet1.Pax , packet1.Pby , packet1.Pcz );
    mavlink_msg_airspeed_autocal_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AIRSPEED_AUTOCAL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AIRSPEED_AUTOCAL) != NULL);
#endif
}

static void mavlink_test_rally_point(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RALLY_POINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rally_point_t packet_in = {
        963497464,963497672,17651,17755,17859,175,242,53,120,187
    };
    mavlink_rally_point_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        packet1.alt = packet_in.alt;
        packet1.break_alt = packet_in.break_alt;
        packet1.land_dir = packet_in.land_dir;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.idx = packet_in.idx;
        packet1.count = packet_in.count;
        packet1.flags = packet_in.flags;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RALLY_POINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_point_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rally_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_point_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.idx , packet1.count , packet1.lat , packet1.lng , packet1.alt , packet1.break_alt , packet1.land_dir , packet1.flags );
    mavlink_msg_rally_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_point_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.idx , packet1.count , packet1.lat , packet1.lng , packet1.alt , packet1.break_alt , packet1.land_dir , packet1.flags );
    mavlink_msg_rally_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rally_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_point_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.idx , packet1.count , packet1.lat , packet1.lng , packet1.alt , packet1.break_alt , packet1.land_dir , packet1.flags );
    mavlink_msg_rally_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RALLY_POINT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RALLY_POINT) != NULL);
#endif
}

static void mavlink_test_rally_fetch_point(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RALLY_FETCH_POINT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rally_fetch_point_t packet_in = {
        5,72,139
    };
    mavlink_rally_fetch_point_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.idx = packet_in.idx;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RALLY_FETCH_POINT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_fetch_point_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rally_fetch_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_fetch_point_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.idx );
    mavlink_msg_rally_fetch_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_fetch_point_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.idx );
    mavlink_msg_rally_fetch_point_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rally_fetch_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rally_fetch_point_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.idx );
    mavlink_msg_rally_fetch_point_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RALLY_FETCH_POINT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RALLY_FETCH_POINT) != NULL);
#endif
}

static void mavlink_test_compassmot_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMPASSMOT_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_compassmot_status_t packet_in = {
        17.0,45.0,73.0,101.0,18067,18171
    };
    mavlink_compassmot_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.current = packet_in.current;
        packet1.CompensationX = packet_in.CompensationX;
        packet1.CompensationY = packet_in.CompensationY;
        packet1.CompensationZ = packet_in.CompensationZ;
        packet1.throttle = packet_in.throttle;
        packet1.interference = packet_in.interference;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMPASSMOT_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_compassmot_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_compassmot_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_compassmot_status_pack(system_id, component_id, &msg , packet1.throttle , packet1.current , packet1.interference , packet1.CompensationX , packet1.CompensationY , packet1.CompensationZ );
    mavlink_msg_compassmot_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_compassmot_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.throttle , packet1.current , packet1.interference , packet1.CompensationX , packet1.CompensationY , packet1.CompensationZ );
    mavlink_msg_compassmot_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_compassmot_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_compassmot_status_send(MAVLINK_COMM_1 , packet1.throttle , packet1.current , packet1.interference , packet1.CompensationX , packet1.CompensationY , packet1.CompensationZ );
    mavlink_msg_compassmot_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COMPASSMOT_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COMPASSMOT_STATUS) != NULL);
#endif
}

static void mavlink_test_ahrs2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AHRS2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ahrs2_t packet_in = {
        17.0,45.0,73.0,101.0,963498296,963498504
    };
    mavlink_ahrs2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.altitude = packet_in.altitude;
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AHRS2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AHRS2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ahrs2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs2_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng );
    mavlink_msg_ahrs2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng );
    mavlink_msg_ahrs2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ahrs2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs2_send(MAVLINK_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng );
    mavlink_msg_ahrs2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AHRS2") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AHRS2) != NULL);
#endif
}

static void mavlink_test_camera_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_status_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,18483,211,22,89
    };
    mavlink_camera_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.p1 = packet_in.p1;
        packet1.p2 = packet_in.p2;
        packet1.p3 = packet_in.p3;
        packet1.p4 = packet_in.p4;
        packet1.img_idx = packet_in.img_idx;
        packet1.target_system = packet_in.target_system;
        packet1.cam_idx = packet_in.cam_idx;
        packet1.event_id = packet_in.event_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_status_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.event_id , packet1.p1 , packet1.p2 , packet1.p3 , packet1.p4 );
    mavlink_msg_camera_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.event_id , packet1.p1 , packet1.p2 , packet1.p3 , packet1.p4 );
    mavlink_msg_camera_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_status_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.event_id , packet1.p1 , packet1.p2 , packet1.p3 , packet1.p4 );
    mavlink_msg_camera_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_STATUS) != NULL);
#endif
}

static void mavlink_test_camera_feedback(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAMERA_FEEDBACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_camera_feedback_t packet_in = {
        93372036854775807ULL,963497880,963498088,129.0,157.0,185.0,213.0,241.0,269.0,19315,3,70,137,19575
    };
    mavlink_camera_feedback_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        packet1.alt_msl = packet_in.alt_msl;
        packet1.alt_rel = packet_in.alt_rel;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.foc_len = packet_in.foc_len;
        packet1.img_idx = packet_in.img_idx;
        packet1.target_system = packet_in.target_system;
        packet1.cam_idx = packet_in.cam_idx;
        packet1.flags = packet_in.flags;
        packet1.completed_captures = packet_in.completed_captures;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAMERA_FEEDBACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_feedback_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_camera_feedback_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_feedback_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.lat , packet1.lng , packet1.alt_msl , packet1.alt_rel , packet1.roll , packet1.pitch , packet1.yaw , packet1.foc_len , packet1.flags , packet1.completed_captures );
    mavlink_msg_camera_feedback_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_feedback_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.lat , packet1.lng , packet1.alt_msl , packet1.alt_rel , packet1.roll , packet1.pitch , packet1.yaw , packet1.foc_len , packet1.flags , packet1.completed_captures );
    mavlink_msg_camera_feedback_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_camera_feedback_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_camera_feedback_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_system , packet1.cam_idx , packet1.img_idx , packet1.lat , packet1.lng , packet1.alt_msl , packet1.alt_rel , packet1.roll , packet1.pitch , packet1.yaw , packet1.foc_len , packet1.flags , packet1.completed_captures );
    mavlink_msg_camera_feedback_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAMERA_FEEDBACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAMERA_FEEDBACK) != NULL);
#endif
}

static void mavlink_test_battery2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BATTERY2 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_battery2_t packet_in = {
        17235,17339
    };
    mavlink_battery2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.voltage = packet_in.voltage;
        packet1.current_battery = packet_in.current_battery;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BATTERY2_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BATTERY2_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery2_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_battery2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery2_pack(system_id, component_id, &msg , packet1.voltage , packet1.current_battery );
    mavlink_msg_battery2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.voltage , packet1.current_battery );
    mavlink_msg_battery2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_battery2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery2_send(MAVLINK_COMM_1 , packet1.voltage , packet1.current_battery );
    mavlink_msg_battery2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("BATTERY2") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_BATTERY2) != NULL);
#endif
}

static void mavlink_test_ahrs3(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AHRS3 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ahrs3_t packet_in = {
        17.0,45.0,73.0,101.0,963498296,963498504,185.0,213.0,241.0,269.0
    };
    mavlink_ahrs3_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.altitude = packet_in.altitude;
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        packet1.v1 = packet_in.v1;
        packet1.v2 = packet_in.v2;
        packet1.v3 = packet_in.v3;
        packet1.v4 = packet_in.v4;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AHRS3_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AHRS3_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs3_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ahrs3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs3_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng , packet1.v1 , packet1.v2 , packet1.v3 , packet1.v4 );
    mavlink_msg_ahrs3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs3_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng , packet1.v1 , packet1.v2 , packet1.v3 , packet1.v4 );
    mavlink_msg_ahrs3_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ahrs3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ahrs3_send(MAVLINK_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.lat , packet1.lng , packet1.v1 , packet1.v2 , packet1.v3 , packet1.v4 );
    mavlink_msg_ahrs3_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AHRS3") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AHRS3) != NULL);
#endif
}

static void mavlink_test_autopilot_version_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_autopilot_version_request_t packet_in = {
        5,72
    };
    mavlink_autopilot_version_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_autopilot_version_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_request_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_autopilot_version_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component );
    mavlink_msg_autopilot_version_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_autopilot_version_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_autopilot_version_request_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component );
    mavlink_msg_autopilot_version_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AUTOPILOT_VERSION_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST) != NULL);
#endif
}

static void mavlink_test_remote_log_data_block(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_remote_log_data_block_t packet_in = {
        963497464,17,84,{ 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94 }
    };
    mavlink_remote_log_data_block_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seqno = packet_in.seqno;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*200);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_data_block_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_remote_log_data_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_data_block_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seqno , packet1.data );
    mavlink_msg_remote_log_data_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_data_block_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seqno , packet1.data );
    mavlink_msg_remote_log_data_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_remote_log_data_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_data_block_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seqno , packet1.data );
    mavlink_msg_remote_log_data_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("REMOTE_LOG_DATA_BLOCK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK) != NULL);
#endif
}

static void mavlink_test_remote_log_block_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_remote_log_block_status_t packet_in = {
        963497464,17,84,151
    };
    mavlink_remote_log_block_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seqno = packet_in.seqno;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.status = packet_in.status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_block_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_remote_log_block_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_block_status_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.seqno , packet1.status );
    mavlink_msg_remote_log_block_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_block_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.seqno , packet1.status );
    mavlink_msg_remote_log_block_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_remote_log_block_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_remote_log_block_status_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.seqno , packet1.status );
    mavlink_msg_remote_log_block_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("REMOTE_LOG_BLOCK_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS) != NULL);
#endif
}

static void mavlink_test_led_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LED_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_led_control_t packet_in = {
        5,72,139,206,17,{ 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107 }
    };
    mavlink_led_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.instance = packet_in.instance;
        packet1.pattern = packet_in.pattern;
        packet1.custom_len = packet_in.custom_len;
        
        mav_array_memcpy(packet1.custom_bytes, packet_in.custom_bytes, sizeof(uint8_t)*24);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.instance , packet1.pattern , packet1.custom_len , packet1.custom_bytes );
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.instance , packet1.pattern , packet1.custom_len , packet1.custom_bytes );
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_led_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.instance , packet1.pattern , packet1.custom_len , packet1.custom_bytes );
    mavlink_msg_led_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LED_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LED_CONTROL) != NULL);
#endif
}

static void mavlink_test_mag_cal_progress(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAG_CAL_PROGRESS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mag_cal_progress_t packet_in = {
        17.0,45.0,73.0,41,108,175,242,53,{ 120, 121, 122, 123, 124, 125, 126, 127, 128, 129 }
    };
    mavlink_mag_cal_progress_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.direction_x = packet_in.direction_x;
        packet1.direction_y = packet_in.direction_y;
        packet1.direction_z = packet_in.direction_z;
        packet1.compass_id = packet_in.compass_id;
        packet1.cal_mask = packet_in.cal_mask;
        packet1.cal_status = packet_in.cal_status;
        packet1.attempt = packet_in.attempt;
        packet1.completion_pct = packet_in.completion_pct;
        
        mav_array_memcpy(packet1.completion_mask, packet_in.completion_mask, sizeof(uint8_t)*10);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAG_CAL_PROGRESS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mag_cal_progress_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_pack(system_id, component_id, &msg , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.attempt , packet1.completion_pct , packet1.completion_mask , packet1.direction_x , packet1.direction_y , packet1.direction_z );
    mavlink_msg_mag_cal_progress_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.attempt , packet1.completion_pct , packet1.completion_mask , packet1.direction_x , packet1.direction_y , packet1.direction_z );
    mavlink_msg_mag_cal_progress_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mag_cal_progress_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mag_cal_progress_send(MAVLINK_COMM_1 , packet1.compass_id , packet1.cal_mask , packet1.cal_status , packet1.attempt , packet1.completion_pct , packet1.completion_mask , packet1.direction_x , packet1.direction_y , packet1.direction_z );
    mavlink_msg_mag_cal_progress_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAG_CAL_PROGRESS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAG_CAL_PROGRESS) != NULL);
#endif
}

static void mavlink_test_ekf_status_report(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_EKF_STATUS_REPORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ekf_status_report_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,18275,171.0
    };
    mavlink_ekf_status_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.velocity_variance = packet_in.velocity_variance;
        packet1.pos_horiz_variance = packet_in.pos_horiz_variance;
        packet1.pos_vert_variance = packet_in.pos_vert_variance;
        packet1.compass_variance = packet_in.compass_variance;
        packet1.terrain_alt_variance = packet_in.terrain_alt_variance;
        packet1.flags = packet_in.flags;
        packet1.airspeed_variance = packet_in.airspeed_variance;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_EKF_STATUS_REPORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_EKF_STATUS_REPORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ekf_status_report_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ekf_status_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ekf_status_report_pack(system_id, component_id, &msg , packet1.flags , packet1.velocity_variance , packet1.pos_horiz_variance , packet1.pos_vert_variance , packet1.compass_variance , packet1.terrain_alt_variance , packet1.airspeed_variance );
    mavlink_msg_ekf_status_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ekf_status_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.flags , packet1.velocity_variance , packet1.pos_horiz_variance , packet1.pos_vert_variance , packet1.compass_variance , packet1.terrain_alt_variance , packet1.airspeed_variance );
    mavlink_msg_ekf_status_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ekf_status_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ekf_status_report_send(MAVLINK_COMM_1 , packet1.flags , packet1.velocity_variance , packet1.pos_horiz_variance , packet1.pos_vert_variance , packet1.compass_variance , packet1.terrain_alt_variance , packet1.airspeed_variance );
    mavlink_msg_ekf_status_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("EKF_STATUS_REPORT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_EKF_STATUS_REPORT) != NULL);
#endif
}

static void mavlink_test_pid_tuning(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PID_TUNING >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_pid_tuning_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,77,192.0,220.0
    };
    mavlink_pid_tuning_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.desired = packet_in.desired;
        packet1.achieved = packet_in.achieved;
        packet1.FF = packet_in.FF;
        packet1.P = packet_in.P;
        packet1.I = packet_in.I;
        packet1.D = packet_in.D;
        packet1.axis = packet_in.axis;
        packet1.SRate = packet_in.SRate;
        packet1.PDmod = packet_in.PDmod;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PID_TUNING_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PID_TUNING_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pid_tuning_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_pid_tuning_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pid_tuning_pack(system_id, component_id, &msg , packet1.axis , packet1.desired , packet1.achieved , packet1.FF , packet1.P , packet1.I , packet1.D , packet1.SRate , packet1.PDmod );
    mavlink_msg_pid_tuning_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pid_tuning_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.axis , packet1.desired , packet1.achieved , packet1.FF , packet1.P , packet1.I , packet1.D , packet1.SRate , packet1.PDmod );
    mavlink_msg_pid_tuning_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_pid_tuning_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pid_tuning_send(MAVLINK_COMM_1 , packet1.axis , packet1.desired , packet1.achieved , packet1.FF , packet1.P , packet1.I , packet1.D , packet1.SRate , packet1.PDmod );
    mavlink_msg_pid_tuning_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PID_TUNING") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PID_TUNING) != NULL);
#endif
}

static void mavlink_test_deepstall(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEEPSTALL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_deepstall_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,963498504,185.0,213.0,241.0,113
    };
    mavlink_deepstall_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.landing_lat = packet_in.landing_lat;
        packet1.landing_lon = packet_in.landing_lon;
        packet1.path_lat = packet_in.path_lat;
        packet1.path_lon = packet_in.path_lon;
        packet1.arc_entry_lat = packet_in.arc_entry_lat;
        packet1.arc_entry_lon = packet_in.arc_entry_lon;
        packet1.altitude = packet_in.altitude;
        packet1.expected_travel_distance = packet_in.expected_travel_distance;
        packet1.cross_track_error = packet_in.cross_track_error;
        packet1.stage = packet_in.stage;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEEPSTALL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_deepstall_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_deepstall_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_deepstall_pack(system_id, component_id, &msg , packet1.landing_lat , packet1.landing_lon , packet1.path_lat , packet1.path_lon , packet1.arc_entry_lat , packet1.arc_entry_lon , packet1.altitude , packet1.expected_travel_distance , packet1.cross_track_error , packet1.stage );
    mavlink_msg_deepstall_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_deepstall_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.landing_lat , packet1.landing_lon , packet1.path_lat , packet1.path_lon , packet1.arc_entry_lat , packet1.arc_entry_lon , packet1.altitude , packet1.expected_travel_distance , packet1.cross_track_error , packet1.stage );
    mavlink_msg_deepstall_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_deepstall_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_deepstall_send(MAVLINK_COMM_1 , packet1.landing_lat , packet1.landing_lon , packet1.path_lat , packet1.path_lon , packet1.arc_entry_lat , packet1.arc_entry_lon , packet1.altitude , packet1.expected_travel_distance , packet1.cross_track_error , packet1.stage );
    mavlink_msg_deepstall_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEEPSTALL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEEPSTALL) != NULL);
#endif
}

static void mavlink_test_gimbal_report(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_REPORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_report_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,125,192
    };
    mavlink_gimbal_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.delta_time = packet_in.delta_time;
        packet1.delta_angle_x = packet_in.delta_angle_x;
        packet1.delta_angle_y = packet_in.delta_angle_y;
        packet1.delta_angle_z = packet_in.delta_angle_z;
        packet1.delta_velocity_x = packet_in.delta_velocity_x;
        packet1.delta_velocity_y = packet_in.delta_velocity_y;
        packet1.delta_velocity_z = packet_in.delta_velocity_z;
        packet1.joint_roll = packet_in.joint_roll;
        packet1.joint_el = packet_in.joint_el;
        packet1.joint_az = packet_in.joint_az;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_REPORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_report_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_report_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.delta_time , packet1.delta_angle_x , packet1.delta_angle_y , packet1.delta_angle_z , packet1.delta_velocity_x , packet1.delta_velocity_y , packet1.delta_velocity_z , packet1.joint_roll , packet1.joint_el , packet1.joint_az );
    mavlink_msg_gimbal_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.delta_time , packet1.delta_angle_x , packet1.delta_angle_y , packet1.delta_angle_z , packet1.delta_velocity_x , packet1.delta_velocity_y , packet1.delta_velocity_z , packet1.joint_roll , packet1.joint_el , packet1.joint_az );
    mavlink_msg_gimbal_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_report_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.delta_time , packet1.delta_angle_x , packet1.delta_angle_y , packet1.delta_angle_z , packet1.delta_velocity_x , packet1.delta_velocity_y , packet1.delta_velocity_z , packet1.joint_roll , packet1.joint_el , packet1.joint_az );
    mavlink_msg_gimbal_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_REPORT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_REPORT) != NULL);
#endif
}

static void mavlink_test_gimbal_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_control_t packet_in = {
        17.0,45.0,73.0,41,108
    };
    mavlink_gimbal_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.demanded_rate_x = packet_in.demanded_rate_x;
        packet1.demanded_rate_y = packet_in.demanded_rate_y;
        packet1.demanded_rate_z = packet_in.demanded_rate_z;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.demanded_rate_x , packet1.demanded_rate_y , packet1.demanded_rate_z );
    mavlink_msg_gimbal_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.demanded_rate_x , packet1.demanded_rate_y , packet1.demanded_rate_z );
    mavlink_msg_gimbal_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.demanded_rate_x , packet1.demanded_rate_y , packet1.demanded_rate_z );
    mavlink_msg_gimbal_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_CONTROL) != NULL);
#endif
}

static void mavlink_test_gimbal_torque_cmd_report(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gimbal_torque_cmd_report_t packet_in = {
        17235,17339,17443,151,218
    };
    mavlink_gimbal_torque_cmd_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rl_torque_cmd = packet_in.rl_torque_cmd;
        packet1.el_torque_cmd = packet_in.el_torque_cmd;
        packet1.az_torque_cmd = packet_in.az_torque_cmd;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_torque_cmd_report_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gimbal_torque_cmd_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_torque_cmd_report_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.rl_torque_cmd , packet1.el_torque_cmd , packet1.az_torque_cmd );
    mavlink_msg_gimbal_torque_cmd_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_torque_cmd_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.rl_torque_cmd , packet1.el_torque_cmd , packet1.az_torque_cmd );
    mavlink_msg_gimbal_torque_cmd_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gimbal_torque_cmd_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gimbal_torque_cmd_report_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.rl_torque_cmd , packet1.el_torque_cmd , packet1.az_torque_cmd );
    mavlink_msg_gimbal_torque_cmd_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GIMBAL_TORQUE_CMD_REPORT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GIMBAL_TORQUE_CMD_REPORT) != NULL);
#endif
}

static void mavlink_test_gopro_heartbeat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GOPRO_HEARTBEAT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gopro_heartbeat_t packet_in = {
        5,72,139
    };
    mavlink_gopro_heartbeat_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.status = packet_in.status;
        packet1.capture_mode = packet_in.capture_mode;
        packet1.flags = packet_in.flags;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GOPRO_HEARTBEAT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_heartbeat_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gopro_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_heartbeat_pack(system_id, component_id, &msg , packet1.status , packet1.capture_mode , packet1.flags );
    mavlink_msg_gopro_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_heartbeat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status , packet1.capture_mode , packet1.flags );
    mavlink_msg_gopro_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gopro_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_heartbeat_send(MAVLINK_COMM_1 , packet1.status , packet1.capture_mode , packet1.flags );
    mavlink_msg_gopro_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GOPRO_HEARTBEAT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GOPRO_HEARTBEAT) != NULL);
#endif
}

static void mavlink_test_gopro_get_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GOPRO_GET_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gopro_get_request_t packet_in = {
        5,72,139
    };
    mavlink_gopro_get_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.cmd_id = packet_in.cmd_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GOPRO_GET_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_get_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gopro_get_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_get_request_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.cmd_id );
    mavlink_msg_gopro_get_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_get_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.cmd_id );
    mavlink_msg_gopro_get_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gopro_get_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_get_request_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.cmd_id );
    mavlink_msg_gopro_get_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GOPRO_GET_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GOPRO_GET_REQUEST) != NULL);
#endif
}

static void mavlink_test_gopro_get_response(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GOPRO_GET_RESPONSE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gopro_get_response_t packet_in = {
        5,72,{ 139, 140, 141, 142 }
    };
    mavlink_gopro_get_response_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.cmd_id = packet_in.cmd_id;
        packet1.status = packet_in.status;
        
        mav_array_memcpy(packet1.value, packet_in.value, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GOPRO_GET_RESPONSE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GOPRO_GET_RESPONSE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_get_response_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gopro_get_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_get_response_pack(system_id, component_id, &msg , packet1.cmd_id , packet1.status , packet1.value );
    mavlink_msg_gopro_get_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_get_response_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cmd_id , packet1.status , packet1.value );
    mavlink_msg_gopro_get_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gopro_get_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_get_response_send(MAVLINK_COMM_1 , packet1.cmd_id , packet1.status , packet1.value );
    mavlink_msg_gopro_get_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GOPRO_GET_RESPONSE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GOPRO_GET_RESPONSE) != NULL);
#endif
}

static void mavlink_test_gopro_set_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GOPRO_SET_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gopro_set_request_t packet_in = {
        5,72,139,{ 206, 207, 208, 209 }
    };
    mavlink_gopro_set_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.cmd_id = packet_in.cmd_id;
        
        mav_array_memcpy(packet1.value, packet_in.value, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GOPRO_SET_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GOPRO_SET_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_set_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gopro_set_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_set_request_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.cmd_id , packet1.value );
    mavlink_msg_gopro_set_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_set_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.cmd_id , packet1.value );
    mavlink_msg_gopro_set_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gopro_set_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_set_request_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.cmd_id , packet1.value );
    mavlink_msg_gopro_set_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GOPRO_SET_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GOPRO_SET_REQUEST) != NULL);
#endif
}

static void mavlink_test_gopro_set_response(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GOPRO_SET_RESPONSE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gopro_set_response_t packet_in = {
        5,72
    };
    mavlink_gopro_set_response_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.cmd_id = packet_in.cmd_id;
        packet1.status = packet_in.status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GOPRO_SET_RESPONSE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GOPRO_SET_RESPONSE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_set_response_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gopro_set_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_set_response_pack(system_id, component_id, &msg , packet1.cmd_id , packet1.status );
    mavlink_msg_gopro_set_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_set_response_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cmd_id , packet1.status );
    mavlink_msg_gopro_set_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gopro_set_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gopro_set_response_send(MAVLINK_COMM_1 , packet1.cmd_id , packet1.status );
    mavlink_msg_gopro_set_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GOPRO_SET_RESPONSE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GOPRO_SET_RESPONSE) != NULL);
#endif
}

static void mavlink_test_rpm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RPM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rpm_t packet_in = {
        17.0,45.0
    };
    mavlink_rpm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.rpm1 = packet_in.rpm1;
        packet1.rpm2 = packet_in.rpm2;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RPM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RPM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rpm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rpm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rpm_pack(system_id, component_id, &msg , packet1.rpm1 , packet1.rpm2 );
    mavlink_msg_rpm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rpm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rpm1 , packet1.rpm2 );
    mavlink_msg_rpm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rpm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rpm_send(MAVLINK_COMM_1 , packet1.rpm1 , packet1.rpm2 );
    mavlink_msg_rpm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RPM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RPM) != NULL);
#endif
}

static void mavlink_test_device_op_read(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEVICE_OP_READ >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_device_op_read_t packet_in = {
        963497464,17,84,151,218,29,"JKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUV",216,27,94
    };
    mavlink_device_op_read_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.request_id = packet_in.request_id;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.bustype = packet_in.bustype;
        packet1.bus = packet_in.bus;
        packet1.address = packet_in.address;
        packet1.regstart = packet_in.regstart;
        packet1.count = packet_in.count;
        packet1.bank = packet_in.bank;
        
        mav_array_memcpy(packet1.busname, packet_in.busname, sizeof(char)*40);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_read_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_device_op_read_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_read_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.request_id , packet1.bustype , packet1.bus , packet1.address , packet1.busname , packet1.regstart , packet1.count , packet1.bank );
    mavlink_msg_device_op_read_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_read_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.request_id , packet1.bustype , packet1.bus , packet1.address , packet1.busname , packet1.regstart , packet1.count , packet1.bank );
    mavlink_msg_device_op_read_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_device_op_read_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_read_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.request_id , packet1.bustype , packet1.bus , packet1.address , packet1.busname , packet1.regstart , packet1.count , packet1.bank );
    mavlink_msg_device_op_read_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEVICE_OP_READ") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEVICE_OP_READ) != NULL);
#endif
}

static void mavlink_test_device_op_read_reply(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_device_op_read_reply_t packet_in = {
        963497464,17,84,151,{ 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89 },90
    };
    mavlink_device_op_read_reply_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.request_id = packet_in.request_id;
        packet1.result = packet_in.result;
        packet1.regstart = packet_in.regstart;
        packet1.count = packet_in.count;
        packet1.bank = packet_in.bank;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*128);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_read_reply_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_device_op_read_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_read_reply_pack(system_id, component_id, &msg , packet1.request_id , packet1.result , packet1.regstart , packet1.count , packet1.data , packet1.bank );
    mavlink_msg_device_op_read_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_read_reply_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.request_id , packet1.result , packet1.regstart , packet1.count , packet1.data , packet1.bank );
    mavlink_msg_device_op_read_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_device_op_read_reply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_read_reply_send(MAVLINK_COMM_1 , packet1.request_id , packet1.result , packet1.regstart , packet1.count , packet1.data , packet1.bank );
    mavlink_msg_device_op_read_reply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEVICE_OP_READ_REPLY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY) != NULL);
#endif
}

static void mavlink_test_device_op_write(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEVICE_OP_WRITE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_device_op_write_t packet_in = {
        963497464,17,84,151,218,29,"JKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUV",216,27,{ 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221 },222
    };
    mavlink_device_op_write_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.request_id = packet_in.request_id;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.bustype = packet_in.bustype;
        packet1.bus = packet_in.bus;
        packet1.address = packet_in.address;
        packet1.regstart = packet_in.regstart;
        packet1.count = packet_in.count;
        packet1.bank = packet_in.bank;
        
        mav_array_memcpy(packet1.busname, packet_in.busname, sizeof(char)*40);
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*128);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEVICE_OP_WRITE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEVICE_OP_WRITE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_write_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_device_op_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_write_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.request_id , packet1.bustype , packet1.bus , packet1.address , packet1.busname , packet1.regstart , packet1.count , packet1.data , packet1.bank );
    mavlink_msg_device_op_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_write_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.request_id , packet1.bustype , packet1.bus , packet1.address , packet1.busname , packet1.regstart , packet1.count , packet1.data , packet1.bank );
    mavlink_msg_device_op_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_device_op_write_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_write_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.request_id , packet1.bustype , packet1.bus , packet1.address , packet1.busname , packet1.regstart , packet1.count , packet1.data , packet1.bank );
    mavlink_msg_device_op_write_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEVICE_OP_WRITE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEVICE_OP_WRITE) != NULL);
#endif
}

static void mavlink_test_device_op_write_reply(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DEVICE_OP_WRITE_REPLY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_device_op_write_reply_t packet_in = {
        963497464,17
    };
    mavlink_device_op_write_reply_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.request_id = packet_in.request_id;
        packet1.result = packet_in.result;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DEVICE_OP_WRITE_REPLY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DEVICE_OP_WRITE_REPLY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_write_reply_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_device_op_write_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_write_reply_pack(system_id, component_id, &msg , packet1.request_id , packet1.result );
    mavlink_msg_device_op_write_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_write_reply_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.request_id , packet1.result );
    mavlink_msg_device_op_write_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_device_op_write_reply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_device_op_write_reply_send(MAVLINK_COMM_1 , packet1.request_id , packet1.result );
    mavlink_msg_device_op_write_reply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DEVICE_OP_WRITE_REPLY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DEVICE_OP_WRITE_REPLY) != NULL);
#endif
}

static void mavlink_test_secure_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SECURE_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_secure_command_t packet_in = {
        963497464,963497672,29,96,163,230,{ 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4 }
    };
    mavlink_secure_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sequence = packet_in.sequence;
        packet1.operation = packet_in.operation;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.data_length = packet_in.data_length;
        packet1.sig_length = packet_in.sig_length;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*220);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SECURE_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_secure_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_secure_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_secure_command_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.sequence , packet1.operation , packet1.data_length , packet1.sig_length , packet1.data );
    mavlink_msg_secure_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_secure_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.sequence , packet1.operation , packet1.data_length , packet1.sig_length , packet1.data );
    mavlink_msg_secure_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_secure_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_secure_command_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.sequence , packet1.operation , packet1.data_length , packet1.sig_length , packet1.data );
    mavlink_msg_secure_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SECURE_COMMAND") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SECURE_COMMAND) != NULL);
#endif
}

static void mavlink_test_secure_command_reply(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SECURE_COMMAND_REPLY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_secure_command_reply_t packet_in = {
        963497464,963497672,29,96,{ 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126 }
    };
    mavlink_secure_command_reply_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sequence = packet_in.sequence;
        packet1.operation = packet_in.operation;
        packet1.result = packet_in.result;
        packet1.data_length = packet_in.data_length;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*220);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SECURE_COMMAND_REPLY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_secure_command_reply_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_secure_command_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_secure_command_reply_pack(system_id, component_id, &msg , packet1.sequence , packet1.operation , packet1.result , packet1.data_length , packet1.data );
    mavlink_msg_secure_command_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_secure_command_reply_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sequence , packet1.operation , packet1.result , packet1.data_length , packet1.data );
    mavlink_msg_secure_command_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_secure_command_reply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_secure_command_reply_send(MAVLINK_COMM_1 , packet1.sequence , packet1.operation , packet1.result , packet1.data_length , packet1.data );
    mavlink_msg_secure_command_reply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SECURE_COMMAND_REPLY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SECURE_COMMAND_REPLY) != NULL);
#endif
}

static void mavlink_test_adap_tuning(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADAP_TUNING >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_adap_tuning_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,149
    };
    mavlink_adap_tuning_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.desired = packet_in.desired;
        packet1.achieved = packet_in.achieved;
        packet1.error = packet_in.error;
        packet1.theta = packet_in.theta;
        packet1.omega = packet_in.omega;
        packet1.sigma = packet_in.sigma;
        packet1.theta_dot = packet_in.theta_dot;
        packet1.omega_dot = packet_in.omega_dot;
        packet1.sigma_dot = packet_in.sigma_dot;
        packet1.f = packet_in.f;
        packet1.f_dot = packet_in.f_dot;
        packet1.u = packet_in.u;
        packet1.axis = packet_in.axis;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADAP_TUNING_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADAP_TUNING_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adap_tuning_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_adap_tuning_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adap_tuning_pack(system_id, component_id, &msg , packet1.axis , packet1.desired , packet1.achieved , packet1.error , packet1.theta , packet1.omega , packet1.sigma , packet1.theta_dot , packet1.omega_dot , packet1.sigma_dot , packet1.f , packet1.f_dot , packet1.u );
    mavlink_msg_adap_tuning_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adap_tuning_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.axis , packet1.desired , packet1.achieved , packet1.error , packet1.theta , packet1.omega , packet1.sigma , packet1.theta_dot , packet1.omega_dot , packet1.sigma_dot , packet1.f , packet1.f_dot , packet1.u );
    mavlink_msg_adap_tuning_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_adap_tuning_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adap_tuning_send(MAVLINK_COMM_1 , packet1.axis , packet1.desired , packet1.achieved , packet1.error , packet1.theta , packet1.omega , packet1.sigma , packet1.theta_dot , packet1.omega_dot , packet1.sigma_dot , packet1.f , packet1.f_dot , packet1.u );
    mavlink_msg_adap_tuning_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ADAP_TUNING") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ADAP_TUNING) != NULL);
#endif
}

static void mavlink_test_vision_position_delta(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VISION_POSITION_DELTA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_vision_position_delta_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,{ 129.0, 130.0, 131.0 },{ 213.0, 214.0, 215.0 },297.0
    };
    mavlink_vision_position_delta_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.time_delta_usec = packet_in.time_delta_usec;
        packet1.confidence = packet_in.confidence;
        
        mav_array_memcpy(packet1.angle_delta, packet_in.angle_delta, sizeof(float)*3);
        mav_array_memcpy(packet1.position_delta, packet_in.position_delta, sizeof(float)*3);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VISION_POSITION_DELTA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_position_delta_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_vision_position_delta_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_position_delta_pack(system_id, component_id, &msg , packet1.time_usec , packet1.time_delta_usec , packet1.angle_delta , packet1.position_delta , packet1.confidence );
    mavlink_msg_vision_position_delta_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_position_delta_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.time_delta_usec , packet1.angle_delta , packet1.position_delta , packet1.confidence );
    mavlink_msg_vision_position_delta_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_vision_position_delta_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_vision_position_delta_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.time_delta_usec , packet1.angle_delta , packet1.position_delta , packet1.confidence );
    mavlink_msg_vision_position_delta_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("VISION_POSITION_DELTA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_VISION_POSITION_DELTA) != NULL);
#endif
}

static void mavlink_test_aoa_ssa(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AOA_SSA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_aoa_ssa_t packet_in = {
        93372036854775807ULL,73.0,101.0
    };
    mavlink_aoa_ssa_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_usec = packet_in.time_usec;
        packet1.AOA = packet_in.AOA;
        packet1.SSA = packet_in.SSA;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AOA_SSA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AOA_SSA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aoa_ssa_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_aoa_ssa_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aoa_ssa_pack(system_id, component_id, &msg , packet1.time_usec , packet1.AOA , packet1.SSA );
    mavlink_msg_aoa_ssa_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aoa_ssa_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.AOA , packet1.SSA );
    mavlink_msg_aoa_ssa_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_aoa_ssa_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aoa_ssa_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.AOA , packet1.SSA );
    mavlink_msg_aoa_ssa_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AOA_SSA") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AOA_SSA) != NULL);
#endif
}

static void mavlink_test_esc_telemetry_1_to_4(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_esc_telemetry_1_to_4_t packet_in = {
        { 17235, 17236, 17237, 17238 },{ 17651, 17652, 17653, 17654 },{ 18067, 18068, 18069, 18070 },{ 18483, 18484, 18485, 18486 },{ 18899, 18900, 18901, 18902 },{ 125, 126, 127, 128 }
    };
    mavlink_esc_telemetry_1_to_4_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.voltage, packet_in.voltage, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.current, packet_in.current, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.totalcurrent, packet_in.totalcurrent, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.rpm, packet_in.rpm, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.count, packet_in.count, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.temperature, packet_in.temperature, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_1_to_4_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_esc_telemetry_1_to_4_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_1_to_4_pack(system_id, component_id, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_1_to_4_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_1_to_4_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_1_to_4_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_esc_telemetry_1_to_4_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_1_to_4_send(MAVLINK_COMM_1 , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_1_to_4_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ESC_TELEMETRY_1_TO_4") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4) != NULL);
#endif
}

static void mavlink_test_esc_telemetry_5_to_8(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_esc_telemetry_5_to_8_t packet_in = {
        { 17235, 17236, 17237, 17238 },{ 17651, 17652, 17653, 17654 },{ 18067, 18068, 18069, 18070 },{ 18483, 18484, 18485, 18486 },{ 18899, 18900, 18901, 18902 },{ 125, 126, 127, 128 }
    };
    mavlink_esc_telemetry_5_to_8_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.voltage, packet_in.voltage, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.current, packet_in.current, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.totalcurrent, packet_in.totalcurrent, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.rpm, packet_in.rpm, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.count, packet_in.count, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.temperature, packet_in.temperature, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_5_to_8_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_esc_telemetry_5_to_8_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_5_to_8_pack(system_id, component_id, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_5_to_8_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_5_to_8_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_5_to_8_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_esc_telemetry_5_to_8_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_5_to_8_send(MAVLINK_COMM_1 , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_5_to_8_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ESC_TELEMETRY_5_TO_8") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8) != NULL);
#endif
}

static void mavlink_test_esc_telemetry_9_to_12(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_esc_telemetry_9_to_12_t packet_in = {
        { 17235, 17236, 17237, 17238 },{ 17651, 17652, 17653, 17654 },{ 18067, 18068, 18069, 18070 },{ 18483, 18484, 18485, 18486 },{ 18899, 18900, 18901, 18902 },{ 125, 126, 127, 128 }
    };
    mavlink_esc_telemetry_9_to_12_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.voltage, packet_in.voltage, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.current, packet_in.current, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.totalcurrent, packet_in.totalcurrent, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.rpm, packet_in.rpm, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.count, packet_in.count, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.temperature, packet_in.temperature, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_9_to_12_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_esc_telemetry_9_to_12_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_9_to_12_pack(system_id, component_id, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_9_to_12_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_9_to_12_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_9_to_12_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_esc_telemetry_9_to_12_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_9_to_12_send(MAVLINK_COMM_1 , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_9_to_12_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ESC_TELEMETRY_9_TO_12") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12) != NULL);
#endif
}

static void mavlink_test_osd_param_config(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OSD_PARAM_CONFIG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_osd_param_config_t packet_in = {
        963497464,45.0,73.0,101.0,53,120,187,254,"UVWXYZABCDEFGHI",113
    };
    mavlink_osd_param_config_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.request_id = packet_in.request_id;
        packet1.min_value = packet_in.min_value;
        packet1.max_value = packet_in.max_value;
        packet1.increment = packet_in.increment;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.osd_screen = packet_in.osd_screen;
        packet1.osd_index = packet_in.osd_index;
        packet1.config_type = packet_in.config_type;
        
        mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OSD_PARAM_CONFIG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_config_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_osd_param_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_config_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.request_id , packet1.osd_screen , packet1.osd_index , packet1.param_id , packet1.config_type , packet1.min_value , packet1.max_value , packet1.increment );
    mavlink_msg_osd_param_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_config_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.request_id , packet1.osd_screen , packet1.osd_index , packet1.param_id , packet1.config_type , packet1.min_value , packet1.max_value , packet1.increment );
    mavlink_msg_osd_param_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_osd_param_config_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_config_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.request_id , packet1.osd_screen , packet1.osd_index , packet1.param_id , packet1.config_type , packet1.min_value , packet1.max_value , packet1.increment );
    mavlink_msg_osd_param_config_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OSD_PARAM_CONFIG") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OSD_PARAM_CONFIG) != NULL);
#endif
}

static void mavlink_test_osd_param_config_reply(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_osd_param_config_reply_t packet_in = {
        963497464,17
    };
    mavlink_osd_param_config_reply_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.request_id = packet_in.request_id;
        packet1.result = packet_in.result;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_config_reply_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_osd_param_config_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_config_reply_pack(system_id, component_id, &msg , packet1.request_id , packet1.result );
    mavlink_msg_osd_param_config_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_config_reply_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.request_id , packet1.result );
    mavlink_msg_osd_param_config_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_osd_param_config_reply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_config_reply_send(MAVLINK_COMM_1 , packet1.request_id , packet1.result );
    mavlink_msg_osd_param_config_reply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OSD_PARAM_CONFIG_REPLY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OSD_PARAM_CONFIG_REPLY) != NULL);
#endif
}

static void mavlink_test_osd_param_show_config(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_osd_param_show_config_t packet_in = {
        963497464,17,84,151,218
    };
    mavlink_osd_param_show_config_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.request_id = packet_in.request_id;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.osd_screen = packet_in.osd_screen;
        packet1.osd_index = packet_in.osd_index;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_show_config_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_osd_param_show_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_show_config_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.request_id , packet1.osd_screen , packet1.osd_index );
    mavlink_msg_osd_param_show_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_show_config_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.request_id , packet1.osd_screen , packet1.osd_index );
    mavlink_msg_osd_param_show_config_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_osd_param_show_config_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_show_config_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.request_id , packet1.osd_screen , packet1.osd_index );
    mavlink_msg_osd_param_show_config_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OSD_PARAM_SHOW_CONFIG") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG) != NULL);
#endif
}

static void mavlink_test_osd_param_show_config_reply(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_osd_param_show_config_reply_t packet_in = {
        963497464,45.0,73.0,101.0,53,"RSTUVWXYZABCDEF",168
    };
    mavlink_osd_param_show_config_reply_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.request_id = packet_in.request_id;
        packet1.min_value = packet_in.min_value;
        packet1.max_value = packet_in.max_value;
        packet1.increment = packet_in.increment;
        packet1.result = packet_in.result;
        packet1.config_type = packet_in.config_type;
        
        mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_show_config_reply_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_osd_param_show_config_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_show_config_reply_pack(system_id, component_id, &msg , packet1.request_id , packet1.result , packet1.param_id , packet1.config_type , packet1.min_value , packet1.max_value , packet1.increment );
    mavlink_msg_osd_param_show_config_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_show_config_reply_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.request_id , packet1.result , packet1.param_id , packet1.config_type , packet1.min_value , packet1.max_value , packet1.increment );
    mavlink_msg_osd_param_show_config_reply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_osd_param_show_config_reply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_osd_param_show_config_reply_send(MAVLINK_COMM_1 , packet1.request_id , packet1.result , packet1.param_id , packet1.config_type , packet1.min_value , packet1.max_value , packet1.increment );
    mavlink_msg_osd_param_show_config_reply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OSD_PARAM_SHOW_CONFIG_REPLY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG_REPLY) != NULL);
#endif
}

static void mavlink_test_obstacle_distance_3d(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_obstacle_distance_3d_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,18483,211,22
    };
    mavlink_obstacle_distance_3d_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.min_distance = packet_in.min_distance;
        packet1.max_distance = packet_in.max_distance;
        packet1.obstacle_id = packet_in.obstacle_id;
        packet1.sensor_type = packet_in.sensor_type;
        packet1.frame = packet_in.frame;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_obstacle_distance_3d_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_obstacle_distance_3d_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_obstacle_distance_3d_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.sensor_type , packet1.frame , packet1.obstacle_id , packet1.x , packet1.y , packet1.z , packet1.min_distance , packet1.max_distance );
    mavlink_msg_obstacle_distance_3d_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_obstacle_distance_3d_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.sensor_type , packet1.frame , packet1.obstacle_id , packet1.x , packet1.y , packet1.z , packet1.min_distance , packet1.max_distance );
    mavlink_msg_obstacle_distance_3d_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_obstacle_distance_3d_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_obstacle_distance_3d_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.sensor_type , packet1.frame , packet1.obstacle_id , packet1.x , packet1.y , packet1.z , packet1.min_distance , packet1.max_distance );
    mavlink_msg_obstacle_distance_3d_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("OBSTACLE_DISTANCE_3D") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D) != NULL);
#endif
}

static void mavlink_test_water_depth(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WATER_DEPTH >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_water_depth_t packet_in = {
        963497464,963497672,963497880,101.0,129.0,157.0,185.0,213.0,241.0,113,180
    };
    mavlink_water_depth_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.lat = packet_in.lat;
        packet1.lng = packet_in.lng;
        packet1.alt = packet_in.alt;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.distance = packet_in.distance;
        packet1.temperature = packet_in.temperature;
        packet1.id = packet_in.id;
        packet1.healthy = packet_in.healthy;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WATER_DEPTH_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_water_depth_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_water_depth_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_water_depth_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.id , packet1.healthy , packet1.lat , packet1.lng , packet1.alt , packet1.roll , packet1.pitch , packet1.yaw , packet1.distance , packet1.temperature );
    mavlink_msg_water_depth_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_water_depth_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.id , packet1.healthy , packet1.lat , packet1.lng , packet1.alt , packet1.roll , packet1.pitch , packet1.yaw , packet1.distance , packet1.temperature );
    mavlink_msg_water_depth_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_water_depth_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_water_depth_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.id , packet1.healthy , packet1.lat , packet1.lng , packet1.alt , packet1.roll , packet1.pitch , packet1.yaw , packet1.distance , packet1.temperature );
    mavlink_msg_water_depth_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WATER_DEPTH") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WATER_DEPTH) != NULL);
#endif
}

static void mavlink_test_mcu_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MCU_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mcu_status_t packet_in = {
        17235,17339,17443,17547,29
    };
    mavlink_mcu_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.MCU_temperature = packet_in.MCU_temperature;
        packet1.MCU_voltage = packet_in.MCU_voltage;
        packet1.MCU_voltage_min = packet_in.MCU_voltage_min;
        packet1.MCU_voltage_max = packet_in.MCU_voltage_max;
        packet1.id = packet_in.id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MCU_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mcu_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mcu_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mcu_status_pack(system_id, component_id, &msg , packet1.id , packet1.MCU_temperature , packet1.MCU_voltage , packet1.MCU_voltage_min , packet1.MCU_voltage_max );
    mavlink_msg_mcu_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mcu_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.MCU_temperature , packet1.MCU_voltage , packet1.MCU_voltage_min , packet1.MCU_voltage_max );
    mavlink_msg_mcu_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mcu_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mcu_status_send(MAVLINK_COMM_1 , packet1.id , packet1.MCU_temperature , packet1.MCU_voltage , packet1.MCU_voltage_min , packet1.MCU_voltage_max );
    mavlink_msg_mcu_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MCU_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MCU_STATUS) != NULL);
#endif
}

static void mavlink_test_esc_telemetry_13_to_16(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ESC_TELEMETRY_13_TO_16 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_esc_telemetry_13_to_16_t packet_in = {
        { 17235, 17236, 17237, 17238 },{ 17651, 17652, 17653, 17654 },{ 18067, 18068, 18069, 18070 },{ 18483, 18484, 18485, 18486 },{ 18899, 18900, 18901, 18902 },{ 125, 126, 127, 128 }
    };
    mavlink_esc_telemetry_13_to_16_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.voltage, packet_in.voltage, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.current, packet_in.current, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.totalcurrent, packet_in.totalcurrent, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.rpm, packet_in.rpm, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.count, packet_in.count, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.temperature, packet_in.temperature, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ESC_TELEMETRY_13_TO_16_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ESC_TELEMETRY_13_TO_16_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_13_to_16_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_esc_telemetry_13_to_16_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_13_to_16_pack(system_id, component_id, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_13_to_16_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_13_to_16_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_13_to_16_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_esc_telemetry_13_to_16_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_13_to_16_send(MAVLINK_COMM_1 , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_13_to_16_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ESC_TELEMETRY_13_TO_16") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ESC_TELEMETRY_13_TO_16) != NULL);
#endif
}

static void mavlink_test_esc_telemetry_17_to_20(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ESC_TELEMETRY_17_TO_20 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_esc_telemetry_17_to_20_t packet_in = {
        { 17235, 17236, 17237, 17238 },{ 17651, 17652, 17653, 17654 },{ 18067, 18068, 18069, 18070 },{ 18483, 18484, 18485, 18486 },{ 18899, 18900, 18901, 18902 },{ 125, 126, 127, 128 }
    };
    mavlink_esc_telemetry_17_to_20_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.voltage, packet_in.voltage, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.current, packet_in.current, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.totalcurrent, packet_in.totalcurrent, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.rpm, packet_in.rpm, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.count, packet_in.count, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.temperature, packet_in.temperature, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ESC_TELEMETRY_17_TO_20_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ESC_TELEMETRY_17_TO_20_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_17_to_20_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_esc_telemetry_17_to_20_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_17_to_20_pack(system_id, component_id, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_17_to_20_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_17_to_20_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_17_to_20_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_esc_telemetry_17_to_20_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_17_to_20_send(MAVLINK_COMM_1 , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_17_to_20_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ESC_TELEMETRY_17_TO_20") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ESC_TELEMETRY_17_TO_20) != NULL);
#endif
}

static void mavlink_test_esc_telemetry_21_to_24(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_esc_telemetry_21_to_24_t packet_in = {
        { 17235, 17236, 17237, 17238 },{ 17651, 17652, 17653, 17654 },{ 18067, 18068, 18069, 18070 },{ 18483, 18484, 18485, 18486 },{ 18899, 18900, 18901, 18902 },{ 125, 126, 127, 128 }
    };
    mavlink_esc_telemetry_21_to_24_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.voltage, packet_in.voltage, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.current, packet_in.current, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.totalcurrent, packet_in.totalcurrent, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.rpm, packet_in.rpm, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.count, packet_in.count, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.temperature, packet_in.temperature, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_21_to_24_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_esc_telemetry_21_to_24_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_21_to_24_pack(system_id, component_id, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_21_to_24_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_21_to_24_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_21_to_24_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_esc_telemetry_21_to_24_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_21_to_24_send(MAVLINK_COMM_1 , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_21_to_24_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ESC_TELEMETRY_21_TO_24") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ESC_TELEMETRY_21_TO_24) != NULL);
#endif
}

static void mavlink_test_esc_telemetry_25_to_28(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ESC_TELEMETRY_25_TO_28 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_esc_telemetry_25_to_28_t packet_in = {
        { 17235, 17236, 17237, 17238 },{ 17651, 17652, 17653, 17654 },{ 18067, 18068, 18069, 18070 },{ 18483, 18484, 18485, 18486 },{ 18899, 18900, 18901, 18902 },{ 125, 126, 127, 128 }
    };
    mavlink_esc_telemetry_25_to_28_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.voltage, packet_in.voltage, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.current, packet_in.current, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.totalcurrent, packet_in.totalcurrent, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.rpm, packet_in.rpm, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.count, packet_in.count, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.temperature, packet_in.temperature, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ESC_TELEMETRY_25_TO_28_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ESC_TELEMETRY_25_TO_28_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_25_to_28_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_esc_telemetry_25_to_28_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_25_to_28_pack(system_id, component_id, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_25_to_28_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_25_to_28_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_25_to_28_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_esc_telemetry_25_to_28_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_25_to_28_send(MAVLINK_COMM_1 , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_25_to_28_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ESC_TELEMETRY_25_TO_28") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ESC_TELEMETRY_25_TO_28) != NULL);
#endif
}

static void mavlink_test_esc_telemetry_29_to_32(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ESC_TELEMETRY_29_TO_32 >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_esc_telemetry_29_to_32_t packet_in = {
        { 17235, 17236, 17237, 17238 },{ 17651, 17652, 17653, 17654 },{ 18067, 18068, 18069, 18070 },{ 18483, 18484, 18485, 18486 },{ 18899, 18900, 18901, 18902 },{ 125, 126, 127, 128 }
    };
    mavlink_esc_telemetry_29_to_32_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.voltage, packet_in.voltage, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.current, packet_in.current, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.totalcurrent, packet_in.totalcurrent, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.rpm, packet_in.rpm, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.count, packet_in.count, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.temperature, packet_in.temperature, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ESC_TELEMETRY_29_TO_32_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ESC_TELEMETRY_29_TO_32_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_29_to_32_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_esc_telemetry_29_to_32_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_29_to_32_pack(system_id, component_id, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_29_to_32_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_29_to_32_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_29_to_32_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_esc_telemetry_29_to_32_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_esc_telemetry_29_to_32_send(MAVLINK_COMM_1 , packet1.temperature , packet1.voltage , packet1.current , packet1.totalcurrent , packet1.rpm , packet1.count );
    mavlink_msg_esc_telemetry_29_to_32_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ESC_TELEMETRY_29_TO_32") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ESC_TELEMETRY_29_TO_32) != NULL);
#endif
}

static void mavlink_test_ardupilotmega(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_sensor_offsets(system_id, component_id, last_msg);
    mavlink_test_set_mag_offsets(system_id, component_id, last_msg);
    mavlink_test_meminfo(system_id, component_id, last_msg);
    mavlink_test_ap_adc(system_id, component_id, last_msg);
    mavlink_test_digicam_configure(system_id, component_id, last_msg);
    mavlink_test_digicam_control(system_id, component_id, last_msg);
    mavlink_test_mount_configure(system_id, component_id, last_msg);
    mavlink_test_mount_control(system_id, component_id, last_msg);
    mavlink_test_mount_status(system_id, component_id, last_msg);
    mavlink_test_fence_point(system_id, component_id, last_msg);
    mavlink_test_fence_fetch_point(system_id, component_id, last_msg);
    mavlink_test_ahrs(system_id, component_id, last_msg);
    mavlink_test_simstate(system_id, component_id, last_msg);
    mavlink_test_hwstatus(system_id, component_id, last_msg);
    mavlink_test_radio(system_id, component_id, last_msg);
    mavlink_test_limits_status(system_id, component_id, last_msg);
    mavlink_test_wind(system_id, component_id, last_msg);
    mavlink_test_data16(system_id, component_id, last_msg);
    mavlink_test_data32(system_id, component_id, last_msg);
    mavlink_test_data64(system_id, component_id, last_msg);
    mavlink_test_data96(system_id, component_id, last_msg);
    mavlink_test_rangefinder(system_id, component_id, last_msg);
    mavlink_test_airspeed_autocal(system_id, component_id, last_msg);
    mavlink_test_rally_point(system_id, component_id, last_msg);
    mavlink_test_rally_fetch_point(system_id, component_id, last_msg);
    mavlink_test_compassmot_status(system_id, component_id, last_msg);
    mavlink_test_ahrs2(system_id, component_id, last_msg);
    mavlink_test_camera_status(system_id, component_id, last_msg);
    mavlink_test_camera_feedback(system_id, component_id, last_msg);
    mavlink_test_battery2(system_id, component_id, last_msg);
    mavlink_test_ahrs3(system_id, component_id, last_msg);
    mavlink_test_autopilot_version_request(system_id, component_id, last_msg);
    mavlink_test_remote_log_data_block(system_id, component_id, last_msg);
    mavlink_test_remote_log_block_status(system_id, component_id, last_msg);
    mavlink_test_led_control(system_id, component_id, last_msg);
    mavlink_test_mag_cal_progress(system_id, component_id, last_msg);
    mavlink_test_ekf_status_report(system_id, component_id, last_msg);
    mavlink_test_pid_tuning(system_id, component_id, last_msg);
    mavlink_test_deepstall(system_id, component_id, last_msg);
    mavlink_test_gimbal_report(system_id, component_id, last_msg);
    mavlink_test_gimbal_control(system_id, component_id, last_msg);
    mavlink_test_gimbal_torque_cmd_report(system_id, component_id, last_msg);
    mavlink_test_gopro_heartbeat(system_id, component_id, last_msg);
    mavlink_test_gopro_get_request(system_id, component_id, last_msg);
    mavlink_test_gopro_get_response(system_id, component_id, last_msg);
    mavlink_test_gopro_set_request(system_id, component_id, last_msg);
    mavlink_test_gopro_set_response(system_id, component_id, last_msg);
    mavlink_test_rpm(system_id, component_id, last_msg);
    mavlink_test_device_op_read(system_id, component_id, last_msg);
    mavlink_test_device_op_read_reply(system_id, component_id, last_msg);
    mavlink_test_device_op_write(system_id, component_id, last_msg);
    mavlink_test_device_op_write_reply(system_id, component_id, last_msg);
    mavlink_test_secure_command(system_id, component_id, last_msg);
    mavlink_test_secure_command_reply(system_id, component_id, last_msg);
    mavlink_test_adap_tuning(system_id, component_id, last_msg);
    mavlink_test_vision_position_delta(system_id, component_id, last_msg);
    mavlink_test_aoa_ssa(system_id, component_id, last_msg);
    mavlink_test_esc_telemetry_1_to_4(system_id, component_id, last_msg);
    mavlink_test_esc_telemetry_5_to_8(system_id, component_id, last_msg);
    mavlink_test_esc_telemetry_9_to_12(system_id, component_id, last_msg);
    mavlink_test_osd_param_config(system_id, component_id, last_msg);
    mavlink_test_osd_param_config_reply(system_id, component_id, last_msg);
    mavlink_test_osd_param_show_config(system_id, component_id, last_msg);
    mavlink_test_osd_param_show_config_reply(system_id, component_id, last_msg);
    mavlink_test_obstacle_distance_3d(system_id, component_id, last_msg);
    mavlink_test_water_depth(system_id, component_id, last_msg);
    mavlink_test_mcu_status(system_id, component_id, last_msg);
    mavlink_test_esc_telemetry_13_to_16(system_id, component_id, last_msg);
    mavlink_test_esc_telemetry_17_to_20(system_id, component_id, last_msg);
    mavlink_test_esc_telemetry_21_to_24(system_id, component_id, last_msg);
    mavlink_test_esc_telemetry_25_to_28(system_id, component_id, last_msg);
    mavlink_test_esc_telemetry_29_to_32(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ARDUPILOTMEGA_TESTSUITE_H

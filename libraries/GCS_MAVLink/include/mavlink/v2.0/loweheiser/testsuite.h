/** @file
 *    @brief MAVLink comm protocol testsuite generated from loweheiser.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef LOWEHEISER_TESTSUITE_H
#define LOWEHEISER_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_minimal(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_loweheiser(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_minimal(system_id, component_id, last_msg);
    mavlink_test_loweheiser(system_id, component_id, last_msg);
}
#endif

#include "../minimal/testsuite.h"


static void mavlink_test_loweheiser_gov_efi(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_loweheiser_gov_efi_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,963498712,963498920,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,21395,21499,1
    };
    mavlink_loweheiser_gov_efi_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.volt_batt = packet_in.volt_batt;
        packet1.curr_batt = packet_in.curr_batt;
        packet1.curr_gen = packet_in.curr_gen;
        packet1.curr_rot = packet_in.curr_rot;
        packet1.fuel_level = packet_in.fuel_level;
        packet1.throttle = packet_in.throttle;
        packet1.runtime = packet_in.runtime;
        packet1.until_maintenance = packet_in.until_maintenance;
        packet1.rectifier_temp = packet_in.rectifier_temp;
        packet1.generator_temp = packet_in.generator_temp;
        packet1.efi_batt = packet_in.efi_batt;
        packet1.efi_rpm = packet_in.efi_rpm;
        packet1.efi_pw = packet_in.efi_pw;
        packet1.efi_fuel_flow = packet_in.efi_fuel_flow;
        packet1.efi_fuel_consumed = packet_in.efi_fuel_consumed;
        packet1.efi_baro = packet_in.efi_baro;
        packet1.efi_mat = packet_in.efi_mat;
        packet1.efi_clt = packet_in.efi_clt;
        packet1.efi_tps = packet_in.efi_tps;
        packet1.efi_exhaust_gas_temperature = packet_in.efi_exhaust_gas_temperature;
        packet1.generator_status = packet_in.generator_status;
        packet1.efi_status = packet_in.efi_status;
        packet1.efi_index = packet_in.efi_index;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_loweheiser_gov_efi_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_loweheiser_gov_efi_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_loweheiser_gov_efi_pack(system_id, component_id, &msg , packet1.volt_batt , packet1.curr_batt , packet1.curr_gen , packet1.curr_rot , packet1.fuel_level , packet1.throttle , packet1.runtime , packet1.until_maintenance , packet1.rectifier_temp , packet1.generator_temp , packet1.efi_batt , packet1.efi_rpm , packet1.efi_pw , packet1.efi_fuel_flow , packet1.efi_fuel_consumed , packet1.efi_baro , packet1.efi_mat , packet1.efi_clt , packet1.efi_tps , packet1.efi_exhaust_gas_temperature , packet1.efi_index , packet1.generator_status , packet1.efi_status );
    mavlink_msg_loweheiser_gov_efi_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_loweheiser_gov_efi_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.volt_batt , packet1.curr_batt , packet1.curr_gen , packet1.curr_rot , packet1.fuel_level , packet1.throttle , packet1.runtime , packet1.until_maintenance , packet1.rectifier_temp , packet1.generator_temp , packet1.efi_batt , packet1.efi_rpm , packet1.efi_pw , packet1.efi_fuel_flow , packet1.efi_fuel_consumed , packet1.efi_baro , packet1.efi_mat , packet1.efi_clt , packet1.efi_tps , packet1.efi_exhaust_gas_temperature , packet1.efi_index , packet1.generator_status , packet1.efi_status );
    mavlink_msg_loweheiser_gov_efi_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_loweheiser_gov_efi_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_loweheiser_gov_efi_send(MAVLINK_COMM_1 , packet1.volt_batt , packet1.curr_batt , packet1.curr_gen , packet1.curr_rot , packet1.fuel_level , packet1.throttle , packet1.runtime , packet1.until_maintenance , packet1.rectifier_temp , packet1.generator_temp , packet1.efi_batt , packet1.efi_rpm , packet1.efi_pw , packet1.efi_fuel_flow , packet1.efi_fuel_consumed , packet1.efi_baro , packet1.efi_mat , packet1.efi_clt , packet1.efi_tps , packet1.efi_exhaust_gas_temperature , packet1.efi_index , packet1.generator_status , packet1.efi_status );
    mavlink_msg_loweheiser_gov_efi_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOWEHEISER_GOV_EFI") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI) != NULL);
#endif
}

static void mavlink_test_loweheiser(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_loweheiser_gov_efi(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // LOWEHEISER_TESTSUITE_H

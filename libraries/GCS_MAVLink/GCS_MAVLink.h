// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.

#ifndef GCS_MAVLink_h
#define GCS_MAVLink_h

#include <AP_HAL.h>
#include <AP_Param.h>

// we have separate helpers disabled to make it possible
// to select MAVLink 1.0 in the arduino GUI build
#define MAVLINK_SEPARATE_HELPERS

#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

// define our own MAVLINK_MESSAGE_CRC() macro to allow it to be put
// into progmem
#define MAVLINK_MESSAGE_CRC(msgid) mavlink_get_message_crc(msgid)

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
#include <util/crc16.h>
#define HAVE_CRC_ACCUMULATE
#endif

#include "include/mavlink/v1.0/ardupilotmega/version.h"

// this allows us to make mavlink_message_t much smaller. It means we
// can't support the largest messages in common.xml, but we don't need
// those for APM
#define MAVLINK_MAX_PAYLOAD_LEN 96

#define MAVLINK_COMM_NUM_BUFFERS 2
#include "include/mavlink/v1.0/mavlink_types.h"

/// MAVLink stream used for HIL interaction
extern AP_HAL::BetterStream	*mavlink_comm_0_port;

/// MAVLink stream used for ground control communication
extern AP_HAL::BetterStream	*mavlink_comm_1_port;

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

/// Send a byte to the nominated MAVLink channel
///
/// @param chan		Channel to send to
/// @param ch		Byte to send
///
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    switch(chan) {
	case MAVLINK_COMM_0:
		mavlink_comm_0_port->write(ch);
		break;
	case MAVLINK_COMM_1:
		mavlink_comm_1_port->write(ch);
		break;
	default:
		break;
	}
}

void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

/// Read a byte from the nominated MAVLink channel
///
/// @param chan		Channel to receive on
/// @returns		Byte read
///

#define COOPERATIVE 1

#if COOPERATIVE
#define FOLLOWER_WP_CRC 0xAA
#define ORBIT_CENTER_CRC 0xBB

static    uint8_t o_packet[14] = {0};//temp
static    uint8_t W_packet[13] = {0}; //temp

static    uint8_t flag1 = 0;//flags for interrupt handlers
static    uint8_t flag2 = 0;//flags for interrupt handlers
static    uint8_t ctr = 0;//flags for interrupt handlers

static    uint8_t ourpacket[13] = {0};
static    uint8_t WPpacket[12] = {0};  //IMPORTANT: Stores the most recent correct interaircraft data
static    uint8_t PWPpacket[12] = {0}; //IMPORTANT: Stores the previous most recent correct interaircraft data

static    uint8_t xbee_flag = 0;

static    uint8_t evnt_WPpacket[12] = {0};  //IMPORTANT: Stores the most recent correct interaircraft data for orbit center
static    uint8_t evnt_PWPpacket[12] = {0}; //IMPORTANT: Stores the previous most recent correct interaircraft data for orbit center


	/* Function calculates the interaircraft data crc and checks it against
	// the one in the packet. Then appropriately stores the data.
	*/
static inline void check_recieved_packet(void)
{
uint16_t data_sum=0;
int i;
for(i=0;i<12;i++){
	data_sum = data_sum + W_packet[i];
	}
if(W_packet[12] == ((0xFF - data_sum + FOLLOWER_WP_CRC) & 0x000000FF)){
	for(int k = 0; k< 12; k++){ 
        WPpacket[k] = W_packet[k]; 
        }
	
	}
if(W_packet[12] == ((0xFF - data_sum + ORBIT_CENTER_CRC) & 0x000000FF)){
	for(int k = 0; k< 12; k++){ 
        evnt_WPpacket[k] = W_packet[k]; 
        }
	}

}

#endif
	/* Modified it to look for interaircraft xbee data start tag 0xAF 0xFB
	// and when found, store the next 13 bytes in a temp array, check for data 
	// crc using check_recieved_packet() and store the data prmanently 	
	*/
static inline uint8_t comm_receive_ch(mavlink_channel_t chan)
{
    uint8_t data = 0;

    switch(chan) {
	case MAVLINK_COMM_0:
		data = mavlink_comm_0_port->read();
		break;
	case MAVLINK_COMM_1:
		data = mavlink_comm_1_port->read();

#if COOPERATIVE
 		 if(data == 0xAF) { //changed from A0, B0 (make sure LDR Xbee.pde sends these packets only!)
                   flag1 =1;
                    }
         if(flag1 == 1 && data ==0xFB){
                   flag1=0;flag2=1;
                    }
         if(flag2==1 && ctr <14) { // since we have 12 actual non-api packets:: STUPID : old code would include B0 in ourpacket!
                   o_packet[ctr]=data;
                   ctr++;
                    }
         if (ctr == 14){
              ctr = 0;
              flag2 = 0;
              for(int k = 1; k< 14; k++){ //k=1 to skip including 0xb0
                      W_packet[k-1] = o_packet[k];  //BTAK 2nd edit to debug, replace with ourpacket     
                          }
              for (int p=0;p<14;p++){
                       o_packet[p] = 0;
                       }
              check_recieved_packet();
              }
#endif
		break;
	default:
		break;
	}
    return data;
}

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
static inline uint16_t comm_get_available(mavlink_channel_t chan)
{
    int16_t bytes = 0;
    switch(chan) {
	case MAVLINK_COMM_0:
		bytes = mavlink_comm_0_port->available();
		break;
	case MAVLINK_COMM_1:
		bytes = mavlink_comm_1_port->available();
		break;
	default:
		break;
	}
	if (bytes == -1) {
		return 0;
	}
    return (uint16_t)bytes;
}


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
static inline uint16_t comm_get_txspace(mavlink_channel_t chan)
{
	int16_t ret = 0;
    switch(chan) {
	case MAVLINK_COMM_0:
		ret = mavlink_comm_0_port->txspace();
		break;
	case MAVLINK_COMM_1:
		ret = mavlink_comm_1_port->txspace();
		break;
	default:
		break;
	}
	if (ret < 0) {
		ret = 0;
	}
    return (uint16_t)ret;
}

#ifdef HAVE_CRC_ACCUMULATE
// use the AVR C library implementation. This is a bit over twice as
// fast as the C version
static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
	*crcAccum = _crc_ccitt_update(*crcAccum, data);
}
#endif

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "include/mavlink/v1.0/ardupilotmega/mavlink.h"

uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid);

// return a MAVLink variable type given a AP_Param type
uint8_t mav_var_type(enum ap_var_type t);

// return CRC byte for a mavlink message ID
uint8_t mavlink_get_message_crc(uint8_t msgid);

#endif // GCS_MAVLink_h

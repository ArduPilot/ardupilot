#ifndef  _MAVLINK_PROTOCOL_H_
#define  _MAVLINK_PROTOCOL_H_

#include "string.h"
#include "checksum.h"

#include "mavlink_types.h"


/**
 * @brief Initialize the communication stack
 *
 * This function has to be called before using commParseBuffer() to initialize the different status registers.
 *
 * @return Will initialize the different buffers and status registers.
 */
static void mavlink_parse_state_initialize(mavlink_status_t* initStatus)
{
	if ((initStatus->parse_state <= MAVLINK_PARSE_STATE_UNINIT) || (initStatus->parse_state > MAVLINK_PARSE_STATE_GOT_CRC1))
	{
		initStatus->ck_a = 0;
		initStatus->ck_b = 0;
		initStatus->msg_received = 0;
		initStatus->buffer_overrun = 0;
		initStatus->parse_error = 0;
		initStatus->parse_state = MAVLINK_PARSE_STATE_UNINIT;
		initStatus->packet_idx = 0;
		initStatus->packet_rx_drop_count = 0;
		initStatus->packet_rx_success_count = 0;
		initStatus->current_rx_seq = 0;
		initStatus->current_tx_seq = 0;
	}
}

static inline mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
#if (defined linux) | (defined __linux) | (defined  __MACH__) | (defined _WIN32)
	static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NB_HIGH];
#else
	static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NB];
#endif

	return &m_mavlink_status[chan];
}

/**
 * @brief Finalize a MAVLink message with MAVLINK_COMM_0 as default channel
 *
 * This function calculates the checksum and sets length and aircraft id correctly.
 * It assumes that the message id and the payload are already correctly set. 
 *
 * @warning This function implicitely assumes the message is sent over channel zero.
 *          if the message is sent over a different channel it will reach the receiver
 *          without error, BUT the sequence number might be wrong due to the wrong
 *          channel sequence counter. This will result is wrongly reported excessive
 *          packet loss. Please use @see mavlink_{pack|encode}_headerless and then
 *          @see mavlink_finalize_message_chan before sending for a correct channel
 *          assignment. Please note that the mavlink_msg_xxx_pack and encode functions
 *          assign channel zero as default and thus induce possible loss counter errors.\
 *          They have been left to ensure code compatibility.
 *
 * @see mavlink_finalize_message_chan
 * @param msg Message to finalize
 * @param system_id Id of the sending (this) system, 1-127
 * @param length Message length, usually just the counter incremented while packing the message
 */
static inline uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, uint16_t length)
{
	// This code part is the same for all messages;
	uint16_t checksum;
	msg->len = length;
	msg->sysid = system_id;
	msg->compid = component_id;
	// One sequence number per component
	msg->seq = mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq;
        mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq = mavlink_get_channel_status(MAVLINK_COMM_0)->current_tx_seq+1;
	checksum = crc_calculate((uint8_t*)((void*)msg), length + MAVLINK_CORE_HEADER_LEN);
	msg->ck_a = (uint8_t)(checksum & 0xFF); ///< High byte
	msg->ck_b = (uint8_t)(checksum >> 8); ///< Low byte

	return length + MAVLINK_NUM_NON_STX_PAYLOAD_BYTES;
}

/**
 * @brief Finalize a MAVLink message with channel assignment
 *
 * This function calculates the checksum and sets length and aircraft id correctly.
 * It assumes that the message id and the payload are already correctly set. This function
 * can also be used if the message header has already been written before (as in mavlink_msg_xxx_pack
 * instead of mavlink_msg_xxx_pack_headerless), it just introduces little extra overhead.
 *
 * @param msg Message to finalize
 * @param system_id Id of the sending (this) system, 1-127
 * @param length Message length, usually just the counter incremented while packing the message
 */
static inline uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, uint8_t chan, uint16_t length)
{
	// This code part is the same for all messages;
	uint16_t checksum;
	msg->len = length;
	msg->sysid = system_id;
	msg->compid = component_id;
	// One sequence number per component
	msg->seq = mavlink_get_channel_status(chan)->current_tx_seq;
	mavlink_get_channel_status(chan)->current_tx_seq = mavlink_get_channel_status(chan)->current_tx_seq+1;
	checksum = crc_calculate((uint8_t*)((void*)msg), length + MAVLINK_CORE_HEADER_LEN);
	msg->ck_a = (uint8_t)(checksum & 0xFF); ///< High byte
	msg->ck_b = (uint8_t)(checksum >> 8); ///< Low byte

	return length + MAVLINK_NUM_NON_STX_PAYLOAD_BYTES;
}

/**
 * @brief Pack a message to send it over a serial byte stream
 */
static inline uint16_t mavlink_msg_to_send_buffer(uint8_t* buffer, const mavlink_message_t* msg)
{
	*(buffer+0) = MAVLINK_STX; ///< Start transmit
	memcpy((buffer+1), msg, msg->len + MAVLINK_CORE_HEADER_LEN); ///< Core header plus payload
	*(buffer + msg->len + MAVLINK_CORE_HEADER_LEN + 1) = msg->ck_a;
	*(buffer + msg->len + MAVLINK_CORE_HEADER_LEN + 2) = msg->ck_b;
	return msg->len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	return 0;
}

/**
 * @brief Get the required buffer size for this message
 */
static inline uint16_t mavlink_msg_get_send_buffer_length(const mavlink_message_t* msg)
{
	return msg->len + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

union checksum_ {
	uint16_t s;
	uint8_t c[2];
};

union __mavlink_bitfield {
	uint8_t uint8;
	int8_t int8;
	uint16_t uint16;
	int16_t int16;
	uint32_t uint32;
	int32_t int32;
};


static inline void mavlink_start_checksum(mavlink_message_t* msg)
{
	union checksum_ ck;
	crc_init(&(ck.s));
	msg->ck_a = ck.c[0];
	msg->ck_b = ck.c[1];
}

static inline void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c)
{
	union checksum_ ck;
	ck.c[0] = msg->ck_a;
	ck.c[1] = msg->ck_b;
	crc_accumulate(c, &(ck.s));
	msg->ck_a = ck.c[0];
	msg->ck_b = ck.c[1];
}

/**
 * This is a convenience function which handles the complete MAVLink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. Checksum and other failures will be silently
 * ignored.
 *
 * @param chan     ID of the current channel. This allows to parse different channels with this function.
 *                 a channel is not a physical message channel like a serial port, but a logic partition of
 *                 the communication streams in this case. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * @param c        The char to barse
 *
 * @param returnMsg NULL if no message could be decoded, the message data else
 * @return 0 if no message could be decoded, 1 else
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <inttypes.h> // For fixed-width uint8_t type
 *
 * mavlink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (mavlink_parse_char(chan, byte, &msg))
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
 *     }
 * }
 *
 *
 * @endcode
 */
static inline uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
#if (defined linux) | (defined __linux) | (defined  __MACH__) | (defined _WIN32)
	static mavlink_message_t m_mavlink_message[MAVLINK_COMM_NB_HIGH];
#else
	static mavlink_message_t m_mavlink_message[MAVLINK_COMM_NB];
#endif
	// Initializes only once, values keep unchanged after first initialization
	mavlink_parse_state_initialize(mavlink_get_channel_status(chan));

	mavlink_message_t* rxmsg = &m_mavlink_message[chan]; ///< The currently decoded message
	mavlink_status_t* status = mavlink_get_channel_status(chan); ///< The current decode status
	int bufferIndex = 0;

	status->msg_received = 0;

	switch (status->parse_state)
	{
	case MAVLINK_PARSE_STATE_UNINIT:
	case MAVLINK_PARSE_STATE_IDLE:
		if (c == MAVLINK_STX)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
			mavlink_start_checksum(rxmsg);
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_STX:
		if (status->msg_received)
		{
			status->buffer_overrun++;
			status->parse_error++;
			status->msg_received = 0;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
		}
		else
		{
			// NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
			rxmsg->len = c;
			status->packet_idx = 0;
			mavlink_update_checksum(rxmsg, c);
			status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_LENGTH:
		rxmsg->seq = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
		break;

	case MAVLINK_PARSE_STATE_GOT_SEQ:
		rxmsg->sysid = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
		break;

	case MAVLINK_PARSE_STATE_GOT_SYSID:
		rxmsg->compid = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
		break;

	case MAVLINK_PARSE_STATE_GOT_COMPID:
		rxmsg->msgid = c;
		mavlink_update_checksum(rxmsg, c);
		if (rxmsg->len == 0)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		}
		else
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID:
		rxmsg->payload[status->packet_idx++] = c;
		mavlink_update_checksum(rxmsg, c);
		if (status->packet_idx == rxmsg->len)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
		if (c != rxmsg->ck_a)
		{
			// Check first checksum byte
			status->parse_error++;
			status->msg_received = 0;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			if (c == MAVLINK_STX)
			{
				status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
				mavlink_start_checksum(rxmsg);
			}
		}
		else
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_CRC1:
		if (c != rxmsg->ck_b)
		{// Check second checksum byte
			status->parse_error++;
			status->msg_received = 0;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			if (c == MAVLINK_STX)
			{
				status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
				mavlink_start_checksum(rxmsg);
			}
		}
		else
		{
			// Successfully got message
			status->msg_received = 1;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
		}
		break;
	}

	bufferIndex++;
	// If a message has been sucessfully decoded, check index
	if (status->msg_received == 1)
	{
		//while(status->current_seq != rxmsg->seq)
		//{
		//	status->packet_rx_drop_count++;
		//               status->current_seq++;
		//}
		status->current_rx_seq = rxmsg->seq;
		// Initial condition: If no packet has been received so far, drop count is undefined
		if (status->packet_rx_success_count == 0) status->packet_rx_drop_count = 0;
		// Count this packet as received
		status->packet_rx_success_count++;
	}

	r_mavlink_status->current_rx_seq = status->current_rx_seq+1;
	r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;
	r_mavlink_status->packet_rx_drop_count = status->parse_error;
	status->parse_error = 0;
	return status->msg_received;
}


/**
 * This is a convenience function which handles the complete MAVLink parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. Checksum and other failures will be silently
 * ignored.
 *
 * @param chan     ID of the current channel. This allows to parse different channels with this function.
 *                 a channel is not a physical message channel like a serial port, but a logic partition of
 *                 the communication streams in this case. COMM_NB is the limit for the number of channels
 *                 on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
 * @param c        The char to barse
 *
 * @param returnMsg NULL if no message could be decoded, the message data else
 * @return 0 if no message could be decoded, 1 else
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <inttypes.h> // For fixed-width uint8_t type
 *
 * mavlink_message_t msg;
 * int chan = 0;
 *
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   uint8_t byte = serial.getNextByte();
 *   if (mavlink_parse_char(chan, byte, &msg))
 *     {
 *     printf("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
 *     }
 * }
 *
 *
 * @endcode
 */

#define MAVLINK_PACKET_START_CANDIDATES 50
/*
static inline uint8_t mavlink_parse_char_new(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
	#if (defined linux) | (defined __linux) | (defined  __MACH__) | (defined _WIN32)
	    static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NB_HIGH];
	    static uint8_t m_msgbuf[MAVLINK_COMM_NB_HIGH][MAVLINK_MAX_PACKET_LEN * 2];
	    static uint8_t m_msgbuf_index[MAVLINK_COMM_NB_HIGH];
	    static mavlink_message_t m_mavlink_message[MAVLINK_COMM_NB_HIGH];
	    static uint8_t m_packet_start[MAVLINK_COMM_NB_HIGH][MAVLINK_PACKET_START_CANDIDATES];
	    static uint8_t m_packet_start_index_read[MAVLINK_COMM_NB_HIGH];
	    static uint8_t m_packet_start_index_write[MAVLINK_COMM_NB_HIGH];
	#else
	    static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NB];
	    static uint8_t m_msgbuf[MAVLINK_COMM_NB][MAVLINK_MAX_PACKET_LEN * 2];
	    static uint8_t m_msgbuf_index[MAVLINK_COMM_NB];
	    static mavlink_message_t m_mavlink_message[MAVLINK_COMM_NB];
	    static uint8_t m_packet_start[MAVLINK_COMM_NB][MAVLINK_PACKET_START_CANDIDATES];
	    static uint8_t m_packet_start_index_read[MAVLINK_COMM_NB];
	    static uint8_t m_packet_start_index_write[MAVLINK_COMM_NB];
	#endif

	    // Set a packet start candidate index if sign is start sign
	    if (c == MAVLINK_STX)
	    {
	    	m_packet_start[chan][++(m_packet_start_index_write[chan]) % MAVLINK_PACKET_START_CANDIDATES] = m_msgbuf_index[chan];
	    }

	    // Parse normally, if a CRC mismatch occurs retry with the next packet index
}
//#if (defined linux) | (defined __linux) | (defined  __MACH__) | (defined _WIN32)
//    static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NB_HIGH];
//    static mavlink_message_t m_mavlink_message[MAVLINK_COMM_NB_HIGH];
//#else
//    static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NB];
//    static mavlink_message_t m_mavlink_message[MAVLINK_COMM_NB];
//#endif
//// Initializes only once, values keep unchanged after first initialization
//    mavlink_parse_state_initialize(&m_mavlink_status[chan]);
//
//mavlink_message_t* rxmsg = &m_mavlink_message[chan]; ///< The currently decoded message
//mavlink_status_t* status = &m_mavlink_status[chan]; ///< The current decode status
//int bufferIndex = 0;
//
//status->msg_received = 0;
//
//switch (status->parse_state)
//{
//case MAVLINK_PARSE_STATE_UNINIT:
//case MAVLINK_PARSE_STATE_IDLE:
//            if (c == MAVLINK_STX)
//	{
//		status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
//		mavlink_start_checksum(rxmsg);
//	}
//	break;
//
//case MAVLINK_PARSE_STATE_GOT_STX:
//	if (status->msg_received)
//	{
//		status->buffer_overrun++;
//		status->parse_error++;
//		status->msg_received = 0;
//		status->parse_state = MAVLINK_PARSE_STATE_IDLE;
//	}
//	else
//	{
//		// NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
//		rxmsg->len = c;
//		status->packet_idx = 0;
//		mavlink_update_checksum(rxmsg, c);
//		status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
//	}
//	break;
//
//case MAVLINK_PARSE_STATE_GOT_LENGTH:
//	rxmsg->seq = c;
//	mavlink_update_checksum(rxmsg, c);
//	status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
//	break;
//
//case MAVLINK_PARSE_STATE_GOT_SEQ:
//	rxmsg->sysid = c;
//	mavlink_update_checksum(rxmsg, c);
//	status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
//	break;
//
//case MAVLINK_PARSE_STATE_GOT_SYSID:
//	rxmsg->compid = c;
//	mavlink_update_checksum(rxmsg, c);
//	status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
//	break;
//
//case MAVLINK_PARSE_STATE_GOT_COMPID:
//	rxmsg->msgid = c;
//	mavlink_update_checksum(rxmsg, c);
//	if (rxmsg->len == 0)
//	{
//		status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
//	}
//	else
//	{
//		status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID;
//	}
//	break;
//
//case MAVLINK_PARSE_STATE_GOT_MSGID:
//	rxmsg->payload[status->packet_idx++] = c;
//	mavlink_update_checksum(rxmsg, c);
//	if (status->packet_idx == rxmsg->len)
//	{
//		status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
//	}
//	break;
//
//case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
//	if (c != rxmsg->ck_a)
//	{
//		// Check first checksum byte
//		status->parse_error++;
//		status->msg_received = 0;
//		status->parse_state = MAVLINK_PARSE_STATE_IDLE;
//	}
//	else
//	{
//		status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
//	}
//	break;
//
//case MAVLINK_PARSE_STATE_GOT_CRC1:
//	if (c != rxmsg->ck_b)
//	{// Check second checksum byte
//		status->parse_error++;
//		status->msg_received = 0;
//		status->parse_state = MAVLINK_PARSE_STATE_IDLE;
//	}
//	else
//	{
//		// Successfully got message
//		status->msg_received = 1;
//		status->parse_state = MAVLINK_PARSE_STATE_IDLE;
//		memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
//	}
//	break;
//}

bufferIndex++;
// If a message has been sucessfully decoded, check index
if (status->msg_received == 1)
{
    //while(status->current_seq != rxmsg->seq)
	//{
	//	status->packet_rx_drop_count++;
    //               status->current_seq++;
	//}
	status->current_seq = rxmsg->seq;
	// Initial condition: If no packet has been received so far, drop count is undefined
	if (status->packet_rx_success_count == 0) status->packet_rx_drop_count = 0;
	// Count this packet as received
	status->packet_rx_success_count++;
}

r_mavlink_status->current_seq = status->current_seq+1;
r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;
r_mavlink_status->packet_rx_drop_count = status->parse_error;
return status->msg_received;
}
 */


typedef union __generic_16bit
{
	uint8_t b[2];
	int16_t s;
} generic_16bit;

typedef union __generic_32bit
{
	uint8_t b[4];
	float f;
	int32_t i;
	int16_t s;
} generic_32bit;

typedef union __generic_64bit
{
	uint8_t b[8];
	int64_t ll; ///< Long long (64 bit)
} generic_64bit;

/**
 * @brief Place an unsigned byte into the buffer
 *
 * @param b the byte to add
 * @param bindex the position in the packet
 * @param buffer the packet buffer
 * @return the new position of the last used byte in the buffer
 */
static inline uint8_t put_uint8_t_by_index(uint8_t b, uint8_t bindex, uint8_t* buffer)
{
	*(buffer + bindex) = b;
	return sizeof(b);
}

/**
 * @brief Place a signed byte into the buffer
 *
 * @param b the byte to add
 * @param bindex the position in the packet
 * @param buffer the packet buffer
 * @return the new position of the last used byte in the buffer
 */
static inline uint8_t put_int8_t_by_index(int8_t b, int8_t bindex, uint8_t* buffer)
{
	*(buffer + bindex) = (uint8_t)b;
	return sizeof(b);
}

/**
 * @brief Place two unsigned bytes into the buffer
 *
 * @param b the bytes to add
 * @param bindex the position in the packet
 * @param buffer the packet buffer
 * @return the new position of the last used byte in the buffer
 */
static inline uint8_t put_uint16_t_by_index(uint16_t b, const uint8_t bindex, uint8_t* buffer)
{
	buffer[bindex]   = (b>>8)&0xff;
	buffer[bindex+1] = (b & 0xff);
	return sizeof(b);
}

/**
 * @brief Place two signed bytes into the buffer
 *
 * @param b the bytes to add
 * @param bindex the position in the packet
 * @param buffer the packet buffer
 * @return the new position of the last used byte in the buffer
 */
static inline uint8_t put_int16_t_by_index(int16_t b, uint8_t bindex, uint8_t* buffer)
{
	return put_uint16_t_by_index(b, bindex, buffer);
}

/**
 * @brief Place four unsigned bytes into the buffer
 *
 * @param b the bytes to add
 * @param bindex the position in the packet
 * @param buffer the packet buffer
 * @return the new position of the last used byte in the buffer
 */
static inline uint8_t put_uint32_t_by_index(uint32_t b, const uint8_t bindex, uint8_t* buffer)
{
	buffer[bindex]   = (b>>24)&0xff;
	buffer[bindex+1] = (b>>16)&0xff;
	buffer[bindex+2] = (b>>8)&0xff;
	buffer[bindex+3] = (b & 0xff);
	return sizeof(b);
}

/**
 * @brief Place four signed bytes into the buffer
 *
 * @param b the bytes to add
 * @param bindex the position in the packet
 * @param buffer the packet buffer
 * @return the new position of the last used byte in the buffer
 */
static inline uint8_t put_int32_t_by_index(int32_t b, uint8_t bindex, uint8_t* buffer)
{
	buffer[bindex]   = (b>>24)&0xff;
	buffer[bindex+1] = (b>>16)&0xff;
	buffer[bindex+2] = (b>>8)&0xff;
	buffer[bindex+3] = (b & 0xff);
	return sizeof(b);
}

/**
 * @brief Place four unsigned bytes into the buffer
 *
 * @param b the bytes to add
 * @param bindex the position in the packet
 * @param buffer the packet buffer
 * @return the new position of the last used byte in the buffer
 */
static inline uint8_t put_uint64_t_by_index(uint64_t b, const uint8_t bindex, uint8_t* buffer)
{
	buffer[bindex]   = (b>>56)&0xff;
	buffer[bindex+1] = (b>>48)&0xff;
	buffer[bindex+2] = (b>>40)&0xff;
	buffer[bindex+3] = (b>>32)&0xff;
	buffer[bindex+4] = (b>>24)&0xff;
	buffer[bindex+5] = (b>>16)&0xff;
	buffer[bindex+6] = (b>>8)&0xff;
	buffer[bindex+7] = (b & 0xff);
	return sizeof(b);
}

/**
 * @brief Place four signed bytes into the buffer
 *
 * @param b the bytes to add
 * @param bindex the position in the packet
 * @param buffer the packet buffer
 * @return the new position of the last used byte in the buffer
 */
static inline uint8_t put_int64_t_by_index(int64_t b, uint8_t bindex, uint8_t* buffer)
{
	return put_uint64_t_by_index(b, bindex, buffer);
}

/**
 * @brief Place a float into the buffer
 *
 * @param b the float to add
 * @param bindex the position in the packet
 * @param buffer the packet buffer
 * @return the new position of the last used byte in the buffer
 */
static inline uint8_t put_float_by_index(float b, uint8_t bindex, uint8_t* buffer)
{
	generic_32bit g;
	g.f = b;
	return put_int32_t_by_index(g.i, bindex, buffer);
}

/**
 * @brief Place an array into the buffer
 *
 * @param b the array to add
 * @param length size of the array (for strings: length WITH '\0' char)
 * @param bindex the position in the packet
 * @param buffer packet buffer
 * @return new position of the last used byte in the buffer
 */
static inline uint8_t put_array_by_index(const int8_t* b, uint8_t length, uint8_t bindex, uint8_t* buffer)
{
	memcpy(buffer+bindex, b, length);
	return length;
}

/**
 * @brief Place a string into the buffer
 *
 * @param b the string to add
 * @param maxlength size of the array (for strings: length WITHOUT '\0' char)
 * @param bindex the position in the packet
 * @param buffer packet buffer
 * @return new position of the last used byte in the buffer
 */
static inline uint8_t put_string_by_index(const char* b, uint8_t maxlength, uint8_t bindex, uint8_t* buffer)
{
	uint16_t length = 0;
	// Copy string into buffer, ensuring not to exceed the buffer size
	int i;
	for (i = 1; i < maxlength; i++)
	{
		length++;
		// String characters
		if (i < (maxlength - 1))
		{
			buffer[bindex+i] = b[i];
			// Stop at null character
			if (b[i] == '\0')
			{
				break;
			}
		}
		// Enforce null termination at end of buffer
		else if (i == (maxlength - 1))
		{
			buffer[i] = '\0';
		}
	}
	// Write length into first field
	put_uint8_t_by_index(length, bindex, buffer);
	return length;
}

/**
 * @brief Put a bitfield of length 1-32 bit into the buffer
 *
 * @param b the value to add, will be encoded in the bitfield
 * @param bits number of bits to use to encode b, e.g. 1 for boolean, 2, 3, etc.
 * @param packet_index the position in the packet (the index of the first byte to use)
 * @param bit_index the position in the byte (the index of the first bit to use)
 * @param buffer packet buffer to write into
 * @return new position of the last used byte in the buffer
 */
static inline uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index, uint8_t* r_bit_index, uint8_t* buffer)
{
	uint16_t bits_remain = bits;
	// Transform number into network order
	generic_32bit bin;
	generic_32bit bout;
	uint8_t i_bit_index, i_byte_index, curr_bits_n;
	bin.i = b;
	bout.b[0] = bin.b[3];
	bout.b[1] = bin.b[2];
	bout.b[2] = bin.b[1];
	bout.b[3] = bin.b[0];

	// buffer in
	// 01100000 01000000 00000000 11110001
	// buffer out
	// 11110001 00000000 01000000 01100000

	// Existing partly filled byte (four free slots)
	// 0111xxxx

	// Mask n free bits
	// 00001111 = 2^0 + 2^1 + 2^2 + 2^3 = 2^n - 1
	// = ((uint32_t)(1 << n)) - 1; // = 2^n - 1

	// Shift n bits into the right position
	// out = in >> n;

	// Mask and shift bytes
	i_bit_index = bit_index;
	i_byte_index = packet_index;
	if (bit_index > 0)
	{
		// If bits were available at start, they were available
		// in the byte before the current index
		i_byte_index--;
	}

	// While bits have not been packed yet
	while (bits_remain > 0)
	{
		// Bits still have to be packed
		// there can be more than 8 bits, so
		// we might have to pack them into more than one byte

		// First pack everything we can into the current 'open' byte
		//curr_bits_n = bits_remain << 3; // Equals  bits_remain mod 8
		//FIXME
		if (bits_remain <= (8 - i_bit_index))
		{
			// Enough space
			curr_bits_n = bits_remain;
		}
		else
		{
			curr_bits_n = (8 - i_bit_index);
		}
		
		// Pack these n bits into the current byte
		// Mask out whatever was at that position with ones (xxx11111)
		buffer[i_byte_index] &= (0xFF >> (8 - curr_bits_n));
		// Put content to this position, by masking out the non-used part
		buffer[i_byte_index] |= ((0x00 << curr_bits_n) & bout.i);
		
		// Increment the bit index
		i_bit_index += curr_bits_n;

		// Now proceed to the next byte, if necessary
		bits_remain -= curr_bits_n;
		if (bits_remain > 0)
		{
			// Offer another 8 bits / one byte
			i_byte_index++;
			i_bit_index = 0;
		}
	}
	
	*r_bit_index = i_bit_index;
	// If a partly filled byte is present, mark this as consumed
	if (i_bit_index != 7) i_byte_index++;
	return i_byte_index - packet_index;
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

// To make MAVLink work on your MCU, define a similar function

/*

#include "mavlink_types.h"

void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_0)
    {
        uart0_transmit(ch);
    }
    if (chan == MAVLINK_COMM_1)
    {
    	uart1_transmit(ch);
    }
}
 */


static inline void mavlink_send_uart(mavlink_channel_t chan, mavlink_message_t* msg)
{
	// ARM7 MCU board implementation
	// Create pointer on message struct
	// Send STX
	comm_send_ch(chan, MAVLINK_STX);
	comm_send_ch(chan, msg->len);
	comm_send_ch(chan, msg->seq);
	comm_send_ch(chan, msg->sysid);
	comm_send_ch(chan, msg->compid);
	comm_send_ch(chan, msg->msgid);
	for(uint16_t i = 0; i < msg->len; i++)
	{
		comm_send_ch(chan, msg->payload[i]);
	}
	comm_send_ch(chan, msg->ck_a);
	comm_send_ch(chan, msg->ck_b);
}
#endif

#endif /* _MAVLINK_PROTOCOL_H_ */

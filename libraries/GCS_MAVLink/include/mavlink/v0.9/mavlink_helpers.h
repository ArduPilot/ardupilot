#ifndef  _MAVLINK_HELPERS_H_
#define  _MAVLINK_HELPERS_H_

#include "string.h"
#include "checksum.h"
#include "mavlink_types.h"

#ifndef MAVLINK_HELPER
#define MAVLINK_HELPER
#endif

/*
  internal function to give access to the channel status for each channel
 */
MAVLINK_HELPER mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
	static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
	return &m_mavlink_status[chan];
}

/*
 internal function to give access to the channel buffer for each channel
 */
MAVLINK_HELPER mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan)
{
	
#if MAVLINK_EXTERNAL_RX_BUFFER
	// No m_mavlink_message array defined in function,
	// has to be defined externally
#ifndef m_mavlink_message
#error ERROR: IF #define MAVLINK_EXTERNAL_RX_BUFFER IS SET, THE BUFFER HAS TO BE ALLOCATED OUTSIDE OF THIS FUNCTION (mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];)
#endif
#else
	static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
#endif
	return &m_mavlink_buffer[chan];
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
 * @param length Message length
 */
#if MAVLINK_CRC_EXTRA
MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, 
						      uint8_t chan, uint8_t length, uint8_t crc_extra)
#else
MAVLINK_HELPER uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, 
						      uint8_t chan, uint8_t length)
#endif
{
	// This code part is the same for all messages;
	uint16_t checksum;
	msg->magic = MAVLINK_STX;
	msg->len = length;
	msg->sysid = system_id;
	msg->compid = component_id;
	// One sequence number per component
	msg->seq = mavlink_get_channel_status(chan)->current_tx_seq;
	mavlink_get_channel_status(chan)->current_tx_seq = mavlink_get_channel_status(chan)->current_tx_seq+1;
	checksum = crc_calculate((uint8_t*)&msg->len, length + MAVLINK_CORE_HEADER_LEN);
#if MAVLINK_CRC_EXTRA
	crc_accumulate(crc_extra, &checksum);
#endif
	mavlink_ck_a(msg) = (uint8_t)(checksum & 0xFF);
	mavlink_ck_b(msg) = (uint8_t)(checksum >> 8);

	return length + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}


/**
 * @brief Finalize a MAVLink message with MAVLINK_COMM_0 as default channel
 */
#if MAVLINK_CRC_EXTRA
MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, 
						 uint8_t length, uint8_t crc_extra)
{
	return mavlink_finalize_message_chan(msg, system_id, component_id, MAVLINK_COMM_0, length, crc_extra);
}
#else
MAVLINK_HELPER uint16_t mavlink_finalize_message(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, 
						 uint8_t length)
{
	return mavlink_finalize_message_chan(msg, system_id, component_id, MAVLINK_COMM_0, length);
}
#endif

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS
MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len);

/**
 * @brief Finalize a MAVLink message with channel assignment and send
 */
#if MAVLINK_CRC_EXTRA
MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint8_t msgid, const char *packet, 
						    uint8_t length, uint8_t crc_extra)
#else
MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint8_t msgid, const char *packet, uint8_t length)
#endif
{
	uint16_t checksum;
	uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(chan);
	buf[0] = MAVLINK_STX;
	buf[1] = length;
	buf[2] = status->current_tx_seq;
	buf[3] = mavlink_system.sysid;
	buf[4] = mavlink_system.compid;
	buf[5] = msgid;
	status->current_tx_seq++;
	checksum = crc_calculate((uint8_t*)&buf[1], MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, packet, length);
#if MAVLINK_CRC_EXTRA
	crc_accumulate(crc_extra, &checksum);
#endif
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	MAVLINK_START_UART_SEND(chan, MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)length);
	_mavlink_send_uart(chan, (const char *)buf, MAVLINK_NUM_HEADER_BYTES);
	_mavlink_send_uart(chan, packet, length);
	_mavlink_send_uart(chan, (const char *)ck, 2);
	MAVLINK_END_UART_SEND(chan, MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)length);
}

/**
 * @brief re-send a message over a uart channel
 * this is more stack efficient than re-marshalling the message
 */
MAVLINK_HELPER void _mavlink_resend_uart(mavlink_channel_t chan, const mavlink_message_t *msg)
{
	uint8_t ck[2];

	ck[0] = (uint8_t)(msg->checksum & 0xFF);
	ck[1] = (uint8_t)(msg->checksum >> 8);

	MAVLINK_START_UART_SEND(chan, MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len);
	_mavlink_send_uart(chan, (const char *)&msg->magic, MAVLINK_NUM_HEADER_BYTES);
	_mavlink_send_uart(chan, _MAV_PAYLOAD(msg), msg->len);
	_mavlink_send_uart(chan, (const char *)ck, 2);
	MAVLINK_END_UART_SEND(chan, MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len);
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

/**
 * @brief Pack a message to send it over a serial byte stream
 */
MAVLINK_HELPER uint16_t mavlink_msg_to_send_buffer(uint8_t *buffer, const mavlink_message_t *msg)
{
	memcpy(buffer, (const uint8_t *)&msg->magic, MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)msg->len);
	return MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)msg->len;
}

union __mavlink_bitfield {
	uint8_t uint8;
	int8_t int8;
	uint16_t uint16;
	int16_t int16;
	uint32_t uint32;
	int32_t int32;
};


MAVLINK_HELPER void mavlink_start_checksum(mavlink_message_t* msg)
{
	crc_init(&msg->checksum);
}

MAVLINK_HELPER void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c)
{
	crc_accumulate(c, &msg->checksum);
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
MAVLINK_HELPER uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
        /*
	  default message crc function. You can override this per-system to
	  put this data in a different memory segment
	*/
#if MAVLINK_CRC_EXTRA
#ifndef MAVLINK_MESSAGE_CRC
	static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;
#define MAVLINK_MESSAGE_CRC(msgid) mavlink_message_crcs[msgid]
#endif
#endif

	mavlink_message_t* rxmsg = mavlink_get_channel_buffer(chan); ///< The currently decoded message
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
			rxmsg->len = 0;
			rxmsg->magic = c;
			mavlink_start_checksum(rxmsg);
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_STX:
			if (status->msg_received 
/* Support shorter buffers than the
   default maximum packet size */
#if (MAVLINK_MAX_PAYLOAD_LEN < 255)
				|| c > MAVLINK_MAX_PAYLOAD_LEN
#endif
				)
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
		_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
		mavlink_update_checksum(rxmsg, c);
		if (status->packet_idx == rxmsg->len)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
#if MAVLINK_CRC_EXTRA
		mavlink_update_checksum(rxmsg, MAVLINK_MESSAGE_CRC(rxmsg->msgid));
#endif
		if (c != (rxmsg->checksum & 0xFF)) {
			// Check first checksum byte
			status->parse_error++;
			status->msg_received = 0;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			if (c == MAVLINK_STX)
			{
				status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
				rxmsg->len = 0;
				mavlink_start_checksum(rxmsg);
			}
		}
		else
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
			_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx] = (char)c;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_CRC1:
		if (c != (rxmsg->checksum >> 8)) {
			// Check second checksum byte
			status->parse_error++;
			status->msg_received = 0;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			if (c == MAVLINK_STX)
			{
				status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
				rxmsg->len = 0;
				mavlink_start_checksum(rxmsg);
			}
		}
		else
		{
			// Successfully got message
			status->msg_received = 1;
			status->parse_state = MAVLINK_PARSE_STATE_IDLE;
			_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx+1] = (char)c;
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
 * @brief Put a bitfield of length 1-32 bit into the buffer
 *
 * @param b the value to add, will be encoded in the bitfield
 * @param bits number of bits to use to encode b, e.g. 1 for boolean, 2, 3, etc.
 * @param packet_index the position in the packet (the index of the first byte to use)
 * @param bit_index the position in the byte (the index of the first bit to use)
 * @param buffer packet buffer to write into
 * @return new position of the last used byte in the buffer
 */
MAVLINK_HELPER uint8_t put_bitfield_n_by_index(int32_t b, uint8_t bits, uint8_t packet_index, uint8_t bit_index, uint8_t* r_bit_index, uint8_t* buffer)
{
	uint16_t bits_remain = bits;
	// Transform number into network order
	int32_t v;
	uint8_t i_bit_index, i_byte_index, curr_bits_n;
#if MAVLINK_NEED_BYTE_SWAP
	union {
		int32_t i;
		uint8_t b[4];
	} bin, bout;
	bin.i = b;
	bout.b[0] = bin.b[3];
	bout.b[1] = bin.b[2];
	bout.b[2] = bin.b[1];
	bout.b[3] = bin.b[0];
	v = bout.i;
#else
	v = b;
#endif

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
		if (bits_remain <= (uint8_t)(8 - i_bit_index))
		{
			// Enough space
			curr_bits_n = (uint8_t)bits_remain;
		}
		else
		{
			curr_bits_n = (8 - i_bit_index);
		}
		
		// Pack these n bits into the current byte
		// Mask out whatever was at that position with ones (xxx11111)
		buffer[i_byte_index] &= (0xFF >> (8 - curr_bits_n));
		// Put content to this position, by masking out the non-used part
		buffer[i_byte_index] |= ((0x00 << curr_bits_n) & v);
		
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

// To make MAVLink work on your MCU, define comm_send_ch() if you wish
// to send 1 byte at a time, or MAVLINK_SEND_UART_BYTES() to send a
// whole packet at a time

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

MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len)
{
#ifdef MAVLINK_SEND_UART_BYTES
	/* this is the more efficient approach, if the platform
	   defines it */
	MAVLINK_SEND_UART_BYTES(chan, (uint8_t *)buf, len);
#else
	/* fallback to one byte at a time */
	uint16_t i;
	for (i = 0; i < len; i++) {
		comm_send_ch(chan, (uint8_t)buf[i]);
	}
#endif
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS

#endif /* _MAVLINK_HELPERS_H_ */

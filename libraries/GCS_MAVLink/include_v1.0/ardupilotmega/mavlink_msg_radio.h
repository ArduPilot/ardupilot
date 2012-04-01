// MESSAGE RADIO PACKING

#define MAVLINK_MSG_ID_RADIO 166

typedef struct __mavlink_radio_t
{
 uint16_t rxerrors; ///< receive errors
 uint16_t serrors; ///< serial errors
 uint16_t fixed; ///< count of error corrected packets
 uint16_t txbuf; ///< number of bytes available in transmit buffer
 uint8_t rssi; ///< local signal strength
 uint8_t remrssi; ///< remote signal strength
} mavlink_radio_t;

#define MAVLINK_MSG_ID_RADIO_LEN 10
#define MAVLINK_MSG_ID_166_LEN 10



#define MAVLINK_MESSAGE_INFO_RADIO { \
	"RADIO", \
	6, \
	{  { "rxerrors", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_radio_t, rxerrors) }, \
         { "serrors", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_radio_t, serrors) }, \
         { "fixed", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_radio_t, fixed) }, \
         { "txbuf", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_radio_t, txbuf) }, \
         { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_radio_t, rssi) }, \
         { "remrssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_radio_t, remrssi) }, \
         } \
}


/**
 * @brief Pack a radio message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rssi local signal strength
 * @param remrssi remote signal strength
 * @param rxerrors receive errors
 * @param serrors serial errors
 * @param fixed count of error corrected packets
 * @param txbuf number of bytes available in transmit buffer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t rssi, uint8_t remrssi, uint16_t rxerrors, uint16_t serrors, uint16_t fixed, uint16_t txbuf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_uint16_t(buf, 0, rxerrors);
	_mav_put_uint16_t(buf, 2, serrors);
	_mav_put_uint16_t(buf, 4, fixed);
	_mav_put_uint16_t(buf, 6, txbuf);
	_mav_put_uint8_t(buf, 8, rssi);
	_mav_put_uint8_t(buf, 9, remrssi);

        memcpy(_MAV_PAYLOAD(msg), buf, 10);
#else
	mavlink_radio_t packet;
	packet.rxerrors = rxerrors;
	packet.serrors = serrors;
	packet.fixed = fixed;
	packet.txbuf = txbuf;
	packet.rssi = rssi;
	packet.remrssi = remrssi;

        memcpy(_MAV_PAYLOAD(msg), &packet, 10);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADIO;
	return mavlink_finalize_message(msg, system_id, component_id, 10, 128);
}

/**
 * @brief Pack a radio message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param rssi local signal strength
 * @param remrssi remote signal strength
 * @param rxerrors receive errors
 * @param serrors serial errors
 * @param fixed count of error corrected packets
 * @param txbuf number of bytes available in transmit buffer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t rssi,uint8_t remrssi,uint16_t rxerrors,uint16_t serrors,uint16_t fixed,uint16_t txbuf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_uint16_t(buf, 0, rxerrors);
	_mav_put_uint16_t(buf, 2, serrors);
	_mav_put_uint16_t(buf, 4, fixed);
	_mav_put_uint16_t(buf, 6, txbuf);
	_mav_put_uint8_t(buf, 8, rssi);
	_mav_put_uint8_t(buf, 9, remrssi);

        memcpy(_MAV_PAYLOAD(msg), buf, 10);
#else
	mavlink_radio_t packet;
	packet.rxerrors = rxerrors;
	packet.serrors = serrors;
	packet.fixed = fixed;
	packet.txbuf = txbuf;
	packet.rssi = rssi;
	packet.remrssi = remrssi;

        memcpy(_MAV_PAYLOAD(msg), &packet, 10);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADIO;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 10, 128);
}

/**
 * @brief Encode a radio struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radio C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_radio_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radio_t* radio)
{
	return mavlink_msg_radio_pack(system_id, component_id, msg, radio->rssi, radio->remrssi, radio->rxerrors, radio->serrors, radio->fixed, radio->txbuf);
}

/**
 * @brief Send a radio message
 * @param chan MAVLink channel to send the message
 *
 * @param rssi local signal strength
 * @param remrssi remote signal strength
 * @param rxerrors receive errors
 * @param serrors serial errors
 * @param fixed count of error corrected packets
 * @param txbuf number of bytes available in transmit buffer
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radio_send(mavlink_channel_t chan, uint8_t rssi, uint8_t remrssi, uint16_t rxerrors, uint16_t serrors, uint16_t fixed, uint16_t txbuf)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_uint16_t(buf, 0, rxerrors);
	_mav_put_uint16_t(buf, 2, serrors);
	_mav_put_uint16_t(buf, 4, fixed);
	_mav_put_uint16_t(buf, 6, txbuf);
	_mav_put_uint8_t(buf, 8, rssi);
	_mav_put_uint8_t(buf, 9, remrssi);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO, buf, 10, 128);
#else
	mavlink_radio_t packet;
	packet.rxerrors = rxerrors;
	packet.serrors = serrors;
	packet.fixed = fixed;
	packet.txbuf = txbuf;
	packet.rssi = rssi;
	packet.remrssi = remrssi;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO, (const char *)&packet, 10, 128);
#endif
}

#endif

// MESSAGE RADIO UNPACKING


/**
 * @brief Get field rssi from radio message
 *
 * @return local signal strength
 */
static inline uint8_t mavlink_msg_radio_get_rssi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field remrssi from radio message
 *
 * @return remote signal strength
 */
static inline uint8_t mavlink_msg_radio_get_remrssi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field rxerrors from radio message
 *
 * @return receive errors
 */
static inline uint16_t mavlink_msg_radio_get_rxerrors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field serrors from radio message
 *
 * @return serial errors
 */
static inline uint16_t mavlink_msg_radio_get_serrors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field fixed from radio message
 *
 * @return count of error corrected packets
 */
static inline uint16_t mavlink_msg_radio_get_fixed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field txbuf from radio message
 *
 * @return number of bytes available in transmit buffer
 */
static inline uint16_t mavlink_msg_radio_get_txbuf(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a radio message into a struct
 *
 * @param msg The message to decode
 * @param radio C-struct to decode the message contents into
 */
static inline void mavlink_msg_radio_decode(const mavlink_message_t* msg, mavlink_radio_t* radio)
{
#if MAVLINK_NEED_BYTE_SWAP
	radio->rxerrors = mavlink_msg_radio_get_rxerrors(msg);
	radio->serrors = mavlink_msg_radio_get_serrors(msg);
	radio->fixed = mavlink_msg_radio_get_fixed(msg);
	radio->txbuf = mavlink_msg_radio_get_txbuf(msg);
	radio->rssi = mavlink_msg_radio_get_rssi(msg);
	radio->remrssi = mavlink_msg_radio_get_remrssi(msg);
#else
	memcpy(radio, _MAV_PAYLOAD(msg), 10);
#endif
}

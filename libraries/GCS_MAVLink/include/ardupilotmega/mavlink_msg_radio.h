// MESSAGE RADIO PACKING

#define MAVLINK_MSG_ID_RADIO 166

typedef struct __mavlink_radio_t
{
 uint8_t rssi; ///< local signal strength
 uint8_t remrssi; ///< remote signal strength
 uint8_t txbuf; ///< how full the tx buffer is as a percentage
 uint16_t rxerrors; ///< receive errors
 uint16_t serrors; ///< serial errors
 uint16_t fixed; ///< count of error corrected packets
} mavlink_radio_t;

#define MAVLINK_MSG_ID_RADIO_LEN 9
#define MAVLINK_MSG_ID_166_LEN 9



#define MAVLINK_MESSAGE_INFO_RADIO { \
	"RADIO", \
	6, \
	{  { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_radio_t, rssi) }, \
         { "remrssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_radio_t, remrssi) }, \
         { "txbuf", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_radio_t, txbuf) }, \
         { "rxerrors", NULL, MAVLINK_TYPE_UINT16_T, 0, 3, offsetof(mavlink_radio_t, rxerrors) }, \
         { "serrors", NULL, MAVLINK_TYPE_UINT16_T, 0, 5, offsetof(mavlink_radio_t, serrors) }, \
         { "fixed", NULL, MAVLINK_TYPE_UINT16_T, 0, 7, offsetof(mavlink_radio_t, fixed) }, \
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
 * @param txbuf how full the tx buffer is as a percentage
 * @param rxerrors receive errors
 * @param serrors serial errors
 * @param fixed count of error corrected packets
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint16_t rxerrors, uint16_t serrors, uint16_t fixed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_uint8_t(buf, 0, rssi);
	_mav_put_uint8_t(buf, 1, remrssi);
	_mav_put_uint8_t(buf, 2, txbuf);
	_mav_put_uint16_t(buf, 3, rxerrors);
	_mav_put_uint16_t(buf, 5, serrors);
	_mav_put_uint16_t(buf, 7, fixed);

        memcpy(_MAV_PAYLOAD(msg), buf, 9);
#else
	mavlink_radio_t packet;
	packet.rssi = rssi;
	packet.remrssi = remrssi;
	packet.txbuf = txbuf;
	packet.rxerrors = rxerrors;
	packet.serrors = serrors;
	packet.fixed = fixed;

        memcpy(_MAV_PAYLOAD(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADIO;
	return mavlink_finalize_message(msg, system_id, component_id, 9);
}

/**
 * @brief Pack a radio message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param rssi local signal strength
 * @param remrssi remote signal strength
 * @param txbuf how full the tx buffer is as a percentage
 * @param rxerrors receive errors
 * @param serrors serial errors
 * @param fixed count of error corrected packets
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_radio_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t rssi,uint8_t remrssi,uint8_t txbuf,uint16_t rxerrors,uint16_t serrors,uint16_t fixed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_uint8_t(buf, 0, rssi);
	_mav_put_uint8_t(buf, 1, remrssi);
	_mav_put_uint8_t(buf, 2, txbuf);
	_mav_put_uint16_t(buf, 3, rxerrors);
	_mav_put_uint16_t(buf, 5, serrors);
	_mav_put_uint16_t(buf, 7, fixed);

        memcpy(_MAV_PAYLOAD(msg), buf, 9);
#else
	mavlink_radio_t packet;
	packet.rssi = rssi;
	packet.remrssi = remrssi;
	packet.txbuf = txbuf;
	packet.rxerrors = rxerrors;
	packet.serrors = serrors;
	packet.fixed = fixed;

        memcpy(_MAV_PAYLOAD(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_RADIO;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 9);
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
	return mavlink_msg_radio_pack(system_id, component_id, msg, radio->rssi, radio->remrssi, radio->txbuf, radio->rxerrors, radio->serrors, radio->fixed);
}

/**
 * @brief Send a radio message
 * @param chan MAVLink channel to send the message
 *
 * @param rssi local signal strength
 * @param remrssi remote signal strength
 * @param txbuf how full the tx buffer is as a percentage
 * @param rxerrors receive errors
 * @param serrors serial errors
 * @param fixed count of error corrected packets
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radio_send(mavlink_channel_t chan, uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint16_t rxerrors, uint16_t serrors, uint16_t fixed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_uint8_t(buf, 0, rssi);
	_mav_put_uint8_t(buf, 1, remrssi);
	_mav_put_uint8_t(buf, 2, txbuf);
	_mav_put_uint16_t(buf, 3, rxerrors);
	_mav_put_uint16_t(buf, 5, serrors);
	_mav_put_uint16_t(buf, 7, fixed);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO, buf, 9);
#else
	mavlink_radio_t packet;
	packet.rssi = rssi;
	packet.remrssi = remrssi;
	packet.txbuf = txbuf;
	packet.rxerrors = rxerrors;
	packet.serrors = serrors;
	packet.fixed = fixed;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO, (const char *)&packet, 9);
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
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field remrssi from radio message
 *
 * @return remote signal strength
 */
static inline uint8_t mavlink_msg_radio_get_remrssi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field txbuf from radio message
 *
 * @return how full the tx buffer is as a percentage
 */
static inline uint8_t mavlink_msg_radio_get_txbuf(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field rxerrors from radio message
 *
 * @return receive errors
 */
static inline uint16_t mavlink_msg_radio_get_rxerrors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  3);
}

/**
 * @brief Get field serrors from radio message
 *
 * @return serial errors
 */
static inline uint16_t mavlink_msg_radio_get_serrors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  5);
}

/**
 * @brief Get field fixed from radio message
 *
 * @return count of error corrected packets
 */
static inline uint16_t mavlink_msg_radio_get_fixed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  7);
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
	radio->rssi = mavlink_msg_radio_get_rssi(msg);
	radio->remrssi = mavlink_msg_radio_get_remrssi(msg);
	radio->txbuf = mavlink_msg_radio_get_txbuf(msg);
	radio->rxerrors = mavlink_msg_radio_get_rxerrors(msg);
	radio->serrors = mavlink_msg_radio_get_serrors(msg);
	radio->fixed = mavlink_msg_radio_get_fixed(msg);
#else
	memcpy(radio, _MAV_PAYLOAD(msg), 9);
#endif
}

// MESSAGE ALT_SENSOR_RAW PACKING

#define MAVLINK_MSG_ID_ALT_SENSOR_RAW 173

typedef struct __mavlink_alt_sensor_raw_t
{
 int32_t baro_alt; ///< barometer altitude
 int16_t sonar_alt; ///< sonar altitude
} mavlink_alt_sensor_raw_t;

#define MAVLINK_MSG_ID_ALT_SENSOR_RAW_LEN 6
#define MAVLINK_MSG_ID_173_LEN 6



#define MAVLINK_MESSAGE_INFO_ALT_SENSOR_RAW { \
	"ALT_SENSOR_RAW", \
	2, \
	{  { "baro_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_alt_sensor_raw_t, baro_alt) }, \
         { "sonar_alt", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_alt_sensor_raw_t, sonar_alt) }, \
         } \
}


/**
 * @brief Pack a alt_sensor_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param baro_alt barometer altitude
 * @param sonar_alt sonar altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_alt_sensor_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t baro_alt, int16_t sonar_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int32_t(buf, 0, baro_alt);
	_mav_put_int16_t(buf, 4, sonar_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_alt_sensor_raw_t packet;
	packet.baro_alt = baro_alt;
	packet.sonar_alt = sonar_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_ALT_SENSOR_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 6, 98);
}

/**
 * @brief Pack a alt_sensor_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param baro_alt barometer altitude
 * @param sonar_alt sonar altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_alt_sensor_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t baro_alt,int16_t sonar_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int32_t(buf, 0, baro_alt);
	_mav_put_int16_t(buf, 4, sonar_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_alt_sensor_raw_t packet;
	packet.baro_alt = baro_alt;
	packet.sonar_alt = sonar_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_ALT_SENSOR_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 6, 98);
}

/**
 * @brief Encode a alt_sensor_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param alt_sensor_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_alt_sensor_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_alt_sensor_raw_t* alt_sensor_raw)
{
	return mavlink_msg_alt_sensor_raw_pack(system_id, component_id, msg, alt_sensor_raw->baro_alt, alt_sensor_raw->sonar_alt);
}

/**
 * @brief Send a alt_sensor_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param baro_alt barometer altitude
 * @param sonar_alt sonar altitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_alt_sensor_raw_send(mavlink_channel_t chan, int32_t baro_alt, int16_t sonar_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int32_t(buf, 0, baro_alt);
	_mav_put_int16_t(buf, 4, sonar_alt);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALT_SENSOR_RAW, buf, 6, 98);
#else
	mavlink_alt_sensor_raw_t packet;
	packet.baro_alt = baro_alt;
	packet.sonar_alt = sonar_alt;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ALT_SENSOR_RAW, (const char *)&packet, 6, 98);
#endif
}

#endif

// MESSAGE ALT_SENSOR_RAW UNPACKING


/**
 * @brief Get field baro_alt from alt_sensor_raw message
 *
 * @return barometer altitude
 */
static inline int32_t mavlink_msg_alt_sensor_raw_get_baro_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field sonar_alt from alt_sensor_raw message
 *
 * @return sonar altitude
 */
static inline int16_t mavlink_msg_alt_sensor_raw_get_sonar_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Decode a alt_sensor_raw message into a struct
 *
 * @param msg The message to decode
 * @param alt_sensor_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_alt_sensor_raw_decode(const mavlink_message_t* msg, mavlink_alt_sensor_raw_t* alt_sensor_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	alt_sensor_raw->baro_alt = mavlink_msg_alt_sensor_raw_get_baro_alt(msg);
	alt_sensor_raw->sonar_alt = mavlink_msg_alt_sensor_raw_get_sonar_alt(msg);
#else
	memcpy(alt_sensor_raw, _MAV_PAYLOAD(msg), 6);
#endif
}

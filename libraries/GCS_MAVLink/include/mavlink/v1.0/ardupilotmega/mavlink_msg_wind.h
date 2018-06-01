// MESSAGE WIND PACKING

#define MAVLINK_MSG_ID_WIND 168

typedef struct __mavlink_wind_t
{
 float direction; /*< wind direction that wind is coming from (degrees)*/
 float speed; /*< wind speed in ground plane (m/s)*/
 float speed_z; /*< vertical wind speed (m/s)*/
} mavlink_wind_t;

#define MAVLINK_MSG_ID_WIND_LEN 12
#define MAVLINK_MSG_ID_168_LEN 12

#define MAVLINK_MSG_ID_WIND_CRC 1
#define MAVLINK_MSG_ID_168_CRC 1



#define MAVLINK_MESSAGE_INFO_WIND { \
	"WIND", \
	3, \
	{  { "direction", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_wind_t, direction) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_wind_t, speed) }, \
         { "speed_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wind_t, speed_z) }, \
         } \
}


/**
 * @brief Pack a wind message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param direction wind direction that wind is coming from (degrees)
 * @param speed wind speed in ground plane (m/s)
 * @param speed_z vertical wind speed (m/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wind_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float direction, float speed, float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WIND_LEN];
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, speed);
	_mav_put_float(buf, 8, speed_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIND_LEN);
#else
	mavlink_wind_t packet;
	packet.direction = direction;
	packet.speed = speed;
	packet.speed_z = speed_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WIND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WIND_LEN, MAVLINK_MSG_ID_WIND_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WIND_LEN);
#endif
}

/**
 * @brief Pack a wind message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param direction wind direction that wind is coming from (degrees)
 * @param speed wind speed in ground plane (m/s)
 * @param speed_z vertical wind speed (m/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wind_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float direction,float speed,float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WIND_LEN];
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, speed);
	_mav_put_float(buf, 8, speed_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIND_LEN);
#else
	mavlink_wind_t packet;
	packet.direction = direction;
	packet.speed = speed;
	packet.speed_z = speed_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WIND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WIND_LEN, MAVLINK_MSG_ID_WIND_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WIND_LEN);
#endif
}

/**
 * @brief Encode a wind struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wind C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wind_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wind_t* wind)
{
	return mavlink_msg_wind_pack(system_id, component_id, msg, wind->direction, wind->speed, wind->speed_z);
}

/**
 * @brief Encode a wind struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wind C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wind_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wind_t* wind)
{
	return mavlink_msg_wind_pack_chan(system_id, component_id, chan, msg, wind->direction, wind->speed, wind->speed_z);
}

/**
 * @brief Send a wind message
 * @param chan MAVLink channel to send the message
 *
 * @param direction wind direction that wind is coming from (degrees)
 * @param speed wind speed in ground plane (m/s)
 * @param speed_z vertical wind speed (m/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wind_send(mavlink_channel_t chan, float direction, float speed, float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WIND_LEN];
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, speed);
	_mav_put_float(buf, 8, speed_z);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND, buf, MAVLINK_MSG_ID_WIND_LEN, MAVLINK_MSG_ID_WIND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND, buf, MAVLINK_MSG_ID_WIND_LEN);
#endif
#else
	mavlink_wind_t packet;
	packet.direction = direction;
	packet.speed = speed;
	packet.speed_z = speed_z;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND, (const char *)&packet, MAVLINK_MSG_ID_WIND_LEN, MAVLINK_MSG_ID_WIND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND, (const char *)&packet, MAVLINK_MSG_ID_WIND_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_WIND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wind_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float direction, float speed, float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, direction);
	_mav_put_float(buf, 4, speed);
	_mav_put_float(buf, 8, speed_z);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND, buf, MAVLINK_MSG_ID_WIND_LEN, MAVLINK_MSG_ID_WIND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND, buf, MAVLINK_MSG_ID_WIND_LEN);
#endif
#else
	mavlink_wind_t *packet = (mavlink_wind_t *)msgbuf;
	packet->direction = direction;
	packet->speed = speed;
	packet->speed_z = speed_z;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND, (const char *)packet, MAVLINK_MSG_ID_WIND_LEN, MAVLINK_MSG_ID_WIND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND, (const char *)packet, MAVLINK_MSG_ID_WIND_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE WIND UNPACKING


/**
 * @brief Get field direction from wind message
 *
 * @return wind direction that wind is coming from (degrees)
 */
static inline float mavlink_msg_wind_get_direction(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field speed from wind message
 *
 * @return wind speed in ground plane (m/s)
 */
static inline float mavlink_msg_wind_get_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field speed_z from wind message
 *
 * @return vertical wind speed (m/s)
 */
static inline float mavlink_msg_wind_get_speed_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a wind message into a struct
 *
 * @param msg The message to decode
 * @param wind C-struct to decode the message contents into
 */
static inline void mavlink_msg_wind_decode(const mavlink_message_t* msg, mavlink_wind_t* wind)
{
#if MAVLINK_NEED_BYTE_SWAP
	wind->direction = mavlink_msg_wind_get_direction(msg);
	wind->speed = mavlink_msg_wind_get_speed(msg);
	wind->speed_z = mavlink_msg_wind_get_speed_z(msg);
#else
	memcpy(wind, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_WIND_LEN);
#endif
}

// MESSAGE SIMSTATE PACKING

#define MAVLINK_MSG_ID_SIMSTATE 164

typedef struct __mavlink_simstate_t
{
 float roll; /*< Roll angle (rad)*/
 float pitch; /*< Pitch angle (rad)*/
 float yaw; /*< Yaw angle (rad)*/
 float xacc; /*< X acceleration m/s/s*/
 float yacc; /*< Y acceleration m/s/s*/
 float zacc; /*< Z acceleration m/s/s*/
 float xgyro; /*< Angular speed around X axis rad/s*/
 float ygyro; /*< Angular speed around Y axis rad/s*/
 float zgyro; /*< Angular speed around Z axis rad/s*/
 int32_t lat; /*< Latitude in degrees * 1E7*/
 int32_t lng; /*< Longitude in degrees * 1E7*/
} mavlink_simstate_t;

#define MAVLINK_MSG_ID_SIMSTATE_LEN 44
#define MAVLINK_MSG_ID_164_LEN 44

#define MAVLINK_MSG_ID_SIMSTATE_CRC 154
#define MAVLINK_MSG_ID_164_CRC 154



#define MAVLINK_MESSAGE_INFO_SIMSTATE { \
	"SIMSTATE", \
	11, \
	{  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_simstate_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_simstate_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_simstate_t, yaw) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_simstate_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_simstate_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_simstate_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_simstate_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_simstate_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_simstate_t, zgyro) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_simstate_t, lat) }, \
         { "lng", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_simstate_t, lng) }, \
         } \
}


/**
 * @brief Pack a simstate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param xacc X acceleration m/s/s
 * @param yacc Y acceleration m/s/s
 * @param zacc Z acceleration m/s/s
 * @param xgyro Angular speed around X axis rad/s
 * @param ygyro Angular speed around Y axis rad/s
 * @param zgyro Angular speed around Z axis rad/s
 * @param lat Latitude in degrees * 1E7
 * @param lng Longitude in degrees * 1E7
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_simstate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SIMSTATE_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, xacc);
	_mav_put_float(buf, 16, yacc);
	_mav_put_float(buf, 20, zacc);
	_mav_put_float(buf, 24, xgyro);
	_mav_put_float(buf, 28, ygyro);
	_mav_put_float(buf, 32, zgyro);
	_mav_put_int32_t(buf, 36, lat);
	_mav_put_int32_t(buf, 40, lng);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIMSTATE_LEN);
#else
	mavlink_simstate_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.lat = lat;
	packet.lng = lng;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SIMSTATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif
}

/**
 * @brief Pack a simstate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param xacc X acceleration m/s/s
 * @param yacc Y acceleration m/s/s
 * @param zacc Z acceleration m/s/s
 * @param xgyro Angular speed around X axis rad/s
 * @param ygyro Angular speed around Y axis rad/s
 * @param zgyro Angular speed around Z axis rad/s
 * @param lat Latitude in degrees * 1E7
 * @param lng Longitude in degrees * 1E7
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_simstate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float roll,float pitch,float yaw,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,int32_t lat,int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SIMSTATE_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, xacc);
	_mav_put_float(buf, 16, yacc);
	_mav_put_float(buf, 20, zacc);
	_mav_put_float(buf, 24, xgyro);
	_mav_put_float(buf, 28, ygyro);
	_mav_put_float(buf, 32, zgyro);
	_mav_put_int32_t(buf, 36, lat);
	_mav_put_int32_t(buf, 40, lng);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SIMSTATE_LEN);
#else
	mavlink_simstate_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.lat = lat;
	packet.lng = lng;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SIMSTATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif
}

/**
 * @brief Encode a simstate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param simstate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_simstate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_simstate_t* simstate)
{
	return mavlink_msg_simstate_pack(system_id, component_id, msg, simstate->roll, simstate->pitch, simstate->yaw, simstate->xacc, simstate->yacc, simstate->zacc, simstate->xgyro, simstate->ygyro, simstate->zgyro, simstate->lat, simstate->lng);
}

/**
 * @brief Encode a simstate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param simstate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_simstate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_simstate_t* simstate)
{
	return mavlink_msg_simstate_pack_chan(system_id, component_id, chan, msg, simstate->roll, simstate->pitch, simstate->yaw, simstate->xacc, simstate->yacc, simstate->zacc, simstate->xgyro, simstate->ygyro, simstate->zgyro, simstate->lat, simstate->lng);
}

/**
 * @brief Send a simstate message
 * @param chan MAVLink channel to send the message
 *
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param xacc X acceleration m/s/s
 * @param yacc Y acceleration m/s/s
 * @param zacc Z acceleration m/s/s
 * @param xgyro Angular speed around X axis rad/s
 * @param ygyro Angular speed around Y axis rad/s
 * @param zgyro Angular speed around Z axis rad/s
 * @param lat Latitude in degrees * 1E7
 * @param lng Longitude in degrees * 1E7
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_simstate_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SIMSTATE_LEN];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, xacc);
	_mav_put_float(buf, 16, yacc);
	_mav_put_float(buf, 20, zacc);
	_mav_put_float(buf, 24, xgyro);
	_mav_put_float(buf, 28, ygyro);
	_mav_put_float(buf, 32, zgyro);
	_mav_put_int32_t(buf, 36, lat);
	_mav_put_int32_t(buf, 40, lng);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, buf, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, buf, MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif
#else
	mavlink_simstate_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.lat = lat;
	packet.lng = lng;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, (const char *)&packet, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, (const char *)&packet, MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SIMSTATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_simstate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, int32_t lat, int32_t lng)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, xacc);
	_mav_put_float(buf, 16, yacc);
	_mav_put_float(buf, 20, zacc);
	_mav_put_float(buf, 24, xgyro);
	_mav_put_float(buf, 28, ygyro);
	_mav_put_float(buf, 32, zgyro);
	_mav_put_int32_t(buf, 36, lat);
	_mav_put_int32_t(buf, 40, lng);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, buf, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, buf, MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif
#else
	mavlink_simstate_t *packet = (mavlink_simstate_t *)msgbuf;
	packet->roll = roll;
	packet->pitch = pitch;
	packet->yaw = yaw;
	packet->xacc = xacc;
	packet->yacc = yacc;
	packet->zacc = zacc;
	packet->xgyro = xgyro;
	packet->ygyro = ygyro;
	packet->zgyro = zgyro;
	packet->lat = lat;
	packet->lng = lng;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, (const char *)packet, MAVLINK_MSG_ID_SIMSTATE_LEN, MAVLINK_MSG_ID_SIMSTATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, (const char *)packet, MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SIMSTATE UNPACKING


/**
 * @brief Get field roll from simstate message
 *
 * @return Roll angle (rad)
 */
static inline float mavlink_msg_simstate_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from simstate message
 *
 * @return Pitch angle (rad)
 */
static inline float mavlink_msg_simstate_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from simstate message
 *
 * @return Yaw angle (rad)
 */
static inline float mavlink_msg_simstate_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field xacc from simstate message
 *
 * @return X acceleration m/s/s
 */
static inline float mavlink_msg_simstate_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yacc from simstate message
 *
 * @return Y acceleration m/s/s
 */
static inline float mavlink_msg_simstate_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field zacc from simstate message
 *
 * @return Z acceleration m/s/s
 */
static inline float mavlink_msg_simstate_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field xgyro from simstate message
 *
 * @return Angular speed around X axis rad/s
 */
static inline float mavlink_msg_simstate_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field ygyro from simstate message
 *
 * @return Angular speed around Y axis rad/s
 */
static inline float mavlink_msg_simstate_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field zgyro from simstate message
 *
 * @return Angular speed around Z axis rad/s
 */
static inline float mavlink_msg_simstate_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field lat from simstate message
 *
 * @return Latitude in degrees * 1E7
 */
static inline int32_t mavlink_msg_simstate_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field lng from simstate message
 *
 * @return Longitude in degrees * 1E7
 */
static inline int32_t mavlink_msg_simstate_get_lng(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  40);
}

/**
 * @brief Decode a simstate message into a struct
 *
 * @param msg The message to decode
 * @param simstate C-struct to decode the message contents into
 */
static inline void mavlink_msg_simstate_decode(const mavlink_message_t* msg, mavlink_simstate_t* simstate)
{
#if MAVLINK_NEED_BYTE_SWAP
	simstate->roll = mavlink_msg_simstate_get_roll(msg);
	simstate->pitch = mavlink_msg_simstate_get_pitch(msg);
	simstate->yaw = mavlink_msg_simstate_get_yaw(msg);
	simstate->xacc = mavlink_msg_simstate_get_xacc(msg);
	simstate->yacc = mavlink_msg_simstate_get_yacc(msg);
	simstate->zacc = mavlink_msg_simstate_get_zacc(msg);
	simstate->xgyro = mavlink_msg_simstate_get_xgyro(msg);
	simstate->ygyro = mavlink_msg_simstate_get_ygyro(msg);
	simstate->zgyro = mavlink_msg_simstate_get_zgyro(msg);
	simstate->lat = mavlink_msg_simstate_get_lat(msg);
	simstate->lng = mavlink_msg_simstate_get_lng(msg);
#else
	memcpy(simstate, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SIMSTATE_LEN);
#endif
}

// MESSAGE SIMSTATE PACKING

#define MAVLINK_MSG_ID_SIMSTATE 164

typedef struct __mavlink_simstate_t
{
 float roll; ///< Roll angle (rad)
 float pitch; ///< Pitch angle (rad)
 float yaw; ///< Yaw angle (rad)
 float xacc; ///< X acceleration m/s/s
 float yacc; ///< Y acceleration m/s/s
 float zacc; ///< Z acceleration m/s/s
 float xgyro; ///< Angular speed around X axis rad/s
 float ygyro; ///< Angular speed around Y axis rad/s
 float zgyro; ///< Angular speed around Z axis rad/s
} mavlink_simstate_t;

#define MAVLINK_MSG_ID_SIMSTATE_LEN 36
#define MAVLINK_MSG_ID_164_LEN 36



#define MAVLINK_MESSAGE_INFO_SIMSTATE { \
	"SIMSTATE", \
	9, \
	{  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_simstate_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_simstate_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_simstate_t, yaw) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_simstate_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_simstate_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_simstate_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_simstate_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_simstate_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_simstate_t, zgyro) }, \
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
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_simstate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, xacc);
	_mav_put_float(buf, 16, yacc);
	_mav_put_float(buf, 20, zacc);
	_mav_put_float(buf, 24, xgyro);
	_mav_put_float(buf, 28, ygyro);
	_mav_put_float(buf, 32, zgyro);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
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

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_SIMSTATE;
	return mavlink_finalize_message(msg, system_id, component_id, 36, 42);
}

/**
 * @brief Pack a simstate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
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
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_simstate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float roll,float pitch,float yaw,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, xacc);
	_mav_put_float(buf, 16, yacc);
	_mav_put_float(buf, 20, zacc);
	_mav_put_float(buf, 24, xgyro);
	_mav_put_float(buf, 28, ygyro);
	_mav_put_float(buf, 32, zgyro);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
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

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_SIMSTATE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 42);
}

/**
 * @brief Encode a simstate struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param simstate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_simstate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_simstate_t* simstate)
{
	return mavlink_msg_simstate_pack(system_id, component_id, msg, simstate->roll, simstate->pitch, simstate->yaw, simstate->xacc, simstate->yacc, simstate->zacc, simstate->xgyro, simstate->ygyro, simstate->zgyro);
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
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_simstate_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_float(buf, 0, roll);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, yaw);
	_mav_put_float(buf, 12, xacc);
	_mav_put_float(buf, 16, yacc);
	_mav_put_float(buf, 20, zacc);
	_mav_put_float(buf, 24, xgyro);
	_mav_put_float(buf, 28, ygyro);
	_mav_put_float(buf, 32, zgyro);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, buf, 36, 42);
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

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SIMSTATE, (const char *)&packet, 36, 42);
#endif
}

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
#else
	memcpy(simstate, _MAV_PAYLOAD(msg), 36);
#endif
}

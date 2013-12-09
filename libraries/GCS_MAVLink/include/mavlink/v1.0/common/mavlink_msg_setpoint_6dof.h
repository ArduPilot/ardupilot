// MESSAGE SETPOINT_6DOF PACKING

#define MAVLINK_MSG_ID_SETPOINT_6DOF 149

typedef struct __mavlink_setpoint_6dof_t
{
 float trans_x; ///< Translational Component in x
 float trans_y; ///< Translational Component in y
 float trans_z; ///< Translational Component in z
 float rot_x; ///< Rotational Component in x
 float rot_y; ///< Rotational Component in y
 float rot_z; ///< Rotational Component in z
 uint8_t target_system; ///< System ID
} mavlink_setpoint_6dof_t;

#define MAVLINK_MSG_ID_SETPOINT_6DOF_LEN 25
#define MAVLINK_MSG_ID_149_LEN 25

#define MAVLINK_MSG_ID_SETPOINT_6DOF_CRC 15
#define MAVLINK_MSG_ID_149_CRC 15



#define MAVLINK_MESSAGE_INFO_SETPOINT_6DOF { \
	"SETPOINT_6DOF", \
	7, \
	{  { "trans_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_setpoint_6dof_t, trans_x) }, \
         { "trans_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_setpoint_6dof_t, trans_y) }, \
         { "trans_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_setpoint_6dof_t, trans_z) }, \
         { "rot_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_setpoint_6dof_t, rot_x) }, \
         { "rot_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_setpoint_6dof_t, rot_y) }, \
         { "rot_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_setpoint_6dof_t, rot_z) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_setpoint_6dof_t, target_system) }, \
         } \
}


/**
 * @brief Pack a setpoint_6dof message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param trans_x Translational Component in x
 * @param trans_y Translational Component in y
 * @param trans_z Translational Component in z
 * @param rot_x Rotational Component in x
 * @param rot_y Rotational Component in y
 * @param rot_z Rotational Component in z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_setpoint_6dof_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, float trans_x, float trans_y, float trans_z, float rot_x, float rot_y, float rot_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SETPOINT_6DOF_LEN];
	_mav_put_float(buf, 0, trans_x);
	_mav_put_float(buf, 4, trans_y);
	_mav_put_float(buf, 8, trans_z);
	_mav_put_float(buf, 12, rot_x);
	_mav_put_float(buf, 16, rot_y);
	_mav_put_float(buf, 20, rot_z);
	_mav_put_uint8_t(buf, 24, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN);
#else
	mavlink_setpoint_6dof_t packet;
	packet.trans_x = trans_x;
	packet.trans_y = trans_y;
	packet.trans_z = trans_z;
	packet.rot_x = rot_x;
	packet.rot_y = rot_y;
	packet.rot_z = rot_z;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SETPOINT_6DOF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN, MAVLINK_MSG_ID_SETPOINT_6DOF_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN);
#endif
}

/**
 * @brief Pack a setpoint_6dof message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param trans_x Translational Component in x
 * @param trans_y Translational Component in y
 * @param trans_z Translational Component in z
 * @param rot_x Rotational Component in x
 * @param rot_y Rotational Component in y
 * @param rot_z Rotational Component in z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_setpoint_6dof_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,float trans_x,float trans_y,float trans_z,float rot_x,float rot_y,float rot_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SETPOINT_6DOF_LEN];
	_mav_put_float(buf, 0, trans_x);
	_mav_put_float(buf, 4, trans_y);
	_mav_put_float(buf, 8, trans_z);
	_mav_put_float(buf, 12, rot_x);
	_mav_put_float(buf, 16, rot_y);
	_mav_put_float(buf, 20, rot_z);
	_mav_put_uint8_t(buf, 24, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN);
#else
	mavlink_setpoint_6dof_t packet;
	packet.trans_x = trans_x;
	packet.trans_y = trans_y;
	packet.trans_z = trans_z;
	packet.rot_x = rot_x;
	packet.rot_y = rot_y;
	packet.rot_z = rot_z;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SETPOINT_6DOF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN, MAVLINK_MSG_ID_SETPOINT_6DOF_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN);
#endif
}

/**
 * @brief Encode a setpoint_6dof struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param setpoint_6dof C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_setpoint_6dof_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_setpoint_6dof_t* setpoint_6dof)
{
	return mavlink_msg_setpoint_6dof_pack(system_id, component_id, msg, setpoint_6dof->target_system, setpoint_6dof->trans_x, setpoint_6dof->trans_y, setpoint_6dof->trans_z, setpoint_6dof->rot_x, setpoint_6dof->rot_y, setpoint_6dof->rot_z);
}

/**
 * @brief Encode a setpoint_6dof struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param setpoint_6dof C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_setpoint_6dof_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_setpoint_6dof_t* setpoint_6dof)
{
	return mavlink_msg_setpoint_6dof_pack_chan(system_id, component_id, chan, msg, setpoint_6dof->target_system, setpoint_6dof->trans_x, setpoint_6dof->trans_y, setpoint_6dof->trans_z, setpoint_6dof->rot_x, setpoint_6dof->rot_y, setpoint_6dof->rot_z);
}

/**
 * @brief Send a setpoint_6dof message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param trans_x Translational Component in x
 * @param trans_y Translational Component in y
 * @param trans_z Translational Component in z
 * @param rot_x Rotational Component in x
 * @param rot_y Rotational Component in y
 * @param rot_z Rotational Component in z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_setpoint_6dof_send(mavlink_channel_t chan, uint8_t target_system, float trans_x, float trans_y, float trans_z, float rot_x, float rot_y, float rot_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SETPOINT_6DOF_LEN];
	_mav_put_float(buf, 0, trans_x);
	_mav_put_float(buf, 4, trans_y);
	_mav_put_float(buf, 8, trans_z);
	_mav_put_float(buf, 12, rot_x);
	_mav_put_float(buf, 16, rot_y);
	_mav_put_float(buf, 20, rot_z);
	_mav_put_uint8_t(buf, 24, target_system);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SETPOINT_6DOF, buf, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN, MAVLINK_MSG_ID_SETPOINT_6DOF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SETPOINT_6DOF, buf, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN);
#endif
#else
	mavlink_setpoint_6dof_t packet;
	packet.trans_x = trans_x;
	packet.trans_y = trans_y;
	packet.trans_z = trans_z;
	packet.rot_x = rot_x;
	packet.rot_y = rot_y;
	packet.rot_z = rot_z;
	packet.target_system = target_system;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SETPOINT_6DOF, (const char *)&packet, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN, MAVLINK_MSG_ID_SETPOINT_6DOF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SETPOINT_6DOF, (const char *)&packet, MAVLINK_MSG_ID_SETPOINT_6DOF_LEN);
#endif
#endif
}

#endif

// MESSAGE SETPOINT_6DOF UNPACKING


/**
 * @brief Get field target_system from setpoint_6dof message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_setpoint_6dof_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field trans_x from setpoint_6dof message
 *
 * @return Translational Component in x
 */
static inline float mavlink_msg_setpoint_6dof_get_trans_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field trans_y from setpoint_6dof message
 *
 * @return Translational Component in y
 */
static inline float mavlink_msg_setpoint_6dof_get_trans_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field trans_z from setpoint_6dof message
 *
 * @return Translational Component in z
 */
static inline float mavlink_msg_setpoint_6dof_get_trans_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rot_x from setpoint_6dof message
 *
 * @return Rotational Component in x
 */
static inline float mavlink_msg_setpoint_6dof_get_rot_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field rot_y from setpoint_6dof message
 *
 * @return Rotational Component in y
 */
static inline float mavlink_msg_setpoint_6dof_get_rot_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rot_z from setpoint_6dof message
 *
 * @return Rotational Component in z
 */
static inline float mavlink_msg_setpoint_6dof_get_rot_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a setpoint_6dof message into a struct
 *
 * @param msg The message to decode
 * @param setpoint_6dof C-struct to decode the message contents into
 */
static inline void mavlink_msg_setpoint_6dof_decode(const mavlink_message_t* msg, mavlink_setpoint_6dof_t* setpoint_6dof)
{
#if MAVLINK_NEED_BYTE_SWAP
	setpoint_6dof->trans_x = mavlink_msg_setpoint_6dof_get_trans_x(msg);
	setpoint_6dof->trans_y = mavlink_msg_setpoint_6dof_get_trans_y(msg);
	setpoint_6dof->trans_z = mavlink_msg_setpoint_6dof_get_trans_z(msg);
	setpoint_6dof->rot_x = mavlink_msg_setpoint_6dof_get_rot_x(msg);
	setpoint_6dof->rot_y = mavlink_msg_setpoint_6dof_get_rot_y(msg);
	setpoint_6dof->rot_z = mavlink_msg_setpoint_6dof_get_rot_z(msg);
	setpoint_6dof->target_system = mavlink_msg_setpoint_6dof_get_target_system(msg);
#else
	memcpy(setpoint_6dof, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SETPOINT_6DOF_LEN);
#endif
}

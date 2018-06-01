// MESSAGE SET_ACTUATOR_CONTROL_TARGET PACKING

#define MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET 139

typedef struct __mavlink_set_actuator_control_target_t
{
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 float controls[8]; /*< Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.*/
 uint8_t group_mlx; /*< Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
} mavlink_set_actuator_control_target_t;

#define MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN 43
#define MAVLINK_MSG_ID_139_LEN 43

#define MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_CRC 168
#define MAVLINK_MSG_ID_139_CRC 168

#define MAVLINK_MSG_SET_ACTUATOR_CONTROL_TARGET_FIELD_CONTROLS_LEN 8

#define MAVLINK_MESSAGE_INFO_SET_ACTUATOR_CONTROL_TARGET { \
	"SET_ACTUATOR_CONTROL_TARGET", \
	5, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_set_actuator_control_target_t, time_usec) }, \
         { "controls", NULL, MAVLINK_TYPE_FLOAT, 8, 8, offsetof(mavlink_set_actuator_control_target_t, controls) }, \
         { "group_mlx", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_set_actuator_control_target_t, group_mlx) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_set_actuator_control_target_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_set_actuator_control_target_t, target_component) }, \
         } \
}


/**
 * @brief Pack a set_actuator_control_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param group_mlx Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
 * @param target_system System ID
 * @param target_component Component ID
 * @param controls Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_actuator_control_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t group_mlx, uint8_t target_system, uint8_t target_component, const float *controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 40, group_mlx);
	_mav_put_uint8_t(buf, 41, target_system);
	_mav_put_uint8_t(buf, 42, target_component);
	_mav_put_float_array(buf, 8, controls, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#else
	mavlink_set_actuator_control_target_t packet;
	packet.time_usec = time_usec;
	packet.group_mlx = group_mlx;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.controls, controls, sizeof(float)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#endif
}

/**
 * @brief Pack a set_actuator_control_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param group_mlx Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
 * @param target_system System ID
 * @param target_component Component ID
 * @param controls Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_actuator_control_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t group_mlx,uint8_t target_system,uint8_t target_component,const float *controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 40, group_mlx);
	_mav_put_uint8_t(buf, 41, target_system);
	_mav_put_uint8_t(buf, 42, target_component);
	_mav_put_float_array(buf, 8, controls, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#else
	mavlink_set_actuator_control_target_t packet;
	packet.time_usec = time_usec;
	packet.group_mlx = group_mlx;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.controls, controls, sizeof(float)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#endif
}

/**
 * @brief Encode a set_actuator_control_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_actuator_control_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_actuator_control_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_actuator_control_target_t* set_actuator_control_target)
{
	return mavlink_msg_set_actuator_control_target_pack(system_id, component_id, msg, set_actuator_control_target->time_usec, set_actuator_control_target->group_mlx, set_actuator_control_target->target_system, set_actuator_control_target->target_component, set_actuator_control_target->controls);
}

/**
 * @brief Encode a set_actuator_control_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_actuator_control_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_actuator_control_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_actuator_control_target_t* set_actuator_control_target)
{
	return mavlink_msg_set_actuator_control_target_pack_chan(system_id, component_id, chan, msg, set_actuator_control_target->time_usec, set_actuator_control_target->group_mlx, set_actuator_control_target->target_system, set_actuator_control_target->target_component, set_actuator_control_target->controls);
}

/**
 * @brief Send a set_actuator_control_target message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param group_mlx Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
 * @param target_system System ID
 * @param target_component Component ID
 * @param controls Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_actuator_control_target_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t group_mlx, uint8_t target_system, uint8_t target_component, const float *controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 40, group_mlx);
	_mav_put_uint8_t(buf, 41, target_system);
	_mav_put_uint8_t(buf, 42, target_component);
	_mav_put_float_array(buf, 8, controls, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET, buf, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET, buf, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#endif
#else
	mavlink_set_actuator_control_target_t packet;
	packet.time_usec = time_usec;
	packet.group_mlx = group_mlx;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.controls, controls, sizeof(float)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET, (const char *)&packet, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET, (const char *)&packet, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_actuator_control_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t group_mlx, uint8_t target_system, uint8_t target_component, const float *controls)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint8_t(buf, 40, group_mlx);
	_mav_put_uint8_t(buf, 41, target_system);
	_mav_put_uint8_t(buf, 42, target_component);
	_mav_put_float_array(buf, 8, controls, 8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET, buf, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET, buf, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#endif
#else
	mavlink_set_actuator_control_target_t *packet = (mavlink_set_actuator_control_target_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->group_mlx = group_mlx;
	packet->target_system = target_system;
	packet->target_component = target_component;
	mav_array_memcpy(packet->controls, controls, sizeof(float)*8);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET, (const char *)packet, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET, (const char *)packet, MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_ACTUATOR_CONTROL_TARGET UNPACKING


/**
 * @brief Get field time_usec from set_actuator_control_target message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_set_actuator_control_target_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field group_mlx from set_actuator_control_target message
 *
 * @return Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
 */
static inline uint8_t mavlink_msg_set_actuator_control_target_get_group_mlx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field target_system from set_actuator_control_target message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_actuator_control_target_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field target_component from set_actuator_control_target message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_actuator_control_target_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field controls from set_actuator_control_target message
 *
 * @return Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
 */
static inline uint16_t mavlink_msg_set_actuator_control_target_get_controls(const mavlink_message_t* msg, float *controls)
{
	return _MAV_RETURN_float_array(msg, controls, 8,  8);
}

/**
 * @brief Decode a set_actuator_control_target message into a struct
 *
 * @param msg The message to decode
 * @param set_actuator_control_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_actuator_control_target_decode(const mavlink_message_t* msg, mavlink_set_actuator_control_target_t* set_actuator_control_target)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_actuator_control_target->time_usec = mavlink_msg_set_actuator_control_target_get_time_usec(msg);
	mavlink_msg_set_actuator_control_target_get_controls(msg, set_actuator_control_target->controls);
	set_actuator_control_target->group_mlx = mavlink_msg_set_actuator_control_target_get_group_mlx(msg);
	set_actuator_control_target->target_system = mavlink_msg_set_actuator_control_target_get_target_system(msg);
	set_actuator_control_target->target_component = mavlink_msg_set_actuator_control_target_get_target_component(msg);
#else
	memcpy(set_actuator_control_target, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET_LEN);
#endif
}

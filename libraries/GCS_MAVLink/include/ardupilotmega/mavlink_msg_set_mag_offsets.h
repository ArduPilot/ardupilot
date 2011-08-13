// MESSAGE SET_MAG_OFFSETS PACKING

#define MAVLINK_MSG_ID_SET_MAG_OFFSETS 151

typedef struct __mavlink_set_mag_offsets_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	int16_t mag_ofs_x; ///< magnetometer X offset
	int16_t mag_ofs_y; ///< magnetometer Y offset
	int16_t mag_ofs_z; ///< magnetometer Z offset

} mavlink_set_mag_offsets_t;



/**
 * @brief Pack a set_mag_offsets message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mag_offsets_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SET_MAG_OFFSETS;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_int16_t_by_index(mag_ofs_x, i, msg->payload); // magnetometer X offset
	i += put_int16_t_by_index(mag_ofs_y, i, msg->payload); // magnetometer Y offset
	i += put_int16_t_by_index(mag_ofs_z, i, msg->payload); // magnetometer Z offset

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a set_mag_offsets message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mag_offsets_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SET_MAG_OFFSETS;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_int16_t_by_index(mag_ofs_x, i, msg->payload); // magnetometer X offset
	i += put_int16_t_by_index(mag_ofs_y, i, msg->payload); // magnetometer Y offset
	i += put_int16_t_by_index(mag_ofs_z, i, msg->payload); // magnetometer Z offset

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a set_mag_offsets struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_mag_offsets C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_mag_offsets_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_mag_offsets_t* set_mag_offsets)
{
	return mavlink_msg_set_mag_offsets_pack(system_id, component_id, msg, set_mag_offsets->target_system, set_mag_offsets->target_component, set_mag_offsets->mag_ofs_x, set_mag_offsets->mag_ofs_y, set_mag_offsets->mag_ofs_z);
}

/**
 * @brief Send a set_mag_offsets message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_mag_offsets_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int16_t mag_ofs_x, int16_t mag_ofs_y, int16_t mag_ofs_z)
{
	mavlink_message_t msg;
	mavlink_msg_set_mag_offsets_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, mag_ofs_x, mag_ofs_y, mag_ofs_z);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SET_MAG_OFFSETS UNPACKING

/**
 * @brief Get field target_system from set_mag_offsets message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_mag_offsets_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from set_mag_offsets message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_mag_offsets_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field mag_ofs_x from set_mag_offsets message
 *
 * @return magnetometer X offset
 */
static inline int16_t mavlink_msg_set_mag_offsets_get_mag_ofs_x(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field mag_ofs_y from set_mag_offsets message
 *
 * @return magnetometer Y offset
 */
static inline int16_t mavlink_msg_set_mag_offsets_get_mag_ofs_y(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field mag_ofs_z from set_mag_offsets message
 *
 * @return magnetometer Z offset
 */
static inline int16_t mavlink_msg_set_mag_offsets_get_mag_ofs_z(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Decode a set_mag_offsets message into a struct
 *
 * @param msg The message to decode
 * @param set_mag_offsets C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_mag_offsets_decode(const mavlink_message_t* msg, mavlink_set_mag_offsets_t* set_mag_offsets)
{
	set_mag_offsets->target_system = mavlink_msg_set_mag_offsets_get_target_system(msg);
	set_mag_offsets->target_component = mavlink_msg_set_mag_offsets_get_target_component(msg);
	set_mag_offsets->mag_ofs_x = mavlink_msg_set_mag_offsets_get_mag_ofs_x(msg);
	set_mag_offsets->mag_ofs_y = mavlink_msg_set_mag_offsets_get_mag_ofs_y(msg);
	set_mag_offsets->mag_ofs_z = mavlink_msg_set_mag_offsets_get_mag_ofs_z(msg);
}

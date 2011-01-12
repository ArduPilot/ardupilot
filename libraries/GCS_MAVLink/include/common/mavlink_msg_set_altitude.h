// MESSAGE SET_ALTITUDE PACKING

#define MAVLINK_MSG_ID_SET_ALTITUDE 65

typedef struct __mavlink_set_altitude_t 
{
	uint8_t target; ///< The system setting the altitude
	uint32_t mode; ///< The new altitude in meters

} mavlink_set_altitude_t;



/**
 * @brief Pack a set_altitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the altitude
 * @param mode The new altitude in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_altitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target, uint32_t mode)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SET_ALTITUDE;

	i += put_uint8_t_by_index(target, i, msg->payload); // The system setting the altitude
	i += put_uint32_t_by_index(mode, i, msg->payload); // The new altitude in meters

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a set_altitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the altitude
 * @param mode The new altitude in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_altitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target, uint32_t mode)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SET_ALTITUDE;

	i += put_uint8_t_by_index(target, i, msg->payload); // The system setting the altitude
	i += put_uint32_t_by_index(mode, i, msg->payload); // The new altitude in meters

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a set_altitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_altitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_altitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_altitude_t* set_altitude)
{
	return mavlink_msg_set_altitude_pack(system_id, component_id, msg, set_altitude->target, set_altitude->mode);
}

/**
 * @brief Send a set_altitude message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the altitude
 * @param mode The new altitude in meters
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_altitude_send(mavlink_channel_t chan, uint8_t target, uint32_t mode)
{
	mavlink_message_t msg;
	mavlink_msg_set_altitude_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target, mode);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SET_ALTITUDE UNPACKING

/**
 * @brief Get field target from set_altitude message
 *
 * @return The system setting the altitude
 */
static inline uint8_t mavlink_msg_set_altitude_get_target(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field mode from set_altitude message
 *
 * @return The new altitude in meters
 */
static inline uint32_t mavlink_msg_set_altitude_get_mode(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t))[3];
	return (uint32_t)r.i;
}

/**
 * @brief Decode a set_altitude message into a struct
 *
 * @param msg The message to decode
 * @param set_altitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_altitude_decode(const mavlink_message_t* msg, mavlink_set_altitude_t* set_altitude)
{
	set_altitude->target = mavlink_msg_set_altitude_get_target(msg);
	set_altitude->mode = mavlink_msg_set_altitude_get_mode(msg);
}

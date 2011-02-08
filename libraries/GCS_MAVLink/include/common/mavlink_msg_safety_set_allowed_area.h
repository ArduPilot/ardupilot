// MESSAGE SAFETY_SET_ALLOWED_AREA PACKING

#define MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA 53

typedef struct __mavlink_safety_set_allowed_area_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	uint8_t frame; ///< Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
	float p1x; ///< x position 1 / Latitude 1
	float p1y; ///< y position 1 / Longitude 1
	float p1z; ///< z position 1 / Altitude 1
	float p2x; ///< x position 2 / Latitude 2
	float p2y; ///< y position 2 / Longitude 2
	float p2z; ///< z position 2 / Altitude 2

} mavlink_safety_set_allowed_area_t;



/**
 * @brief Pack a safety_set_allowed_area message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param frame Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x x position 1 / Latitude 1
 * @param p1y y position 1 / Longitude 1
 * @param p1z z position 1 / Altitude 1
 * @param p2x x position 2 / Latitude 2
 * @param p2y y position 2 / Longitude 2
 * @param p2z z position 2 / Altitude 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_safety_set_allowed_area_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_uint8_t_by_index(frame, i, msg->payload); // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
	i += put_float_by_index(p1x, i, msg->payload); // x position 1 / Latitude 1
	i += put_float_by_index(p1y, i, msg->payload); // y position 1 / Longitude 1
	i += put_float_by_index(p1z, i, msg->payload); // z position 1 / Altitude 1
	i += put_float_by_index(p2x, i, msg->payload); // x position 2 / Latitude 2
	i += put_float_by_index(p2y, i, msg->payload); // y position 2 / Longitude 2
	i += put_float_by_index(p2z, i, msg->payload); // z position 2 / Altitude 2

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a safety_set_allowed_area message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param frame Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x x position 1 / Latitude 1
 * @param p1y y position 1 / Longitude 1
 * @param p1z z position 1 / Altitude 1
 * @param p2x x position 2 / Latitude 2
 * @param p2y y position 2 / Longitude 2
 * @param p2z z position 2 / Altitude 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_safety_set_allowed_area_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_uint8_t_by_index(frame, i, msg->payload); // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
	i += put_float_by_index(p1x, i, msg->payload); // x position 1 / Latitude 1
	i += put_float_by_index(p1y, i, msg->payload); // y position 1 / Longitude 1
	i += put_float_by_index(p1z, i, msg->payload); // z position 1 / Altitude 1
	i += put_float_by_index(p2x, i, msg->payload); // x position 2 / Latitude 2
	i += put_float_by_index(p2y, i, msg->payload); // y position 2 / Longitude 2
	i += put_float_by_index(p2z, i, msg->payload); // z position 2 / Altitude 2

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a safety_set_allowed_area struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param safety_set_allowed_area C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_safety_set_allowed_area_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_safety_set_allowed_area_t* safety_set_allowed_area)
{
	return mavlink_msg_safety_set_allowed_area_pack(system_id, component_id, msg, safety_set_allowed_area->target_system, safety_set_allowed_area->target_component, safety_set_allowed_area->frame, safety_set_allowed_area->p1x, safety_set_allowed_area->p1y, safety_set_allowed_area->p1z, safety_set_allowed_area->p2x, safety_set_allowed_area->p2y, safety_set_allowed_area->p2z);
}

/**
 * @brief Send a safety_set_allowed_area message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param frame Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x x position 1 / Latitude 1
 * @param p1y y position 1 / Longitude 1
 * @param p1z z position 1 / Altitude 1
 * @param p2x x position 2 / Latitude 2
 * @param p2y y position 2 / Longitude 2
 * @param p2z z position 2 / Altitude 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_safety_set_allowed_area_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t frame, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z)
{
	mavlink_message_t msg;
	mavlink_msg_safety_set_allowed_area_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, frame, p1x, p1y, p1z, p2x, p2y, p2z);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SAFETY_SET_ALLOWED_AREA UNPACKING

/**
 * @brief Get field target_system from safety_set_allowed_area message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_safety_set_allowed_area_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from safety_set_allowed_area message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_safety_set_allowed_area_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field frame from safety_set_allowed_area message
 *
 * @return Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 */
static inline uint8_t mavlink_msg_safety_set_allowed_area_get_frame(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field p1x from safety_set_allowed_area message
 *
 * @return x position 1 / Latitude 1
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p1x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field p1y from safety_set_allowed_area message
 *
 * @return y position 1 / Longitude 1
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p1y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field p1z from safety_set_allowed_area message
 *
 * @return z position 1 / Altitude 1
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p1z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field p2x from safety_set_allowed_area message
 *
 * @return x position 2 / Latitude 2
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p2x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field p2y from safety_set_allowed_area message
 *
 * @return y position 2 / Longitude 2
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p2y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field p2z from safety_set_allowed_area message
 *
 * @return z position 2 / Altitude 2
 */
static inline float mavlink_msg_safety_set_allowed_area_get_p2z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a safety_set_allowed_area message into a struct
 *
 * @param msg The message to decode
 * @param safety_set_allowed_area C-struct to decode the message contents into
 */
static inline void mavlink_msg_safety_set_allowed_area_decode(const mavlink_message_t* msg, mavlink_safety_set_allowed_area_t* safety_set_allowed_area)
{
	safety_set_allowed_area->target_system = mavlink_msg_safety_set_allowed_area_get_target_system(msg);
	safety_set_allowed_area->target_component = mavlink_msg_safety_set_allowed_area_get_target_component(msg);
	safety_set_allowed_area->frame = mavlink_msg_safety_set_allowed_area_get_frame(msg);
	safety_set_allowed_area->p1x = mavlink_msg_safety_set_allowed_area_get_p1x(msg);
	safety_set_allowed_area->p1y = mavlink_msg_safety_set_allowed_area_get_p1y(msg);
	safety_set_allowed_area->p1z = mavlink_msg_safety_set_allowed_area_get_p1z(msg);
	safety_set_allowed_area->p2x = mavlink_msg_safety_set_allowed_area_get_p2x(msg);
	safety_set_allowed_area->p2y = mavlink_msg_safety_set_allowed_area_get_p2y(msg);
	safety_set_allowed_area->p2z = mavlink_msg_safety_set_allowed_area_get_p2z(msg);
}

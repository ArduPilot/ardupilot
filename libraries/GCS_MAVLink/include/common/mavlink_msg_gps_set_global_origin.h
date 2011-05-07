// MESSAGE GPS_SET_GLOBAL_ORIGIN PACKING

#define MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN 48

typedef struct __mavlink_gps_set_global_origin_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	int32_t latitude; ///< global position * 1E7
	int32_t longitude; ///< global position * 1E7
	int32_t altitude; ///< global position * 1000

} mavlink_gps_set_global_origin_t;



/**
 * @brief Pack a gps_set_global_origin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_set_global_origin_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, int32_t latitude, int32_t longitude, int32_t altitude)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_int32_t_by_index(latitude, i, msg->payload); // global position * 1E7
	i += put_int32_t_by_index(longitude, i, msg->payload); // global position * 1E7
	i += put_int32_t_by_index(altitude, i, msg->payload); // global position * 1000

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a gps_set_global_origin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_set_global_origin_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, int32_t latitude, int32_t longitude, int32_t altitude)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GPS_SET_GLOBAL_ORIGIN;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_int32_t_by_index(latitude, i, msg->payload); // global position * 1E7
	i += put_int32_t_by_index(longitude, i, msg->payload); // global position * 1E7
	i += put_int32_t_by_index(altitude, i, msg->payload); // global position * 1000

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a gps_set_global_origin struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_set_global_origin C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_set_global_origin_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_set_global_origin_t* gps_set_global_origin)
{
	return mavlink_msg_gps_set_global_origin_pack(system_id, component_id, msg, gps_set_global_origin->target_system, gps_set_global_origin->target_component, gps_set_global_origin->latitude, gps_set_global_origin->longitude, gps_set_global_origin->altitude);
}

/**
 * @brief Send a gps_set_global_origin message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_set_global_origin_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int32_t latitude, int32_t longitude, int32_t altitude)
{
	mavlink_message_t msg;
	mavlink_msg_gps_set_global_origin_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, latitude, longitude, altitude);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE GPS_SET_GLOBAL_ORIGIN UNPACKING

/**
 * @brief Get field target_system from gps_set_global_origin message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gps_set_global_origin_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from gps_set_global_origin message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gps_set_global_origin_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field latitude from gps_set_global_origin message
 *
 * @return global position * 1E7
 */
static inline int32_t mavlink_msg_gps_set_global_origin_get_latitude(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field longitude from gps_set_global_origin message
 *
 * @return global position * 1E7
 */
static inline int32_t mavlink_msg_gps_set_global_origin_get_longitude(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int32_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field altitude from gps_set_global_origin message
 *
 * @return global position * 1000
 */
static inline int32_t mavlink_msg_gps_set_global_origin_get_altitude(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int32_t)+sizeof(int32_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int32_t)+sizeof(int32_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int32_t)+sizeof(int32_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(int32_t)+sizeof(int32_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Decode a gps_set_global_origin message into a struct
 *
 * @param msg The message to decode
 * @param gps_set_global_origin C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_set_global_origin_decode(const mavlink_message_t* msg, mavlink_gps_set_global_origin_t* gps_set_global_origin)
{
	gps_set_global_origin->target_system = mavlink_msg_gps_set_global_origin_get_target_system(msg);
	gps_set_global_origin->target_component = mavlink_msg_gps_set_global_origin_get_target_component(msg);
	gps_set_global_origin->latitude = mavlink_msg_gps_set_global_origin_get_latitude(msg);
	gps_set_global_origin->longitude = mavlink_msg_gps_set_global_origin_get_longitude(msg);
	gps_set_global_origin->altitude = mavlink_msg_gps_set_global_origin_get_altitude(msg);
}

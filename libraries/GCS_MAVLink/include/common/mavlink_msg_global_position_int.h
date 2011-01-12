// MESSAGE GLOBAL_POSITION_INT PACKING

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 73

typedef struct __mavlink_global_position_int_t 
{
	int32_t lat; ///< Latitude / X Position, expressed as * 1E7
	int32_t lon; ///< Longitude / Y Position, expressed as * 1E7
	int32_t alt; ///< Altitude / negative Z Position, expressed as * 1000
	int16_t vx; ///< Ground X Speed, expressed as m/s * 100
	int16_t vy; ///< Ground Y Speed, expressed as m/s * 100
	int16_t vz; ///< Ground Z Speed, expressed as m/s * 100

} mavlink_global_position_int_t;



/**
 * @brief Pack a global_position_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat Latitude / X Position, expressed as * 1E7
 * @param lon Longitude / Y Position, expressed as * 1E7
 * @param alt Altitude / negative Z Position, expressed as * 1000
 * @param vx Ground X Speed, expressed as m/s * 100
 * @param vy Ground Y Speed, expressed as m/s * 100
 * @param vz Ground Z Speed, expressed as m/s * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;

	i += put_int32_t_by_index(lat, i, msg->payload); // Latitude / X Position, expressed as * 1E7
	i += put_int32_t_by_index(lon, i, msg->payload); // Longitude / Y Position, expressed as * 1E7
	i += put_int32_t_by_index(alt, i, msg->payload); // Altitude / negative Z Position, expressed as * 1000
	i += put_int16_t_by_index(vx, i, msg->payload); // Ground X Speed, expressed as m/s * 100
	i += put_int16_t_by_index(vy, i, msg->payload); // Ground Y Speed, expressed as m/s * 100
	i += put_int16_t_by_index(vz, i, msg->payload); // Ground Z Speed, expressed as m/s * 100

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a global_position_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat Latitude / X Position, expressed as * 1E7
 * @param lon Longitude / Y Position, expressed as * 1E7
 * @param alt Altitude / negative Z Position, expressed as * 1000
 * @param vx Ground X Speed, expressed as m/s * 100
 * @param vy Ground Y Speed, expressed as m/s * 100
 * @param vz Ground Z Speed, expressed as m/s * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;

	i += put_int32_t_by_index(lat, i, msg->payload); // Latitude / X Position, expressed as * 1E7
	i += put_int32_t_by_index(lon, i, msg->payload); // Longitude / Y Position, expressed as * 1E7
	i += put_int32_t_by_index(alt, i, msg->payload); // Altitude / negative Z Position, expressed as * 1000
	i += put_int16_t_by_index(vx, i, msg->payload); // Ground X Speed, expressed as m/s * 100
	i += put_int16_t_by_index(vy, i, msg->payload); // Ground Y Speed, expressed as m/s * 100
	i += put_int16_t_by_index(vz, i, msg->payload); // Ground Z Speed, expressed as m/s * 100

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a global_position_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_int_t* global_position_int)
{
	return mavlink_msg_global_position_int_pack(system_id, component_id, msg, global_position_int->lat, global_position_int->lon, global_position_int->alt, global_position_int->vx, global_position_int->vy, global_position_int->vz);
}

/**
 * @brief Send a global_position_int message
 * @param chan MAVLink channel to send the message
 *
 * @param lat Latitude / X Position, expressed as * 1E7
 * @param lon Longitude / Y Position, expressed as * 1E7
 * @param alt Altitude / negative Z Position, expressed as * 1000
 * @param vx Ground X Speed, expressed as m/s * 100
 * @param vy Ground Y Speed, expressed as m/s * 100
 * @param vz Ground Z Speed, expressed as m/s * 100
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_int_send(mavlink_channel_t chan, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz)
{
	mavlink_message_t msg;
	mavlink_msg_global_position_int_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, lat, lon, alt, vx, vy, vz);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE GLOBAL_POSITION_INT UNPACKING

/**
 * @brief Get field lat from global_position_int message
 *
 * @return Latitude / X Position, expressed as * 1E7
 */
static inline int32_t mavlink_msg_global_position_int_get_lat(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field lon from global_position_int message
 *
 * @return Longitude / Y Position, expressed as * 1E7
 */
static inline int32_t mavlink_msg_global_position_int_get_lon(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int32_t))[0];
	r.b[2] = (msg->payload+sizeof(int32_t))[1];
	r.b[1] = (msg->payload+sizeof(int32_t))[2];
	r.b[0] = (msg->payload+sizeof(int32_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field alt from global_position_int message
 *
 * @return Altitude / negative Z Position, expressed as * 1000
 */
static inline int32_t mavlink_msg_global_position_int_get_alt(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(int32_t)+sizeof(int32_t))[0];
	r.b[2] = (msg->payload+sizeof(int32_t)+sizeof(int32_t))[1];
	r.b[1] = (msg->payload+sizeof(int32_t)+sizeof(int32_t))[2];
	r.b[0] = (msg->payload+sizeof(int32_t)+sizeof(int32_t))[3];
	return (int32_t)r.i;
}

/**
 * @brief Get field vx from global_position_int message
 *
 * @return Ground X Speed, expressed as m/s * 100
 */
static inline int16_t mavlink_msg_global_position_int_get_vx(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t))[0];
	r.b[0] = (msg->payload+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field vy from global_position_int message
 *
 * @return Ground Y Speed, expressed as m/s * 100
 */
static inline int16_t mavlink_msg_global_position_int_get_vy(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Get field vz from global_position_int message
 *
 * @return Ground Z Speed, expressed as m/s * 100
 */
static inline int16_t mavlink_msg_global_position_int_get_vz(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t)+sizeof(int16_t))[0];
	r.b[0] = (msg->payload+sizeof(int32_t)+sizeof(int32_t)+sizeof(int32_t)+sizeof(int16_t)+sizeof(int16_t))[1];
	return (int16_t)r.s;
}

/**
 * @brief Decode a global_position_int message into a struct
 *
 * @param msg The message to decode
 * @param global_position_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_position_int_decode(const mavlink_message_t* msg, mavlink_global_position_int_t* global_position_int)
{
	global_position_int->lat = mavlink_msg_global_position_int_get_lat(msg);
	global_position_int->lon = mavlink_msg_global_position_int_get_lon(msg);
	global_position_int->alt = mavlink_msg_global_position_int_get_alt(msg);
	global_position_int->vx = mavlink_msg_global_position_int_get_vx(msg);
	global_position_int->vy = mavlink_msg_global_position_int_get_vy(msg);
	global_position_int->vz = mavlink_msg_global_position_int_get_vz(msg);
}

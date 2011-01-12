// MESSAGE POINT_OF_INTEREST PACKING

#define MAVLINK_MSG_ID_POINT_OF_INTEREST 161

typedef struct __mavlink_point_of_interest_t 
{
	uint8_t type; ///< 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
	uint8_t color; ///< 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	uint8_t coordinate_system; ///< 0: global, 1:local
	uint16_t timeout; ///< 0: no timeout, >1: timeout in seconds
	float x; ///< X Position
	float y; ///< Y Position
	float z; ///< Z Position
	int8_t name[25]; ///< POI name

} mavlink_point_of_interest_t;

#define MAVLINK_MSG_POINT_OF_INTEREST_FIELD_NAME_LEN 25


/**
 * @brief Pack a point_of_interest message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param name POI name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_point_of_interest_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t type, uint8_t color, uint8_t coordinate_system, uint16_t timeout, float x, float y, float z, const int8_t* name)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST;

	i += put_uint8_t_by_index(type, i, msg->payload); // 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
	i += put_uint8_t_by_index(color, i, msg->payload); // 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	i += put_uint8_t_by_index(coordinate_system, i, msg->payload); // 0: global, 1:local
	i += put_uint16_t_by_index(timeout, i, msg->payload); // 0: no timeout, >1: timeout in seconds
	i += put_float_by_index(x, i, msg->payload); // X Position
	i += put_float_by_index(y, i, msg->payload); // Y Position
	i += put_float_by_index(z, i, msg->payload); // Z Position
	i += put_array_by_index(name, 25, i, msg->payload); // POI name

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a point_of_interest message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param name POI name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_point_of_interest_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t type, uint8_t color, uint8_t coordinate_system, uint16_t timeout, float x, float y, float z, const int8_t* name)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST;

	i += put_uint8_t_by_index(type, i, msg->payload); // 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
	i += put_uint8_t_by_index(color, i, msg->payload); // 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	i += put_uint8_t_by_index(coordinate_system, i, msg->payload); // 0: global, 1:local
	i += put_uint16_t_by_index(timeout, i, msg->payload); // 0: no timeout, >1: timeout in seconds
	i += put_float_by_index(x, i, msg->payload); // X Position
	i += put_float_by_index(y, i, msg->payload); // Y Position
	i += put_float_by_index(z, i, msg->payload); // Z Position
	i += put_array_by_index(name, 25, i, msg->payload); // POI name

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a point_of_interest struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param point_of_interest C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_point_of_interest_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_point_of_interest_t* point_of_interest)
{
	return mavlink_msg_point_of_interest_pack(system_id, component_id, msg, point_of_interest->type, point_of_interest->color, point_of_interest->coordinate_system, point_of_interest->timeout, point_of_interest->x, point_of_interest->y, point_of_interest->z, point_of_interest->name);
}

/**
 * @brief Send a point_of_interest message
 * @param chan MAVLink channel to send the message
 *
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param name POI name
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_point_of_interest_send(mavlink_channel_t chan, uint8_t type, uint8_t color, uint8_t coordinate_system, uint16_t timeout, float x, float y, float z, const int8_t* name)
{
	mavlink_message_t msg;
	mavlink_msg_point_of_interest_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, type, color, coordinate_system, timeout, x, y, z, name);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE POINT_OF_INTEREST UNPACKING

/**
 * @brief Get field type from point_of_interest message
 *
 * @return 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 */
static inline uint8_t mavlink_msg_point_of_interest_get_type(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field color from point_of_interest message
 *
 * @return 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 */
static inline uint8_t mavlink_msg_point_of_interest_get_color(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field coordinate_system from point_of_interest message
 *
 * @return 0: global, 1:local
 */
static inline uint8_t mavlink_msg_point_of_interest_get_coordinate_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field timeout from point_of_interest message
 *
 * @return 0: no timeout, >1: timeout in seconds
 */
static inline uint16_t mavlink_msg_point_of_interest_get_timeout(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field x from point_of_interest message
 *
 * @return X Position
 */
static inline float mavlink_msg_point_of_interest_get_x(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field y from point_of_interest message
 *
 * @return Y Position
 */
static inline float mavlink_msg_point_of_interest_get_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z from point_of_interest message
 *
 * @return Z Position
 */
static inline float mavlink_msg_point_of_interest_get_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field name from point_of_interest message
 *
 * @return POI name
 */
static inline uint16_t mavlink_msg_point_of_interest_get_name(const mavlink_message_t* msg, int8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(float)+sizeof(float)+sizeof(float), 25);
	return 25;
}

/**
 * @brief Decode a point_of_interest message into a struct
 *
 * @param msg The message to decode
 * @param point_of_interest C-struct to decode the message contents into
 */
static inline void mavlink_msg_point_of_interest_decode(const mavlink_message_t* msg, mavlink_point_of_interest_t* point_of_interest)
{
	point_of_interest->type = mavlink_msg_point_of_interest_get_type(msg);
	point_of_interest->color = mavlink_msg_point_of_interest_get_color(msg);
	point_of_interest->coordinate_system = mavlink_msg_point_of_interest_get_coordinate_system(msg);
	point_of_interest->timeout = mavlink_msg_point_of_interest_get_timeout(msg);
	point_of_interest->x = mavlink_msg_point_of_interest_get_x(msg);
	point_of_interest->y = mavlink_msg_point_of_interest_get_y(msg);
	point_of_interest->z = mavlink_msg_point_of_interest_get_z(msg);
	mavlink_msg_point_of_interest_get_name(msg, point_of_interest->name);
}

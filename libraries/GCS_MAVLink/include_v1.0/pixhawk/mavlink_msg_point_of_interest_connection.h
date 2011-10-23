// MESSAGE POINT_OF_INTEREST_CONNECTION PACKING

#define MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION 192

typedef struct __mavlink_point_of_interest_connection_t
{
 float xp1; ///< X1 Position
 float yp1; ///< Y1 Position
 float zp1; ///< Z1 Position
 float xp2; ///< X2 Position
 float yp2; ///< Y2 Position
 float zp2; ///< Z2 Position
 uint16_t timeout; ///< 0: no timeout, >1: timeout in seconds
 uint8_t type; ///< 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 uint8_t color; ///< 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 uint8_t coordinate_system; ///< 0: global, 1:local
 char name[26]; ///< POI connection name
} mavlink_point_of_interest_connection_t;

#define MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION_LEN 55
#define MAVLINK_MSG_ID_192_LEN 55

#define MAVLINK_MSG_POINT_OF_INTEREST_CONNECTION_FIELD_NAME_LEN 26

#define MAVLINK_MESSAGE_INFO_POINT_OF_INTEREST_CONNECTION { \
	"POINT_OF_INTEREST_CONNECTION", \
	11, \
	{  { "xp1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_point_of_interest_connection_t, xp1) }, \
         { "yp1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_point_of_interest_connection_t, yp1) }, \
         { "zp1", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_point_of_interest_connection_t, zp1) }, \
         { "xp2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_point_of_interest_connection_t, xp2) }, \
         { "yp2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_point_of_interest_connection_t, yp2) }, \
         { "zp2", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_point_of_interest_connection_t, zp2) }, \
         { "timeout", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_point_of_interest_connection_t, timeout) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_point_of_interest_connection_t, type) }, \
         { "color", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_point_of_interest_connection_t, color) }, \
         { "coordinate_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_point_of_interest_connection_t, coordinate_system) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 26, 29, offsetof(mavlink_point_of_interest_connection_t, name) }, \
         } \
}


/**
 * @brief Pack a point_of_interest_connection message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param xp1 X1 Position
 * @param yp1 Y1 Position
 * @param zp1 Z1 Position
 * @param xp2 X2 Position
 * @param yp2 Y2 Position
 * @param zp2 Z2 Position
 * @param name POI connection name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_point_of_interest_connection_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t type, uint8_t color, uint8_t coordinate_system, uint16_t timeout, float xp1, float yp1, float zp1, float xp2, float yp2, float zp2, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[55];
	_mav_put_float(buf, 0, xp1);
	_mav_put_float(buf, 4, yp1);
	_mav_put_float(buf, 8, zp1);
	_mav_put_float(buf, 12, xp2);
	_mav_put_float(buf, 16, yp2);
	_mav_put_float(buf, 20, zp2);
	_mav_put_uint16_t(buf, 24, timeout);
	_mav_put_uint8_t(buf, 26, type);
	_mav_put_uint8_t(buf, 27, color);
	_mav_put_uint8_t(buf, 28, coordinate_system);
	_mav_put_char_array(buf, 29, name, 26);
        memcpy(_MAV_PAYLOAD(msg), buf, 55);
#else
	mavlink_point_of_interest_connection_t packet;
	packet.xp1 = xp1;
	packet.yp1 = yp1;
	packet.zp1 = zp1;
	packet.xp2 = xp2;
	packet.yp2 = yp2;
	packet.zp2 = zp2;
	packet.timeout = timeout;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	memcpy(packet.name, name, sizeof(char)*26);
        memcpy(_MAV_PAYLOAD(msg), &packet, 55);
#endif

	msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION;
	return mavlink_finalize_message(msg, system_id, component_id, 55, 36);
}

/**
 * @brief Pack a point_of_interest_connection message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param xp1 X1 Position
 * @param yp1 Y1 Position
 * @param zp1 Z1 Position
 * @param xp2 X2 Position
 * @param yp2 Y2 Position
 * @param zp2 Z2 Position
 * @param name POI connection name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_point_of_interest_connection_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t type,uint8_t color,uint8_t coordinate_system,uint16_t timeout,float xp1,float yp1,float zp1,float xp2,float yp2,float zp2,const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[55];
	_mav_put_float(buf, 0, xp1);
	_mav_put_float(buf, 4, yp1);
	_mav_put_float(buf, 8, zp1);
	_mav_put_float(buf, 12, xp2);
	_mav_put_float(buf, 16, yp2);
	_mav_put_float(buf, 20, zp2);
	_mav_put_uint16_t(buf, 24, timeout);
	_mav_put_uint8_t(buf, 26, type);
	_mav_put_uint8_t(buf, 27, color);
	_mav_put_uint8_t(buf, 28, coordinate_system);
	_mav_put_char_array(buf, 29, name, 26);
        memcpy(_MAV_PAYLOAD(msg), buf, 55);
#else
	mavlink_point_of_interest_connection_t packet;
	packet.xp1 = xp1;
	packet.yp1 = yp1;
	packet.zp1 = zp1;
	packet.xp2 = xp2;
	packet.yp2 = yp2;
	packet.zp2 = zp2;
	packet.timeout = timeout;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	memcpy(packet.name, name, sizeof(char)*26);
        memcpy(_MAV_PAYLOAD(msg), &packet, 55);
#endif

	msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 55, 36);
}

/**
 * @brief Encode a point_of_interest_connection struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param point_of_interest_connection C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_point_of_interest_connection_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_point_of_interest_connection_t* point_of_interest_connection)
{
	return mavlink_msg_point_of_interest_connection_pack(system_id, component_id, msg, point_of_interest_connection->type, point_of_interest_connection->color, point_of_interest_connection->coordinate_system, point_of_interest_connection->timeout, point_of_interest_connection->xp1, point_of_interest_connection->yp1, point_of_interest_connection->zp1, point_of_interest_connection->xp2, point_of_interest_connection->yp2, point_of_interest_connection->zp2, point_of_interest_connection->name);
}

/**
 * @brief Send a point_of_interest_connection message
 * @param chan MAVLink channel to send the message
 *
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param xp1 X1 Position
 * @param yp1 Y1 Position
 * @param zp1 Z1 Position
 * @param xp2 X2 Position
 * @param yp2 Y2 Position
 * @param zp2 Z2 Position
 * @param name POI connection name
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_point_of_interest_connection_send(mavlink_channel_t chan, uint8_t type, uint8_t color, uint8_t coordinate_system, uint16_t timeout, float xp1, float yp1, float zp1, float xp2, float yp2, float zp2, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[55];
	_mav_put_float(buf, 0, xp1);
	_mav_put_float(buf, 4, yp1);
	_mav_put_float(buf, 8, zp1);
	_mav_put_float(buf, 12, xp2);
	_mav_put_float(buf, 16, yp2);
	_mav_put_float(buf, 20, zp2);
	_mav_put_uint16_t(buf, 24, timeout);
	_mav_put_uint8_t(buf, 26, type);
	_mav_put_uint8_t(buf, 27, color);
	_mav_put_uint8_t(buf, 28, coordinate_system);
	_mav_put_char_array(buf, 29, name, 26);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION, buf, 55, 36);
#else
	mavlink_point_of_interest_connection_t packet;
	packet.xp1 = xp1;
	packet.yp1 = yp1;
	packet.zp1 = zp1;
	packet.xp2 = xp2;
	packet.yp2 = yp2;
	packet.zp2 = zp2;
	packet.timeout = timeout;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	memcpy(packet.name, name, sizeof(char)*26);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION, (const char *)&packet, 55, 36);
#endif
}

#endif

// MESSAGE POINT_OF_INTEREST_CONNECTION UNPACKING


/**
 * @brief Get field type from point_of_interest_connection message
 *
 * @return 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 */
static inline uint8_t mavlink_msg_point_of_interest_connection_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field color from point_of_interest_connection message
 *
 * @return 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 */
static inline uint8_t mavlink_msg_point_of_interest_connection_get_color(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field coordinate_system from point_of_interest_connection message
 *
 * @return 0: global, 1:local
 */
static inline uint8_t mavlink_msg_point_of_interest_connection_get_coordinate_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field timeout from point_of_interest_connection message
 *
 * @return 0: no timeout, >1: timeout in seconds
 */
static inline uint16_t mavlink_msg_point_of_interest_connection_get_timeout(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field xp1 from point_of_interest_connection message
 *
 * @return X1 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_xp1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field yp1 from point_of_interest_connection message
 *
 * @return Y1 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_yp1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field zp1 from point_of_interest_connection message
 *
 * @return Z1 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_zp1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field xp2 from point_of_interest_connection message
 *
 * @return X2 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_xp2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yp2 from point_of_interest_connection message
 *
 * @return Y2 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_yp2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field zp2 from point_of_interest_connection message
 *
 * @return Z2 Position
 */
static inline float mavlink_msg_point_of_interest_connection_get_zp2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field name from point_of_interest_connection message
 *
 * @return POI connection name
 */
static inline uint16_t mavlink_msg_point_of_interest_connection_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 26,  29);
}

/**
 * @brief Decode a point_of_interest_connection message into a struct
 *
 * @param msg The message to decode
 * @param point_of_interest_connection C-struct to decode the message contents into
 */
static inline void mavlink_msg_point_of_interest_connection_decode(const mavlink_message_t* msg, mavlink_point_of_interest_connection_t* point_of_interest_connection)
{
#if MAVLINK_NEED_BYTE_SWAP
	point_of_interest_connection->xp1 = mavlink_msg_point_of_interest_connection_get_xp1(msg);
	point_of_interest_connection->yp1 = mavlink_msg_point_of_interest_connection_get_yp1(msg);
	point_of_interest_connection->zp1 = mavlink_msg_point_of_interest_connection_get_zp1(msg);
	point_of_interest_connection->xp2 = mavlink_msg_point_of_interest_connection_get_xp2(msg);
	point_of_interest_connection->yp2 = mavlink_msg_point_of_interest_connection_get_yp2(msg);
	point_of_interest_connection->zp2 = mavlink_msg_point_of_interest_connection_get_zp2(msg);
	point_of_interest_connection->timeout = mavlink_msg_point_of_interest_connection_get_timeout(msg);
	point_of_interest_connection->type = mavlink_msg_point_of_interest_connection_get_type(msg);
	point_of_interest_connection->color = mavlink_msg_point_of_interest_connection_get_color(msg);
	point_of_interest_connection->coordinate_system = mavlink_msg_point_of_interest_connection_get_coordinate_system(msg);
	mavlink_msg_point_of_interest_connection_get_name(msg, point_of_interest_connection->name);
#else
	memcpy(point_of_interest_connection, _MAV_PAYLOAD(msg), 55);
#endif
}

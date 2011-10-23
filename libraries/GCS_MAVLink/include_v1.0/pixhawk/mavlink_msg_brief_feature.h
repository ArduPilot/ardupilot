// MESSAGE BRIEF_FEATURE PACKING

#define MAVLINK_MSG_ID_BRIEF_FEATURE 195

typedef struct __mavlink_brief_feature_t
{
 float x; ///< x position in m
 float y; ///< y position in m
 float z; ///< z position in m
 float response; ///< Harris operator response at this location
 uint16_t size; ///< Size in pixels
 uint16_t orientation; ///< Orientation
 uint8_t orientation_assignment; ///< Orientation assignment 0: false, 1:true
 uint8_t descriptor[32]; ///< Descriptor
} mavlink_brief_feature_t;

#define MAVLINK_MSG_ID_BRIEF_FEATURE_LEN 53
#define MAVLINK_MSG_ID_195_LEN 53

#define MAVLINK_MSG_BRIEF_FEATURE_FIELD_DESCRIPTOR_LEN 32

#define MAVLINK_MESSAGE_INFO_BRIEF_FEATURE { \
	"BRIEF_FEATURE", \
	8, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_brief_feature_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_brief_feature_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_brief_feature_t, z) }, \
         { "response", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_brief_feature_t, response) }, \
         { "size", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_brief_feature_t, size) }, \
         { "orientation", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_brief_feature_t, orientation) }, \
         { "orientation_assignment", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_brief_feature_t, orientation_assignment) }, \
         { "descriptor", NULL, MAVLINK_TYPE_UINT8_T, 32, 21, offsetof(mavlink_brief_feature_t, descriptor) }, \
         } \
}


/**
 * @brief Pack a brief_feature message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x x position in m
 * @param y y position in m
 * @param z z position in m
 * @param orientation_assignment Orientation assignment 0: false, 1:true
 * @param size Size in pixels
 * @param orientation Orientation
 * @param descriptor Descriptor
 * @param response Harris operator response at this location
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_brief_feature_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float x, float y, float z, uint8_t orientation_assignment, uint16_t size, uint16_t orientation, const uint8_t *descriptor, float response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[53];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, response);
	_mav_put_uint16_t(buf, 16, size);
	_mav_put_uint16_t(buf, 18, orientation);
	_mav_put_uint8_t(buf, 20, orientation_assignment);
	_mav_put_uint8_t_array(buf, 21, descriptor, 32);
        memcpy(_MAV_PAYLOAD(msg), buf, 53);
#else
	mavlink_brief_feature_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.response = response;
	packet.size = size;
	packet.orientation = orientation;
	packet.orientation_assignment = orientation_assignment;
	memcpy(packet.descriptor, descriptor, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD(msg), &packet, 53);
#endif

	msg->msgid = MAVLINK_MSG_ID_BRIEF_FEATURE;
	return mavlink_finalize_message(msg, system_id, component_id, 53, 88);
}

/**
 * @brief Pack a brief_feature message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param x x position in m
 * @param y y position in m
 * @param z z position in m
 * @param orientation_assignment Orientation assignment 0: false, 1:true
 * @param size Size in pixels
 * @param orientation Orientation
 * @param descriptor Descriptor
 * @param response Harris operator response at this location
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_brief_feature_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float x,float y,float z,uint8_t orientation_assignment,uint16_t size,uint16_t orientation,const uint8_t *descriptor,float response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[53];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, response);
	_mav_put_uint16_t(buf, 16, size);
	_mav_put_uint16_t(buf, 18, orientation);
	_mav_put_uint8_t(buf, 20, orientation_assignment);
	_mav_put_uint8_t_array(buf, 21, descriptor, 32);
        memcpy(_MAV_PAYLOAD(msg), buf, 53);
#else
	mavlink_brief_feature_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.response = response;
	packet.size = size;
	packet.orientation = orientation;
	packet.orientation_assignment = orientation_assignment;
	memcpy(packet.descriptor, descriptor, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD(msg), &packet, 53);
#endif

	msg->msgid = MAVLINK_MSG_ID_BRIEF_FEATURE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 53, 88);
}

/**
 * @brief Encode a brief_feature struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param brief_feature C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_brief_feature_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_brief_feature_t* brief_feature)
{
	return mavlink_msg_brief_feature_pack(system_id, component_id, msg, brief_feature->x, brief_feature->y, brief_feature->z, brief_feature->orientation_assignment, brief_feature->size, brief_feature->orientation, brief_feature->descriptor, brief_feature->response);
}

/**
 * @brief Send a brief_feature message
 * @param chan MAVLink channel to send the message
 *
 * @param x x position in m
 * @param y y position in m
 * @param z z position in m
 * @param orientation_assignment Orientation assignment 0: false, 1:true
 * @param size Size in pixels
 * @param orientation Orientation
 * @param descriptor Descriptor
 * @param response Harris operator response at this location
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_brief_feature_send(mavlink_channel_t chan, float x, float y, float z, uint8_t orientation_assignment, uint16_t size, uint16_t orientation, const uint8_t *descriptor, float response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[53];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, response);
	_mav_put_uint16_t(buf, 16, size);
	_mav_put_uint16_t(buf, 18, orientation);
	_mav_put_uint8_t(buf, 20, orientation_assignment);
	_mav_put_uint8_t_array(buf, 21, descriptor, 32);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BRIEF_FEATURE, buf, 53, 88);
#else
	mavlink_brief_feature_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.response = response;
	packet.size = size;
	packet.orientation = orientation;
	packet.orientation_assignment = orientation_assignment;
	memcpy(packet.descriptor, descriptor, sizeof(uint8_t)*32);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BRIEF_FEATURE, (const char *)&packet, 53, 88);
#endif
}

#endif

// MESSAGE BRIEF_FEATURE UNPACKING


/**
 * @brief Get field x from brief_feature message
 *
 * @return x position in m
 */
static inline float mavlink_msg_brief_feature_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from brief_feature message
 *
 * @return y position in m
 */
static inline float mavlink_msg_brief_feature_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from brief_feature message
 *
 * @return z position in m
 */
static inline float mavlink_msg_brief_feature_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field orientation_assignment from brief_feature message
 *
 * @return Orientation assignment 0: false, 1:true
 */
static inline uint8_t mavlink_msg_brief_feature_get_orientation_assignment(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field size from brief_feature message
 *
 * @return Size in pixels
 */
static inline uint16_t mavlink_msg_brief_feature_get_size(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field orientation from brief_feature message
 *
 * @return Orientation
 */
static inline uint16_t mavlink_msg_brief_feature_get_orientation(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field descriptor from brief_feature message
 *
 * @return Descriptor
 */
static inline uint16_t mavlink_msg_brief_feature_get_descriptor(const mavlink_message_t* msg, uint8_t *descriptor)
{
	return _MAV_RETURN_uint8_t_array(msg, descriptor, 32,  21);
}

/**
 * @brief Get field response from brief_feature message
 *
 * @return Harris operator response at this location
 */
static inline float mavlink_msg_brief_feature_get_response(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a brief_feature message into a struct
 *
 * @param msg The message to decode
 * @param brief_feature C-struct to decode the message contents into
 */
static inline void mavlink_msg_brief_feature_decode(const mavlink_message_t* msg, mavlink_brief_feature_t* brief_feature)
{
#if MAVLINK_NEED_BYTE_SWAP
	brief_feature->x = mavlink_msg_brief_feature_get_x(msg);
	brief_feature->y = mavlink_msg_brief_feature_get_y(msg);
	brief_feature->z = mavlink_msg_brief_feature_get_z(msg);
	brief_feature->response = mavlink_msg_brief_feature_get_response(msg);
	brief_feature->size = mavlink_msg_brief_feature_get_size(msg);
	brief_feature->orientation = mavlink_msg_brief_feature_get_orientation(msg);
	brief_feature->orientation_assignment = mavlink_msg_brief_feature_get_orientation_assignment(msg);
	mavlink_msg_brief_feature_get_descriptor(msg, brief_feature->descriptor);
#else
	memcpy(brief_feature, _MAV_PAYLOAD(msg), 53);
#endif
}

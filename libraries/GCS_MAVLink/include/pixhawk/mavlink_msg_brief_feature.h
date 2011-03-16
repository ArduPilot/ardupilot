// MESSAGE BRIEF_FEATURE PACKING

#define MAVLINK_MSG_ID_BRIEF_FEATURE 172

typedef struct __mavlink_brief_feature_t 
{
	float x; ///< x position in m
	float y; ///< y position in m
	float z; ///< z position in m
	uint8_t orientation_assignment; ///< Orientation assignment 0: false, 1:true
	uint16_t size; ///< Size in pixels
	uint16_t orientation; ///< Orientation
	uint8_t descriptor[32]; ///< Descriptor
	float response; ///< Harris operator response at this location

} mavlink_brief_feature_t;

#define MAVLINK_MSG_BRIEF_FEATURE_FIELD_DESCRIPTOR_LEN 32


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
static inline uint16_t mavlink_msg_brief_feature_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float x, float y, float z, uint8_t orientation_assignment, uint16_t size, uint16_t orientation, const uint8_t* descriptor, float response)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_BRIEF_FEATURE;

	i += put_float_by_index(x, i, msg->payload); // x position in m
	i += put_float_by_index(y, i, msg->payload); // y position in m
	i += put_float_by_index(z, i, msg->payload); // z position in m
	i += put_uint8_t_by_index(orientation_assignment, i, msg->payload); // Orientation assignment 0: false, 1:true
	i += put_uint16_t_by_index(size, i, msg->payload); // Size in pixels
	i += put_uint16_t_by_index(orientation, i, msg->payload); // Orientation
	i += put_array_by_index((const int8_t*)descriptor, sizeof(uint8_t)*32, i, msg->payload); // Descriptor
	i += put_float_by_index(response, i, msg->payload); // Harris operator response at this location

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a brief_feature message
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
static inline uint16_t mavlink_msg_brief_feature_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, float x, float y, float z, uint8_t orientation_assignment, uint16_t size, uint16_t orientation, const uint8_t* descriptor, float response)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_BRIEF_FEATURE;

	i += put_float_by_index(x, i, msg->payload); // x position in m
	i += put_float_by_index(y, i, msg->payload); // y position in m
	i += put_float_by_index(z, i, msg->payload); // z position in m
	i += put_uint8_t_by_index(orientation_assignment, i, msg->payload); // Orientation assignment 0: false, 1:true
	i += put_uint16_t_by_index(size, i, msg->payload); // Size in pixels
	i += put_uint16_t_by_index(orientation, i, msg->payload); // Orientation
	i += put_array_by_index((const int8_t*)descriptor, sizeof(uint8_t)*32, i, msg->payload); // Descriptor
	i += put_float_by_index(response, i, msg->payload); // Harris operator response at this location

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
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

static inline void mavlink_msg_brief_feature_send(mavlink_channel_t chan, float x, float y, float z, uint8_t orientation_assignment, uint16_t size, uint16_t orientation, const uint8_t* descriptor, float response)
{
	mavlink_message_t msg;
	mavlink_msg_brief_feature_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, x, y, z, orientation_assignment, size, orientation, descriptor, response);
	mavlink_send_uart(chan, &msg);
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
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (float)r.f;
}

/**
 * @brief Get field y from brief_feature message
 *
 * @return y position in m
 */
static inline float mavlink_msg_brief_feature_get_y(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field z from brief_feature message
 *
 * @return z position in m
 */
static inline float mavlink_msg_brief_feature_get_z(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field orientation_assignment from brief_feature message
 *
 * @return Orientation assignment 0: false, 1:true
 */
static inline uint8_t mavlink_msg_brief_feature_get_orientation_assignment(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(float)+sizeof(float)+sizeof(float))[0];
}

/**
 * @brief Get field size from brief_feature message
 *
 * @return Size in pixels
 */
static inline uint16_t mavlink_msg_brief_feature_get_size(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field orientation from brief_feature message
 *
 * @return Orientation
 */
static inline uint16_t mavlink_msg_brief_feature_get_orientation(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field descriptor from brief_feature message
 *
 * @return Descriptor
 */
static inline uint16_t mavlink_msg_brief_feature_get_descriptor(const mavlink_message_t* msg, uint8_t* r_data)
{

	memcpy(r_data, msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t), sizeof(uint8_t)*32);
	return sizeof(uint8_t)*32;
}

/**
 * @brief Get field response from brief_feature message
 *
 * @return Harris operator response at this location
 */
static inline float mavlink_msg_brief_feature_get_response(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)*32)[0];
	r.b[2] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)*32)[1];
	r.b[1] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)*32)[2];
	r.b[0] = (msg->payload+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t)+sizeof(uint8_t)*32)[3];
	return (float)r.f;
}

/**
 * @brief Decode a brief_feature message into a struct
 *
 * @param msg The message to decode
 * @param brief_feature C-struct to decode the message contents into
 */
static inline void mavlink_msg_brief_feature_decode(const mavlink_message_t* msg, mavlink_brief_feature_t* brief_feature)
{
	brief_feature->x = mavlink_msg_brief_feature_get_x(msg);
	brief_feature->y = mavlink_msg_brief_feature_get_y(msg);
	brief_feature->z = mavlink_msg_brief_feature_get_z(msg);
	brief_feature->orientation_assignment = mavlink_msg_brief_feature_get_orientation_assignment(msg);
	brief_feature->size = mavlink_msg_brief_feature_get_size(msg);
	brief_feature->orientation = mavlink_msg_brief_feature_get_orientation(msg);
	mavlink_msg_brief_feature_get_descriptor(msg, brief_feature->descriptor);
	brief_feature->response = mavlink_msg_brief_feature_get_response(msg);
}

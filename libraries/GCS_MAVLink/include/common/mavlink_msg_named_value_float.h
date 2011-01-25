// MESSAGE NAMED_VALUE_FLOAT PACKING

#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT 252

typedef struct __mavlink_named_value_float_t 
{
	char name[10]; ///< Name of the debug variable
	float value; ///< Floating point value

} mavlink_named_value_float_t;

#define MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN 10


/**
 * @brief Pack a named_value_float message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name Name of the debug variable
 * @param value Floating point value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_value_float_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const char* name, float value)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;

	i += put_array_by_index((const int8_t*)name, sizeof(char)*10, i, msg->payload); // Name of the debug variable
	i += put_float_by_index(value, i, msg->payload); // Floating point value

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a named_value_float message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param name Name of the debug variable
 * @param value Floating point value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_value_float_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const char* name, float value)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;

	i += put_array_by_index((const int8_t*)name, sizeof(char)*10, i, msg->payload); // Name of the debug variable
	i += put_float_by_index(value, i, msg->payload); // Floating point value

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a named_value_float struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param named_value_float C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_value_float_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_named_value_float_t* named_value_float)
{
	return mavlink_msg_named_value_float_pack(system_id, component_id, msg, named_value_float->name, named_value_float->value);
}

/**
 * @brief Send a named_value_float message
 * @param chan MAVLink channel to send the message
 *
 * @param name Name of the debug variable
 * @param value Floating point value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_named_value_float_send(mavlink_channel_t chan, const char* name, float value)
{
	mavlink_message_t msg;
	mavlink_msg_named_value_float_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, name, value);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE NAMED_VALUE_FLOAT UNPACKING

/**
 * @brief Get field name from named_value_float message
 *
 * @return Name of the debug variable
 */
static inline uint16_t mavlink_msg_named_value_float_get_name(const mavlink_message_t* msg, char* r_data)
{

	memcpy(r_data, msg->payload, sizeof(char)*10);
	return sizeof(char)*10;
}

/**
 * @brief Get field value from named_value_float message
 *
 * @return Floating point value
 */
static inline float mavlink_msg_named_value_float_get_value(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(char)*10)[0];
	r.b[2] = (msg->payload+sizeof(char)*10)[1];
	r.b[1] = (msg->payload+sizeof(char)*10)[2];
	r.b[0] = (msg->payload+sizeof(char)*10)[3];
	return (float)r.f;
}

/**
 * @brief Decode a named_value_float message into a struct
 *
 * @param msg The message to decode
 * @param named_value_float C-struct to decode the message contents into
 */
static inline void mavlink_msg_named_value_float_decode(const mavlink_message_t* msg, mavlink_named_value_float_t* named_value_float)
{
	mavlink_msg_named_value_float_get_name(msg, named_value_float->name);
	named_value_float->value = mavlink_msg_named_value_float_get_value(msg);
}

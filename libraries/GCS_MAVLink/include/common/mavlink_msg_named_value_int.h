// MESSAGE NAMED_VALUE_INT PACKING

#define MAVLINK_MSG_ID_NAMED_VALUE_INT 253

typedef struct __mavlink_named_value_int_t 
{
	char name[10]; ///< Name of the debug variable
	int32_t value; ///< Signed integer value

} mavlink_named_value_int_t;

#define MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN 10


/**
 * @brief Pack a named_value_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name Name of the debug variable
 * @param value Signed integer value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_value_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const char* name, int32_t value)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;

	i += put_array_by_index((const int8_t*)name, sizeof(char)*10, i, msg->payload); // Name of the debug variable
	i += put_int32_t_by_index(value, i, msg->payload); // Signed integer value

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a named_value_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param name Name of the debug variable
 * @param value Signed integer value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_value_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const char* name, int32_t value)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;

	i += put_array_by_index((const int8_t*)name, sizeof(char)*10, i, msg->payload); // Name of the debug variable
	i += put_int32_t_by_index(value, i, msg->payload); // Signed integer value

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a named_value_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param named_value_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_value_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_named_value_int_t* named_value_int)
{
	return mavlink_msg_named_value_int_pack(system_id, component_id, msg, named_value_int->name, named_value_int->value);
}

/**
 * @brief Send a named_value_int message
 * @param chan MAVLink channel to send the message
 *
 * @param name Name of the debug variable
 * @param value Signed integer value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_named_value_int_send(mavlink_channel_t chan, const char* name, int32_t value)
{
	mavlink_message_t msg;
	mavlink_msg_named_value_int_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, name, value);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE NAMED_VALUE_INT UNPACKING

/**
 * @brief Get field name from named_value_int message
 *
 * @return Name of the debug variable
 */
static inline uint16_t mavlink_msg_named_value_int_get_name(const mavlink_message_t* msg, char* r_data)
{

	memcpy(r_data, msg->payload, sizeof(char)*10);
	return sizeof(char)*10;
}

/**
 * @brief Get field value from named_value_int message
 *
 * @return Signed integer value
 */
static inline int32_t mavlink_msg_named_value_int_get_value(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(char)*10)[0];
	r.b[2] = (msg->payload+sizeof(char)*10)[1];
	r.b[1] = (msg->payload+sizeof(char)*10)[2];
	r.b[0] = (msg->payload+sizeof(char)*10)[3];
	return (int32_t)r.i;
}

/**
 * @brief Decode a named_value_int message into a struct
 *
 * @param msg The message to decode
 * @param named_value_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_named_value_int_decode(const mavlink_message_t* msg, mavlink_named_value_int_t* named_value_int)
{
	mavlink_msg_named_value_int_get_name(msg, named_value_int->name);
	named_value_int->value = mavlink_msg_named_value_int_get_value(msg);
}

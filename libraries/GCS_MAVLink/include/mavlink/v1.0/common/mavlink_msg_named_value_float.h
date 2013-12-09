// MESSAGE NAMED_VALUE_FLOAT PACKING

#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT 251

typedef struct __mavlink_named_value_float_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float value; ///< Floating point value
 char name[10]; ///< Name of the debug variable
} mavlink_named_value_float_t;

#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN 18
#define MAVLINK_MSG_ID_251_LEN 18

#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_CRC 170
#define MAVLINK_MSG_ID_251_CRC 170

#define MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN 10

#define MAVLINK_MESSAGE_INFO_NAMED_VALUE_FLOAT { \
	"NAMED_VALUE_FLOAT", \
	3, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_named_value_float_t, time_boot_ms) }, \
         { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_named_value_float_t, value) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 8, offsetof(mavlink_named_value_float_t, name) }, \
         } \
}


/**
 * @brief Pack a named_value_float message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param name Name of the debug variable
 * @param value Floating point value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_value_float_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, const char *name, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, value);
	_mav_put_char_array(buf, 8, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN);
#else
	mavlink_named_value_float_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN);
#endif
}

/**
 * @brief Pack a named_value_float message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param name Name of the debug variable
 * @param value Floating point value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_value_float_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,const char *name,float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, value);
	_mav_put_char_array(buf, 8, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN);
#else
	mavlink_named_value_float_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN);
#endif
}

/**
 * @brief Encode a named_value_float struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param named_value_float C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_value_float_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_named_value_float_t* named_value_float)
{
	return mavlink_msg_named_value_float_pack(system_id, component_id, msg, named_value_float->time_boot_ms, named_value_float->name, named_value_float->value);
}

/**
 * @brief Encode a named_value_float struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param named_value_float C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_value_float_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_named_value_float_t* named_value_float)
{
	return mavlink_msg_named_value_float_pack_chan(system_id, component_id, chan, msg, named_value_float->time_boot_ms, named_value_float->name, named_value_float->value);
}

/**
 * @brief Send a named_value_float message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param name Name of the debug variable
 * @param value Floating point value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_named_value_float_send(mavlink_channel_t chan, uint32_t time_boot_ms, const char *name, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, value);
	_mav_put_char_array(buf, 8, name, 10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, buf, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, buf, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN);
#endif
#else
	mavlink_named_value_float_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, (const char *)&packet, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, (const char *)&packet, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN);
#endif
#endif
}

#endif

// MESSAGE NAMED_VALUE_FLOAT UNPACKING


/**
 * @brief Get field time_boot_ms from named_value_float message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_named_value_float_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field name from named_value_float message
 *
 * @return Name of the debug variable
 */
static inline uint16_t mavlink_msg_named_value_float_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 10,  8);
}

/**
 * @brief Get field value from named_value_float message
 *
 * @return Floating point value
 */
static inline float mavlink_msg_named_value_float_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a named_value_float message into a struct
 *
 * @param msg The message to decode
 * @param named_value_float C-struct to decode the message contents into
 */
static inline void mavlink_msg_named_value_float_decode(const mavlink_message_t* msg, mavlink_named_value_float_t* named_value_float)
{
#if MAVLINK_NEED_BYTE_SWAP
	named_value_float->time_boot_ms = mavlink_msg_named_value_float_get_time_boot_ms(msg);
	named_value_float->value = mavlink_msg_named_value_float_get_value(msg);
	mavlink_msg_named_value_float_get_name(msg, named_value_float->name);
#else
	memcpy(named_value_float, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN);
#endif
}

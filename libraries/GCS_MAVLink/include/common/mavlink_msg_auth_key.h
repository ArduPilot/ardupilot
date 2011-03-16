// MESSAGE AUTH_KEY PACKING

#define MAVLINK_MSG_ID_AUTH_KEY 7

typedef struct __mavlink_auth_key_t 
{
	char key[32]; ///< key

} mavlink_auth_key_t;

#define MAVLINK_MSG_AUTH_KEY_FIELD_KEY_LEN 32


/**
 * @brief Pack a auth_key message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param key key
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auth_key_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const char* key)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_AUTH_KEY;

	i += put_array_by_index((const int8_t*)key, sizeof(char)*32, i, msg->payload); // key

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a auth_key message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param key key
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_auth_key_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const char* key)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_AUTH_KEY;

	i += put_array_by_index((const int8_t*)key, sizeof(char)*32, i, msg->payload); // key

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a auth_key struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auth_key C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_auth_key_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_auth_key_t* auth_key)
{
	return mavlink_msg_auth_key_pack(system_id, component_id, msg, auth_key->key);
}

/**
 * @brief Send a auth_key message
 * @param chan MAVLink channel to send the message
 *
 * @param key key
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_auth_key_send(mavlink_channel_t chan, const char* key)
{
	mavlink_message_t msg;
	mavlink_msg_auth_key_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, key);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE AUTH_KEY UNPACKING

/**
 * @brief Get field key from auth_key message
 *
 * @return key
 */
static inline uint16_t mavlink_msg_auth_key_get_key(const mavlink_message_t* msg, char* r_data)
{

	memcpy(r_data, msg->payload, sizeof(char)*32);
	return sizeof(char)*32;
}

/**
 * @brief Decode a auth_key message into a struct
 *
 * @param msg The message to decode
 * @param auth_key C-struct to decode the message contents into
 */
static inline void mavlink_msg_auth_key_decode(const mavlink_message_t* msg, mavlink_auth_key_t* auth_key)
{
	mavlink_msg_auth_key_get_key(msg, auth_key->key);
}

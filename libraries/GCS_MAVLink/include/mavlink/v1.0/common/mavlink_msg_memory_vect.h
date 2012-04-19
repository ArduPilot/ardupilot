// MESSAGE MEMORY_VECT PACKING

#define MAVLINK_MSG_ID_MEMORY_VECT 249

typedef struct __mavlink_memory_vect_t
{
 uint16_t address; ///< Starting address of the debug variables
 uint8_t ver; ///< Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
 uint8_t type; ///< Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
 int8_t value[32]; ///< Memory contents at specified address
} mavlink_memory_vect_t;

#define MAVLINK_MSG_ID_MEMORY_VECT_LEN 36
#define MAVLINK_MSG_ID_249_LEN 36

#define MAVLINK_MSG_MEMORY_VECT_FIELD_VALUE_LEN 32

#define MAVLINK_MESSAGE_INFO_MEMORY_VECT { \
	"MEMORY_VECT", \
	4, \
	{  { "address", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_memory_vect_t, address) }, \
         { "ver", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_memory_vect_t, ver) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_memory_vect_t, type) }, \
         { "value", NULL, MAVLINK_TYPE_INT8_T, 32, 4, offsetof(mavlink_memory_vect_t, value) }, \
         } \
}


/**
 * @brief Pack a memory_vect message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param address Starting address of the debug variables
 * @param ver Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
 * @param type Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
 * @param value Memory contents at specified address
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_memory_vect_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t address, uint8_t ver, uint8_t type, const int8_t *value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_uint16_t(buf, 0, address);
	_mav_put_uint8_t(buf, 2, ver);
	_mav_put_uint8_t(buf, 3, type);
	_mav_put_int8_t_array(buf, 4, value, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 36);
#else
	mavlink_memory_vect_t packet;
	packet.address = address;
	packet.ver = ver;
	packet.type = type;
	mav_array_memcpy(packet.value, value, sizeof(int8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_MEMORY_VECT;
	return mavlink_finalize_message(msg, system_id, component_id, 36, 204);
}

/**
 * @brief Pack a memory_vect message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param address Starting address of the debug variables
 * @param ver Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
 * @param type Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
 * @param value Memory contents at specified address
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_memory_vect_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t address,uint8_t ver,uint8_t type,const int8_t *value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_uint16_t(buf, 0, address);
	_mav_put_uint8_t(buf, 2, ver);
	_mav_put_uint8_t(buf, 3, type);
	_mav_put_int8_t_array(buf, 4, value, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 36);
#else
	mavlink_memory_vect_t packet;
	packet.address = address;
	packet.ver = ver;
	packet.type = type;
	mav_array_memcpy(packet.value, value, sizeof(int8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_MEMORY_VECT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 204);
}

/**
 * @brief Encode a memory_vect struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param memory_vect C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_memory_vect_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_memory_vect_t* memory_vect)
{
	return mavlink_msg_memory_vect_pack(system_id, component_id, msg, memory_vect->address, memory_vect->ver, memory_vect->type, memory_vect->value);
}

/**
 * @brief Send a memory_vect message
 * @param chan MAVLink channel to send the message
 *
 * @param address Starting address of the debug variables
 * @param ver Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
 * @param type Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
 * @param value Memory contents at specified address
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_memory_vect_send(mavlink_channel_t chan, uint16_t address, uint8_t ver, uint8_t type, const int8_t *value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_uint16_t(buf, 0, address);
	_mav_put_uint8_t(buf, 2, ver);
	_mav_put_uint8_t(buf, 3, type);
	_mav_put_int8_t_array(buf, 4, value, 32);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEMORY_VECT, buf, 36, 204);
#else
	mavlink_memory_vect_t packet;
	packet.address = address;
	packet.ver = ver;
	packet.type = type;
	mav_array_memcpy(packet.value, value, sizeof(int8_t)*32);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEMORY_VECT, (const char *)&packet, 36, 204);
#endif
}

#endif

// MESSAGE MEMORY_VECT UNPACKING


/**
 * @brief Get field address from memory_vect message
 *
 * @return Starting address of the debug variables
 */
static inline uint16_t mavlink_msg_memory_vect_get_address(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field ver from memory_vect message
 *
 * @return Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
 */
static inline uint8_t mavlink_msg_memory_vect_get_ver(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field type from memory_vect message
 *
 * @return Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
 */
static inline uint8_t mavlink_msg_memory_vect_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field value from memory_vect message
 *
 * @return Memory contents at specified address
 */
static inline uint16_t mavlink_msg_memory_vect_get_value(const mavlink_message_t* msg, int8_t *value)
{
	return _MAV_RETURN_int8_t_array(msg, value, 32,  4);
}

/**
 * @brief Decode a memory_vect message into a struct
 *
 * @param msg The message to decode
 * @param memory_vect C-struct to decode the message contents into
 */
static inline void mavlink_msg_memory_vect_decode(const mavlink_message_t* msg, mavlink_memory_vect_t* memory_vect)
{
#if MAVLINK_NEED_BYTE_SWAP
	memory_vect->address = mavlink_msg_memory_vect_get_address(msg);
	memory_vect->ver = mavlink_msg_memory_vect_get_ver(msg);
	memory_vect->type = mavlink_msg_memory_vect_get_type(msg);
	mavlink_msg_memory_vect_get_value(msg, memory_vect->value);
#else
	memcpy(memory_vect, _MAV_PAYLOAD(msg), 36);
#endif
}

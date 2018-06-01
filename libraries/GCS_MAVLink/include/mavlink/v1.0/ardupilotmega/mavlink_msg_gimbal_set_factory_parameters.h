// MESSAGE GIMBAL_SET_FACTORY_PARAMETERS PACKING

#define MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS 206

typedef struct __mavlink_gimbal_set_factory_parameters_t
{
 uint32_t magic_1; /*< Magic number 1 for validation*/
 uint32_t magic_2; /*< Magic number 2 for validation*/
 uint32_t magic_3; /*< Magic number 3 for validation*/
 uint32_t serial_number_pt_1; /*< Unit Serial Number Part 1 (part code, design, language/country)*/
 uint32_t serial_number_pt_2; /*< Unit Serial Number Part 2 (option, year, month)*/
 uint32_t serial_number_pt_3; /*< Unit Serial Number Part 3 (incrementing serial number per month)*/
 uint16_t assembly_year; /*< Assembly Date Year*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t assembly_month; /*< Assembly Date Month*/
 uint8_t assembly_day; /*< Assembly Date Day*/
 uint8_t assembly_hour; /*< Assembly Time Hour*/
 uint8_t assembly_minute; /*< Assembly Time Minute*/
 uint8_t assembly_second; /*< Assembly Time Second*/
} mavlink_gimbal_set_factory_parameters_t;

#define MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN 33
#define MAVLINK_MSG_ID_206_LEN 33

#define MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_CRC 112
#define MAVLINK_MSG_ID_206_CRC 112



#define MAVLINK_MESSAGE_INFO_GIMBAL_SET_FACTORY_PARAMETERS { \
	"GIMBAL_SET_FACTORY_PARAMETERS", \
	14, \
	{  { "magic_1", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gimbal_set_factory_parameters_t, magic_1) }, \
         { "magic_2", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gimbal_set_factory_parameters_t, magic_2) }, \
         { "magic_3", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gimbal_set_factory_parameters_t, magic_3) }, \
         { "serial_number_pt_1", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_gimbal_set_factory_parameters_t, serial_number_pt_1) }, \
         { "serial_number_pt_2", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_gimbal_set_factory_parameters_t, serial_number_pt_2) }, \
         { "serial_number_pt_3", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_gimbal_set_factory_parameters_t, serial_number_pt_3) }, \
         { "assembly_year", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_gimbal_set_factory_parameters_t, assembly_year) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_gimbal_set_factory_parameters_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_gimbal_set_factory_parameters_t, target_component) }, \
         { "assembly_month", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_gimbal_set_factory_parameters_t, assembly_month) }, \
         { "assembly_day", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_gimbal_set_factory_parameters_t, assembly_day) }, \
         { "assembly_hour", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_gimbal_set_factory_parameters_t, assembly_hour) }, \
         { "assembly_minute", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_gimbal_set_factory_parameters_t, assembly_minute) }, \
         { "assembly_second", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_gimbal_set_factory_parameters_t, assembly_second) }, \
         } \
}


/**
 * @brief Pack a gimbal_set_factory_parameters message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param magic_1 Magic number 1 for validation
 * @param magic_2 Magic number 2 for validation
 * @param magic_3 Magic number 3 for validation
 * @param assembly_year Assembly Date Year
 * @param assembly_month Assembly Date Month
 * @param assembly_day Assembly Date Day
 * @param assembly_hour Assembly Time Hour
 * @param assembly_minute Assembly Time Minute
 * @param assembly_second Assembly Time Second
 * @param serial_number_pt_1 Unit Serial Number Part 1 (part code, design, language/country)
 * @param serial_number_pt_2 Unit Serial Number Part 2 (option, year, month)
 * @param serial_number_pt_3 Unit Serial Number Part 3 (incrementing serial number per month)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_set_factory_parameters_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint32_t magic_1, uint32_t magic_2, uint32_t magic_3, uint16_t assembly_year, uint8_t assembly_month, uint8_t assembly_day, uint8_t assembly_hour, uint8_t assembly_minute, uint8_t assembly_second, uint32_t serial_number_pt_1, uint32_t serial_number_pt_2, uint32_t serial_number_pt_3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN];
	_mav_put_uint32_t(buf, 0, magic_1);
	_mav_put_uint32_t(buf, 4, magic_2);
	_mav_put_uint32_t(buf, 8, magic_3);
	_mav_put_uint32_t(buf, 12, serial_number_pt_1);
	_mav_put_uint32_t(buf, 16, serial_number_pt_2);
	_mav_put_uint32_t(buf, 20, serial_number_pt_3);
	_mav_put_uint16_t(buf, 24, assembly_year);
	_mav_put_uint8_t(buf, 26, target_system);
	_mav_put_uint8_t(buf, 27, target_component);
	_mav_put_uint8_t(buf, 28, assembly_month);
	_mav_put_uint8_t(buf, 29, assembly_day);
	_mav_put_uint8_t(buf, 30, assembly_hour);
	_mav_put_uint8_t(buf, 31, assembly_minute);
	_mav_put_uint8_t(buf, 32, assembly_second);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#else
	mavlink_gimbal_set_factory_parameters_t packet;
	packet.magic_1 = magic_1;
	packet.magic_2 = magic_2;
	packet.magic_3 = magic_3;
	packet.serial_number_pt_1 = serial_number_pt_1;
	packet.serial_number_pt_2 = serial_number_pt_2;
	packet.serial_number_pt_3 = serial_number_pt_3;
	packet.assembly_year = assembly_year;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.assembly_month = assembly_month;
	packet.assembly_day = assembly_day;
	packet.assembly_hour = assembly_hour;
	packet.assembly_minute = assembly_minute;
	packet.assembly_second = assembly_second;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#endif
}

/**
 * @brief Pack a gimbal_set_factory_parameters message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param magic_1 Magic number 1 for validation
 * @param magic_2 Magic number 2 for validation
 * @param magic_3 Magic number 3 for validation
 * @param assembly_year Assembly Date Year
 * @param assembly_month Assembly Date Month
 * @param assembly_day Assembly Date Day
 * @param assembly_hour Assembly Time Hour
 * @param assembly_minute Assembly Time Minute
 * @param assembly_second Assembly Time Second
 * @param serial_number_pt_1 Unit Serial Number Part 1 (part code, design, language/country)
 * @param serial_number_pt_2 Unit Serial Number Part 2 (option, year, month)
 * @param serial_number_pt_3 Unit Serial Number Part 3 (incrementing serial number per month)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_set_factory_parameters_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint32_t magic_1,uint32_t magic_2,uint32_t magic_3,uint16_t assembly_year,uint8_t assembly_month,uint8_t assembly_day,uint8_t assembly_hour,uint8_t assembly_minute,uint8_t assembly_second,uint32_t serial_number_pt_1,uint32_t serial_number_pt_2,uint32_t serial_number_pt_3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN];
	_mav_put_uint32_t(buf, 0, magic_1);
	_mav_put_uint32_t(buf, 4, magic_2);
	_mav_put_uint32_t(buf, 8, magic_3);
	_mav_put_uint32_t(buf, 12, serial_number_pt_1);
	_mav_put_uint32_t(buf, 16, serial_number_pt_2);
	_mav_put_uint32_t(buf, 20, serial_number_pt_3);
	_mav_put_uint16_t(buf, 24, assembly_year);
	_mav_put_uint8_t(buf, 26, target_system);
	_mav_put_uint8_t(buf, 27, target_component);
	_mav_put_uint8_t(buf, 28, assembly_month);
	_mav_put_uint8_t(buf, 29, assembly_day);
	_mav_put_uint8_t(buf, 30, assembly_hour);
	_mav_put_uint8_t(buf, 31, assembly_minute);
	_mav_put_uint8_t(buf, 32, assembly_second);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#else
	mavlink_gimbal_set_factory_parameters_t packet;
	packet.magic_1 = magic_1;
	packet.magic_2 = magic_2;
	packet.magic_3 = magic_3;
	packet.serial_number_pt_1 = serial_number_pt_1;
	packet.serial_number_pt_2 = serial_number_pt_2;
	packet.serial_number_pt_3 = serial_number_pt_3;
	packet.assembly_year = assembly_year;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.assembly_month = assembly_month;
	packet.assembly_day = assembly_day;
	packet.assembly_hour = assembly_hour;
	packet.assembly_minute = assembly_minute;
	packet.assembly_second = assembly_second;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#endif
}

/**
 * @brief Encode a gimbal_set_factory_parameters struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_set_factory_parameters C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_set_factory_parameters_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_set_factory_parameters_t* gimbal_set_factory_parameters)
{
	return mavlink_msg_gimbal_set_factory_parameters_pack(system_id, component_id, msg, gimbal_set_factory_parameters->target_system, gimbal_set_factory_parameters->target_component, gimbal_set_factory_parameters->magic_1, gimbal_set_factory_parameters->magic_2, gimbal_set_factory_parameters->magic_3, gimbal_set_factory_parameters->assembly_year, gimbal_set_factory_parameters->assembly_month, gimbal_set_factory_parameters->assembly_day, gimbal_set_factory_parameters->assembly_hour, gimbal_set_factory_parameters->assembly_minute, gimbal_set_factory_parameters->assembly_second, gimbal_set_factory_parameters->serial_number_pt_1, gimbal_set_factory_parameters->serial_number_pt_2, gimbal_set_factory_parameters->serial_number_pt_3);
}

/**
 * @brief Encode a gimbal_set_factory_parameters struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_set_factory_parameters C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_set_factory_parameters_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_set_factory_parameters_t* gimbal_set_factory_parameters)
{
	return mavlink_msg_gimbal_set_factory_parameters_pack_chan(system_id, component_id, chan, msg, gimbal_set_factory_parameters->target_system, gimbal_set_factory_parameters->target_component, gimbal_set_factory_parameters->magic_1, gimbal_set_factory_parameters->magic_2, gimbal_set_factory_parameters->magic_3, gimbal_set_factory_parameters->assembly_year, gimbal_set_factory_parameters->assembly_month, gimbal_set_factory_parameters->assembly_day, gimbal_set_factory_parameters->assembly_hour, gimbal_set_factory_parameters->assembly_minute, gimbal_set_factory_parameters->assembly_second, gimbal_set_factory_parameters->serial_number_pt_1, gimbal_set_factory_parameters->serial_number_pt_2, gimbal_set_factory_parameters->serial_number_pt_3);
}

/**
 * @brief Send a gimbal_set_factory_parameters message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param magic_1 Magic number 1 for validation
 * @param magic_2 Magic number 2 for validation
 * @param magic_3 Magic number 3 for validation
 * @param assembly_year Assembly Date Year
 * @param assembly_month Assembly Date Month
 * @param assembly_day Assembly Date Day
 * @param assembly_hour Assembly Time Hour
 * @param assembly_minute Assembly Time Minute
 * @param assembly_second Assembly Time Second
 * @param serial_number_pt_1 Unit Serial Number Part 1 (part code, design, language/country)
 * @param serial_number_pt_2 Unit Serial Number Part 2 (option, year, month)
 * @param serial_number_pt_3 Unit Serial Number Part 3 (incrementing serial number per month)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_set_factory_parameters_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t magic_1, uint32_t magic_2, uint32_t magic_3, uint16_t assembly_year, uint8_t assembly_month, uint8_t assembly_day, uint8_t assembly_hour, uint8_t assembly_minute, uint8_t assembly_second, uint32_t serial_number_pt_1, uint32_t serial_number_pt_2, uint32_t serial_number_pt_3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN];
	_mav_put_uint32_t(buf, 0, magic_1);
	_mav_put_uint32_t(buf, 4, magic_2);
	_mav_put_uint32_t(buf, 8, magic_3);
	_mav_put_uint32_t(buf, 12, serial_number_pt_1);
	_mav_put_uint32_t(buf, 16, serial_number_pt_2);
	_mav_put_uint32_t(buf, 20, serial_number_pt_3);
	_mav_put_uint16_t(buf, 24, assembly_year);
	_mav_put_uint8_t(buf, 26, target_system);
	_mav_put_uint8_t(buf, 27, target_component);
	_mav_put_uint8_t(buf, 28, assembly_month);
	_mav_put_uint8_t(buf, 29, assembly_day);
	_mav_put_uint8_t(buf, 30, assembly_hour);
	_mav_put_uint8_t(buf, 31, assembly_minute);
	_mav_put_uint8_t(buf, 32, assembly_second);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS, buf, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS, buf, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#endif
#else
	mavlink_gimbal_set_factory_parameters_t packet;
	packet.magic_1 = magic_1;
	packet.magic_2 = magic_2;
	packet.magic_3 = magic_3;
	packet.serial_number_pt_1 = serial_number_pt_1;
	packet.serial_number_pt_2 = serial_number_pt_2;
	packet.serial_number_pt_3 = serial_number_pt_3;
	packet.assembly_year = assembly_year;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.assembly_month = assembly_month;
	packet.assembly_day = assembly_day;
	packet.assembly_hour = assembly_hour;
	packet.assembly_minute = assembly_minute;
	packet.assembly_second = assembly_second;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_set_factory_parameters_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t magic_1, uint32_t magic_2, uint32_t magic_3, uint16_t assembly_year, uint8_t assembly_month, uint8_t assembly_day, uint8_t assembly_hour, uint8_t assembly_minute, uint8_t assembly_second, uint32_t serial_number_pt_1, uint32_t serial_number_pt_2, uint32_t serial_number_pt_3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, magic_1);
	_mav_put_uint32_t(buf, 4, magic_2);
	_mav_put_uint32_t(buf, 8, magic_3);
	_mav_put_uint32_t(buf, 12, serial_number_pt_1);
	_mav_put_uint32_t(buf, 16, serial_number_pt_2);
	_mav_put_uint32_t(buf, 20, serial_number_pt_3);
	_mav_put_uint16_t(buf, 24, assembly_year);
	_mav_put_uint8_t(buf, 26, target_system);
	_mav_put_uint8_t(buf, 27, target_component);
	_mav_put_uint8_t(buf, 28, assembly_month);
	_mav_put_uint8_t(buf, 29, assembly_day);
	_mav_put_uint8_t(buf, 30, assembly_hour);
	_mav_put_uint8_t(buf, 31, assembly_minute);
	_mav_put_uint8_t(buf, 32, assembly_second);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS, buf, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS, buf, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#endif
#else
	mavlink_gimbal_set_factory_parameters_t *packet = (mavlink_gimbal_set_factory_parameters_t *)msgbuf;
	packet->magic_1 = magic_1;
	packet->magic_2 = magic_2;
	packet->magic_3 = magic_3;
	packet->serial_number_pt_1 = serial_number_pt_1;
	packet->serial_number_pt_2 = serial_number_pt_2;
	packet->serial_number_pt_3 = serial_number_pt_3;
	packet->assembly_year = assembly_year;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->assembly_month = assembly_month;
	packet->assembly_day = assembly_day;
	packet->assembly_hour = assembly_hour;
	packet->assembly_minute = assembly_minute;
	packet->assembly_second = assembly_second;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GIMBAL_SET_FACTORY_PARAMETERS UNPACKING


/**
 * @brief Get field target_system from gimbal_set_factory_parameters message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gimbal_set_factory_parameters_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field target_component from gimbal_set_factory_parameters message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gimbal_set_factory_parameters_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field magic_1 from gimbal_set_factory_parameters message
 *
 * @return Magic number 1 for validation
 */
static inline uint32_t mavlink_msg_gimbal_set_factory_parameters_get_magic_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field magic_2 from gimbal_set_factory_parameters message
 *
 * @return Magic number 2 for validation
 */
static inline uint32_t mavlink_msg_gimbal_set_factory_parameters_get_magic_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field magic_3 from gimbal_set_factory_parameters message
 *
 * @return Magic number 3 for validation
 */
static inline uint32_t mavlink_msg_gimbal_set_factory_parameters_get_magic_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field assembly_year from gimbal_set_factory_parameters message
 *
 * @return Assembly Date Year
 */
static inline uint16_t mavlink_msg_gimbal_set_factory_parameters_get_assembly_year(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field assembly_month from gimbal_set_factory_parameters message
 *
 * @return Assembly Date Month
 */
static inline uint8_t mavlink_msg_gimbal_set_factory_parameters_get_assembly_month(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field assembly_day from gimbal_set_factory_parameters message
 *
 * @return Assembly Date Day
 */
static inline uint8_t mavlink_msg_gimbal_set_factory_parameters_get_assembly_day(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field assembly_hour from gimbal_set_factory_parameters message
 *
 * @return Assembly Time Hour
 */
static inline uint8_t mavlink_msg_gimbal_set_factory_parameters_get_assembly_hour(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field assembly_minute from gimbal_set_factory_parameters message
 *
 * @return Assembly Time Minute
 */
static inline uint8_t mavlink_msg_gimbal_set_factory_parameters_get_assembly_minute(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field assembly_second from gimbal_set_factory_parameters message
 *
 * @return Assembly Time Second
 */
static inline uint8_t mavlink_msg_gimbal_set_factory_parameters_get_assembly_second(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field serial_number_pt_1 from gimbal_set_factory_parameters message
 *
 * @return Unit Serial Number Part 1 (part code, design, language/country)
 */
static inline uint32_t mavlink_msg_gimbal_set_factory_parameters_get_serial_number_pt_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field serial_number_pt_2 from gimbal_set_factory_parameters message
 *
 * @return Unit Serial Number Part 2 (option, year, month)
 */
static inline uint32_t mavlink_msg_gimbal_set_factory_parameters_get_serial_number_pt_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field serial_number_pt_3 from gimbal_set_factory_parameters message
 *
 * @return Unit Serial Number Part 3 (incrementing serial number per month)
 */
static inline uint32_t mavlink_msg_gimbal_set_factory_parameters_get_serial_number_pt_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Decode a gimbal_set_factory_parameters message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_set_factory_parameters C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_set_factory_parameters_decode(const mavlink_message_t* msg, mavlink_gimbal_set_factory_parameters_t* gimbal_set_factory_parameters)
{
#if MAVLINK_NEED_BYTE_SWAP
	gimbal_set_factory_parameters->magic_1 = mavlink_msg_gimbal_set_factory_parameters_get_magic_1(msg);
	gimbal_set_factory_parameters->magic_2 = mavlink_msg_gimbal_set_factory_parameters_get_magic_2(msg);
	gimbal_set_factory_parameters->magic_3 = mavlink_msg_gimbal_set_factory_parameters_get_magic_3(msg);
	gimbal_set_factory_parameters->serial_number_pt_1 = mavlink_msg_gimbal_set_factory_parameters_get_serial_number_pt_1(msg);
	gimbal_set_factory_parameters->serial_number_pt_2 = mavlink_msg_gimbal_set_factory_parameters_get_serial_number_pt_2(msg);
	gimbal_set_factory_parameters->serial_number_pt_3 = mavlink_msg_gimbal_set_factory_parameters_get_serial_number_pt_3(msg);
	gimbal_set_factory_parameters->assembly_year = mavlink_msg_gimbal_set_factory_parameters_get_assembly_year(msg);
	gimbal_set_factory_parameters->target_system = mavlink_msg_gimbal_set_factory_parameters_get_target_system(msg);
	gimbal_set_factory_parameters->target_component = mavlink_msg_gimbal_set_factory_parameters_get_target_component(msg);
	gimbal_set_factory_parameters->assembly_month = mavlink_msg_gimbal_set_factory_parameters_get_assembly_month(msg);
	gimbal_set_factory_parameters->assembly_day = mavlink_msg_gimbal_set_factory_parameters_get_assembly_day(msg);
	gimbal_set_factory_parameters->assembly_hour = mavlink_msg_gimbal_set_factory_parameters_get_assembly_hour(msg);
	gimbal_set_factory_parameters->assembly_minute = mavlink_msg_gimbal_set_factory_parameters_get_assembly_minute(msg);
	gimbal_set_factory_parameters->assembly_second = mavlink_msg_gimbal_set_factory_parameters_get_assembly_second(msg);
#else
	memcpy(gimbal_set_factory_parameters, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS_LEN);
#endif
}

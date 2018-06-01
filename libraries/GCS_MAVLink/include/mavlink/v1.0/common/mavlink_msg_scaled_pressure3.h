// MESSAGE SCALED_PRESSURE3 PACKING

#define MAVLINK_MSG_ID_SCALED_PRESSURE3 143

typedef struct __mavlink_scaled_pressure3_t
{
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float press_abs; /*< Absolute pressure (hectopascal)*/
 float press_diff; /*< Differential pressure 1 (hectopascal)*/
 int16_t temperature; /*< Temperature measurement (0.01 degrees celsius)*/
} mavlink_scaled_pressure3_t;

#define MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN 14
#define MAVLINK_MSG_ID_143_LEN 14

#define MAVLINK_MSG_ID_SCALED_PRESSURE3_CRC 131
#define MAVLINK_MSG_ID_143_CRC 131



#define MAVLINK_MESSAGE_INFO_SCALED_PRESSURE3 { \
	"SCALED_PRESSURE3", \
	4, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_scaled_pressure3_t, time_boot_ms) }, \
         { "press_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_scaled_pressure3_t, press_abs) }, \
         { "press_diff", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_scaled_pressure3_t, press_diff) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_scaled_pressure3_t, temperature) }, \
         } \
}


/**
 * @brief Pack a scaled_pressure3 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_pressure3_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, press_abs);
	_mav_put_float(buf, 8, press_diff);
	_mav_put_int16_t(buf, 12, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#else
	mavlink_scaled_pressure3_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SCALED_PRESSURE3;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE3_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#endif
}

/**
 * @brief Pack a scaled_pressure3 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_scaled_pressure3_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float press_abs,float press_diff,int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, press_abs);
	_mav_put_float(buf, 8, press_diff);
	_mav_put_int16_t(buf, 12, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#else
	mavlink_scaled_pressure3_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SCALED_PRESSURE3;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE3_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#endif
}

/**
 * @brief Encode a scaled_pressure3 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scaled_pressure3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_pressure3_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scaled_pressure3_t* scaled_pressure3)
{
	return mavlink_msg_scaled_pressure3_pack(system_id, component_id, msg, scaled_pressure3->time_boot_ms, scaled_pressure3->press_abs, scaled_pressure3->press_diff, scaled_pressure3->temperature);
}

/**
 * @brief Encode a scaled_pressure3 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param scaled_pressure3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_scaled_pressure3_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_scaled_pressure3_t* scaled_pressure3)
{
	return mavlink_msg_scaled_pressure3_pack_chan(system_id, component_id, chan, msg, scaled_pressure3->time_boot_ms, scaled_pressure3->press_abs, scaled_pressure3->press_diff, scaled_pressure3->temperature);
}

/**
 * @brief Send a scaled_pressure3 message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_scaled_pressure3_send(mavlink_channel_t chan, uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, press_abs);
	_mav_put_float(buf, 8, press_diff);
	_mav_put_int16_t(buf, 12, temperature);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE3, buf, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE3_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE3, buf, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#endif
#else
	mavlink_scaled_pressure3_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.temperature = temperature;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE3, (const char *)&packet, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE3_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE3, (const char *)&packet, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_scaled_pressure3_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, press_abs);
	_mav_put_float(buf, 8, press_diff);
	_mav_put_int16_t(buf, 12, temperature);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE3, buf, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE3_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE3, buf, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#endif
#else
	mavlink_scaled_pressure3_t *packet = (mavlink_scaled_pressure3_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->press_abs = press_abs;
	packet->press_diff = press_diff;
	packet->temperature = temperature;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE3, (const char *)packet, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN, MAVLINK_MSG_ID_SCALED_PRESSURE3_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE3, (const char *)packet, MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SCALED_PRESSURE3 UNPACKING


/**
 * @brief Get field time_boot_ms from scaled_pressure3 message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_scaled_pressure3_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field press_abs from scaled_pressure3 message
 *
 * @return Absolute pressure (hectopascal)
 */
static inline float mavlink_msg_scaled_pressure3_get_press_abs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field press_diff from scaled_pressure3 message
 *
 * @return Differential pressure 1 (hectopascal)
 */
static inline float mavlink_msg_scaled_pressure3_get_press_diff(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field temperature from scaled_pressure3 message
 *
 * @return Temperature measurement (0.01 degrees celsius)
 */
static inline int16_t mavlink_msg_scaled_pressure3_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Decode a scaled_pressure3 message into a struct
 *
 * @param msg The message to decode
 * @param scaled_pressure3 C-struct to decode the message contents into
 */
static inline void mavlink_msg_scaled_pressure3_decode(const mavlink_message_t* msg, mavlink_scaled_pressure3_t* scaled_pressure3)
{
#if MAVLINK_NEED_BYTE_SWAP
	scaled_pressure3->time_boot_ms = mavlink_msg_scaled_pressure3_get_time_boot_ms(msg);
	scaled_pressure3->press_abs = mavlink_msg_scaled_pressure3_get_press_abs(msg);
	scaled_pressure3->press_diff = mavlink_msg_scaled_pressure3_get_press_diff(msg);
	scaled_pressure3->temperature = mavlink_msg_scaled_pressure3_get_temperature(msg);
#else
	memcpy(scaled_pressure3, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SCALED_PRESSURE3_LEN);
#endif
}

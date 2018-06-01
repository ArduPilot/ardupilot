// MESSAGE RAW_PRESSURE PACKING

#define MAVLINK_MSG_ID_RAW_PRESSURE 28

typedef struct __mavlink_raw_pressure_t
{
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 int16_t press_abs; /*< Absolute pressure (raw)*/
 int16_t press_diff1; /*< Differential pressure 1 (raw)*/
 int16_t press_diff2; /*< Differential pressure 2 (raw)*/
 int16_t temperature; /*< Raw Temperature measurement (raw)*/
} mavlink_raw_pressure_t;

#define MAVLINK_MSG_ID_RAW_PRESSURE_LEN 16
#define MAVLINK_MSG_ID_28_LEN 16

#define MAVLINK_MSG_ID_RAW_PRESSURE_CRC 67
#define MAVLINK_MSG_ID_28_CRC 67



#define MAVLINK_MESSAGE_INFO_RAW_PRESSURE { \
	"RAW_PRESSURE", \
	5, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_raw_pressure_t, time_usec) }, \
         { "press_abs", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_raw_pressure_t, press_abs) }, \
         { "press_diff1", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_raw_pressure_t, press_diff1) }, \
         { "press_diff2", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_raw_pressure_t, press_diff2) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_raw_pressure_t, temperature) }, \
         } \
}


/**
 * @brief Pack a raw_pressure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (raw)
 * @param press_diff1 Differential pressure 1 (raw)
 * @param press_diff2 Differential pressure 2 (raw)
 * @param temperature Raw Temperature measurement (raw)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_pressure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, int16_t press_abs, int16_t press_diff1, int16_t press_diff2, int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RAW_PRESSURE_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int16_t(buf, 8, press_abs);
	_mav_put_int16_t(buf, 10, press_diff1);
	_mav_put_int16_t(buf, 12, press_diff2);
	_mav_put_int16_t(buf, 14, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#else
	mavlink_raw_pressure_t packet;
	packet.time_usec = time_usec;
	packet.press_abs = press_abs;
	packet.press_diff1 = press_diff1;
	packet.press_diff2 = press_diff2;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_PRESSURE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RAW_PRESSURE_LEN, MAVLINK_MSG_ID_RAW_PRESSURE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#endif
}

/**
 * @brief Pack a raw_pressure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (raw)
 * @param press_diff1 Differential pressure 1 (raw)
 * @param press_diff2 Differential pressure 2 (raw)
 * @param temperature Raw Temperature measurement (raw)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_pressure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,int16_t press_abs,int16_t press_diff1,int16_t press_diff2,int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RAW_PRESSURE_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int16_t(buf, 8, press_abs);
	_mav_put_int16_t(buf, 10, press_diff1);
	_mav_put_int16_t(buf, 12, press_diff2);
	_mav_put_int16_t(buf, 14, temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#else
	mavlink_raw_pressure_t packet;
	packet.time_usec = time_usec;
	packet.press_abs = press_abs;
	packet.press_diff1 = press_diff1;
	packet.press_diff2 = press_diff2;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_PRESSURE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RAW_PRESSURE_LEN, MAVLINK_MSG_ID_RAW_PRESSURE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#endif
}

/**
 * @brief Encode a raw_pressure struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_pressure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_pressure_t* raw_pressure)
{
	return mavlink_msg_raw_pressure_pack(system_id, component_id, msg, raw_pressure->time_usec, raw_pressure->press_abs, raw_pressure->press_diff1, raw_pressure->press_diff2, raw_pressure->temperature);
}

/**
 * @brief Encode a raw_pressure struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param raw_pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_pressure_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_raw_pressure_t* raw_pressure)
{
	return mavlink_msg_raw_pressure_pack_chan(system_id, component_id, chan, msg, raw_pressure->time_usec, raw_pressure->press_abs, raw_pressure->press_diff1, raw_pressure->press_diff2, raw_pressure->temperature);
}

/**
 * @brief Send a raw_pressure message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (raw)
 * @param press_diff1 Differential pressure 1 (raw)
 * @param press_diff2 Differential pressure 2 (raw)
 * @param temperature Raw Temperature measurement (raw)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_pressure_send(mavlink_channel_t chan, uint64_t time_usec, int16_t press_abs, int16_t press_diff1, int16_t press_diff2, int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RAW_PRESSURE_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int16_t(buf, 8, press_abs);
	_mav_put_int16_t(buf, 10, press_diff1);
	_mav_put_int16_t(buf, 12, press_diff2);
	_mav_put_int16_t(buf, 14, temperature);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_PRESSURE, buf, MAVLINK_MSG_ID_RAW_PRESSURE_LEN, MAVLINK_MSG_ID_RAW_PRESSURE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_PRESSURE, buf, MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#endif
#else
	mavlink_raw_pressure_t packet;
	packet.time_usec = time_usec;
	packet.press_abs = press_abs;
	packet.press_diff1 = press_diff1;
	packet.press_diff2 = press_diff2;
	packet.temperature = temperature;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_PRESSURE, (const char *)&packet, MAVLINK_MSG_ID_RAW_PRESSURE_LEN, MAVLINK_MSG_ID_RAW_PRESSURE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_PRESSURE, (const char *)&packet, MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RAW_PRESSURE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_raw_pressure_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, int16_t press_abs, int16_t press_diff1, int16_t press_diff2, int16_t temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int16_t(buf, 8, press_abs);
	_mav_put_int16_t(buf, 10, press_diff1);
	_mav_put_int16_t(buf, 12, press_diff2);
	_mav_put_int16_t(buf, 14, temperature);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_PRESSURE, buf, MAVLINK_MSG_ID_RAW_PRESSURE_LEN, MAVLINK_MSG_ID_RAW_PRESSURE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_PRESSURE, buf, MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#endif
#else
	mavlink_raw_pressure_t *packet = (mavlink_raw_pressure_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->press_abs = press_abs;
	packet->press_diff1 = press_diff1;
	packet->press_diff2 = press_diff2;
	packet->temperature = temperature;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_PRESSURE, (const char *)packet, MAVLINK_MSG_ID_RAW_PRESSURE_LEN, MAVLINK_MSG_ID_RAW_PRESSURE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_PRESSURE, (const char *)packet, MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RAW_PRESSURE UNPACKING


/**
 * @brief Get field time_usec from raw_pressure message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_raw_pressure_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field press_abs from raw_pressure message
 *
 * @return Absolute pressure (raw)
 */
static inline int16_t mavlink_msg_raw_pressure_get_press_abs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field press_diff1 from raw_pressure message
 *
 * @return Differential pressure 1 (raw)
 */
static inline int16_t mavlink_msg_raw_pressure_get_press_diff1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field press_diff2 from raw_pressure message
 *
 * @return Differential pressure 2 (raw)
 */
static inline int16_t mavlink_msg_raw_pressure_get_press_diff2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field temperature from raw_pressure message
 *
 * @return Raw Temperature measurement (raw)
 */
static inline int16_t mavlink_msg_raw_pressure_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Decode a raw_pressure message into a struct
 *
 * @param msg The message to decode
 * @param raw_pressure C-struct to decode the message contents into
 */
static inline void mavlink_msg_raw_pressure_decode(const mavlink_message_t* msg, mavlink_raw_pressure_t* raw_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP
	raw_pressure->time_usec = mavlink_msg_raw_pressure_get_time_usec(msg);
	raw_pressure->press_abs = mavlink_msg_raw_pressure_get_press_abs(msg);
	raw_pressure->press_diff1 = mavlink_msg_raw_pressure_get_press_diff1(msg);
	raw_pressure->press_diff2 = mavlink_msg_raw_pressure_get_press_diff2(msg);
	raw_pressure->temperature = mavlink_msg_raw_pressure_get_temperature(msg);
#else
	memcpy(raw_pressure, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RAW_PRESSURE_LEN);
#endif
}

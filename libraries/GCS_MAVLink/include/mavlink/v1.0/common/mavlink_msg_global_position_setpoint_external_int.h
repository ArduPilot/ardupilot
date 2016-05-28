// MESSAGE GLOBAL_POSITION_SETPOINT_EXTERNAL_INT PACKING

#define MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT 84

typedef struct __mavlink_global_position_setpoint_external_int_t
{
 uint32_t time_boot_ms; ///< Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 int32_t lat_int; ///< X Position in WGS84 frame in 1e7 * meters
 int32_t lon_int; ///< Y Position in WGS84 frame in 1e7 * meters
 float alt; ///< Altitude in WGS84, not AMSL
 float vx; ///< X velocity in NED frame in meter / s
 float vy; ///< Y velocity in NED frame in meter / s
 float vz; ///< Z velocity in NED frame in meter / s
 float afx; ///< X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float afy; ///< Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float afz; ///< Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 uint16_t type_mask; ///< Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_global_position_setpoint_external_int_t;

#define MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN 44
#define MAVLINK_MSG_ID_84_LEN 44

#define MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_CRC 198
#define MAVLINK_MSG_ID_84_CRC 198



#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT { \
	"GLOBAL_POSITION_SETPOINT_EXTERNAL_INT", \
	13, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_global_position_setpoint_external_int_t, time_boot_ms) }, \
         { "lat_int", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_global_position_setpoint_external_int_t, lat_int) }, \
         { "lon_int", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_position_setpoint_external_int_t, lon_int) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_global_position_setpoint_external_int_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_global_position_setpoint_external_int_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_global_position_setpoint_external_int_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_global_position_setpoint_external_int_t, vz) }, \
         { "afx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_global_position_setpoint_external_int_t, afx) }, \
         { "afy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_global_position_setpoint_external_int_t, afy) }, \
         { "afz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_global_position_setpoint_external_int_t, afz) }, \
         { "type_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_global_position_setpoint_external_int_t, type_mask) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_global_position_setpoint_external_int_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_global_position_setpoint_external_int_t, target_component) }, \
         } \
}


/**
 * @brief Pack a global_position_setpoint_external_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 * @param target_system System ID
 * @param target_component Component ID
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint
 * @param lat_int X Position in WGS84 frame in 1e7 * meters
 * @param lon_int Y Position in WGS84 frame in 1e7 * meters
 * @param alt Altitude in WGS84, not AMSL
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_setpoint_external_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat_int);
	_mav_put_int32_t(buf, 8, lon_int);
	_mav_put_float(buf, 12, alt);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, afx);
	_mav_put_float(buf, 32, afy);
	_mav_put_float(buf, 36, afz);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint8_t(buf, 42, target_system);
	_mav_put_uint8_t(buf, 43, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#else
	mavlink_global_position_setpoint_external_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat_int = lat_int;
	packet.lon_int = lon_int;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.afx = afx;
	packet.afy = afy;
	packet.afz = afz;
	packet.type_mask = type_mask;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#endif
}

/**
 * @brief Pack a global_position_setpoint_external_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 * @param target_system System ID
 * @param target_component Component ID
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint
 * @param lat_int X Position in WGS84 frame in 1e7 * meters
 * @param lon_int Y Position in WGS84 frame in 1e7 * meters
 * @param alt Altitude in WGS84, not AMSL
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_setpoint_external_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t target_system,uint8_t target_component,uint16_t type_mask,int32_t lat_int,int32_t lon_int,float alt,float vx,float vy,float vz,float afx,float afy,float afz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat_int);
	_mav_put_int32_t(buf, 8, lon_int);
	_mav_put_float(buf, 12, alt);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, afx);
	_mav_put_float(buf, 32, afy);
	_mav_put_float(buf, 36, afz);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint8_t(buf, 42, target_system);
	_mav_put_uint8_t(buf, 43, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#else
	mavlink_global_position_setpoint_external_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat_int = lat_int;
	packet.lon_int = lon_int;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.afx = afx;
	packet.afy = afy;
	packet.afz = afz;
	packet.type_mask = type_mask;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#endif
}

/**
 * @brief Encode a global_position_setpoint_external_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_setpoint_external_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_setpoint_external_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_setpoint_external_int_t* global_position_setpoint_external_int)
{
	return mavlink_msg_global_position_setpoint_external_int_pack(system_id, component_id, msg, global_position_setpoint_external_int->time_boot_ms, global_position_setpoint_external_int->target_system, global_position_setpoint_external_int->target_component, global_position_setpoint_external_int->type_mask, global_position_setpoint_external_int->lat_int, global_position_setpoint_external_int->lon_int, global_position_setpoint_external_int->alt, global_position_setpoint_external_int->vx, global_position_setpoint_external_int->vy, global_position_setpoint_external_int->vz, global_position_setpoint_external_int->afx, global_position_setpoint_external_int->afy, global_position_setpoint_external_int->afz);
}

/**
 * @brief Encode a global_position_setpoint_external_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_position_setpoint_external_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_setpoint_external_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_position_setpoint_external_int_t* global_position_setpoint_external_int)
{
	return mavlink_msg_global_position_setpoint_external_int_pack_chan(system_id, component_id, chan, msg, global_position_setpoint_external_int->time_boot_ms, global_position_setpoint_external_int->target_system, global_position_setpoint_external_int->target_component, global_position_setpoint_external_int->type_mask, global_position_setpoint_external_int->lat_int, global_position_setpoint_external_int->lon_int, global_position_setpoint_external_int->alt, global_position_setpoint_external_int->vx, global_position_setpoint_external_int->vy, global_position_setpoint_external_int->vz, global_position_setpoint_external_int->afx, global_position_setpoint_external_int->afy, global_position_setpoint_external_int->afz);
}

/**
 * @brief Send a global_position_setpoint_external_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 * @param target_system System ID
 * @param target_component Component ID
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint
 * @param lat_int X Position in WGS84 frame in 1e7 * meters
 * @param lon_int Y Position in WGS84 frame in 1e7 * meters
 * @param alt Altitude in WGS84, not AMSL
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_setpoint_external_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat_int);
	_mav_put_int32_t(buf, 8, lon_int);
	_mav_put_float(buf, 12, alt);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, afx);
	_mav_put_float(buf, 32, afy);
	_mav_put_float(buf, 36, afz);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint8_t(buf, 42, target_system);
	_mav_put_uint8_t(buf, 43, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#endif
#else
	mavlink_global_position_setpoint_external_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat_int = lat_int;
	packet.lon_int = lon_int;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.afx = afx;
	packet.afy = afy;
	packet.afz = afz;
	packet.type_mask = type_mask;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_global_position_setpoint_external_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint16_t type_mask, int32_t lat_int, int32_t lon_int, float alt, float vx, float vy, float vz, float afx, float afy, float afz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat_int);
	_mav_put_int32_t(buf, 8, lon_int);
	_mav_put_float(buf, 12, alt);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, afx);
	_mav_put_float(buf, 32, afy);
	_mav_put_float(buf, 36, afz);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint8_t(buf, 42, target_system);
	_mav_put_uint8_t(buf, 43, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#endif
#else
	mavlink_global_position_setpoint_external_int_t *packet = (mavlink_global_position_setpoint_external_int_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->lat_int = lat_int;
	packet->lon_int = lon_int;
	packet->alt = alt;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	packet->afx = afx;
	packet->afy = afy;
	packet->afz = afz;
	packet->type_mask = type_mask;
	packet->target_system = target_system;
	packet->target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GLOBAL_POSITION_SETPOINT_EXTERNAL_INT UNPACKING


/**
 * @brief Get field time_boot_ms from global_position_setpoint_external_int message
 *
 * @return Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 */
static inline uint32_t mavlink_msg_global_position_setpoint_external_int_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field target_system from global_position_setpoint_external_int message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_global_position_setpoint_external_int_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field target_component from global_position_setpoint_external_int message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_global_position_setpoint_external_int_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field type_mask from global_position_setpoint_external_int message
 *
 * @return Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint
 */
static inline uint16_t mavlink_msg_global_position_setpoint_external_int_get_type_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field lat_int from global_position_setpoint_external_int message
 *
 * @return X Position in WGS84 frame in 1e7 * meters
 */
static inline int32_t mavlink_msg_global_position_setpoint_external_int_get_lat_int(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon_int from global_position_setpoint_external_int message
 *
 * @return Y Position in WGS84 frame in 1e7 * meters
 */
static inline int32_t mavlink_msg_global_position_setpoint_external_int_get_lon_int(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from global_position_setpoint_external_int message
 *
 * @return Altitude in WGS84, not AMSL
 */
static inline float mavlink_msg_global_position_setpoint_external_int_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vx from global_position_setpoint_external_int message
 *
 * @return X velocity in NED frame in meter / s
 */
static inline float mavlink_msg_global_position_setpoint_external_int_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vy from global_position_setpoint_external_int message
 *
 * @return Y velocity in NED frame in meter / s
 */
static inline float mavlink_msg_global_position_setpoint_external_int_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vz from global_position_setpoint_external_int message
 *
 * @return Z velocity in NED frame in meter / s
 */
static inline float mavlink_msg_global_position_setpoint_external_int_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field afx from global_position_setpoint_external_int message
 *
 * @return X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_global_position_setpoint_external_int_get_afx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field afy from global_position_setpoint_external_int message
 *
 * @return Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_global_position_setpoint_external_int_get_afy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field afz from global_position_setpoint_external_int message
 *
 * @return Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_global_position_setpoint_external_int_get_afz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a global_position_setpoint_external_int message into a struct
 *
 * @param msg The message to decode
 * @param global_position_setpoint_external_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_position_setpoint_external_int_decode(const mavlink_message_t* msg, mavlink_global_position_setpoint_external_int_t* global_position_setpoint_external_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	global_position_setpoint_external_int->time_boot_ms = mavlink_msg_global_position_setpoint_external_int_get_time_boot_ms(msg);
	global_position_setpoint_external_int->lat_int = mavlink_msg_global_position_setpoint_external_int_get_lat_int(msg);
	global_position_setpoint_external_int->lon_int = mavlink_msg_global_position_setpoint_external_int_get_lon_int(msg);
	global_position_setpoint_external_int->alt = mavlink_msg_global_position_setpoint_external_int_get_alt(msg);
	global_position_setpoint_external_int->vx = mavlink_msg_global_position_setpoint_external_int_get_vx(msg);
	global_position_setpoint_external_int->vy = mavlink_msg_global_position_setpoint_external_int_get_vy(msg);
	global_position_setpoint_external_int->vz = mavlink_msg_global_position_setpoint_external_int_get_vz(msg);
	global_position_setpoint_external_int->afx = mavlink_msg_global_position_setpoint_external_int_get_afx(msg);
	global_position_setpoint_external_int->afy = mavlink_msg_global_position_setpoint_external_int_get_afy(msg);
	global_position_setpoint_external_int->afz = mavlink_msg_global_position_setpoint_external_int_get_afz(msg);
	global_position_setpoint_external_int->type_mask = mavlink_msg_global_position_setpoint_external_int_get_type_mask(msg);
	global_position_setpoint_external_int->target_system = mavlink_msg_global_position_setpoint_external_int_get_target_system(msg);
	global_position_setpoint_external_int->target_component = mavlink_msg_global_position_setpoint_external_int_get_target_component(msg);
#else
	memcpy(global_position_setpoint_external_int, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT_LEN);
#endif
}

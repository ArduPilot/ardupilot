// MESSAGE LOCAL_NED_POSITION_SETPOINT_EXTERNAL PACKING

#define MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL 83

typedef struct __mavlink_local_ned_position_setpoint_external_t
{
 uint32_t time_boot_ms; ///< Timestamp in milliseconds since system boot
 float x; ///< X Position in NED frame in meters
 float y; ///< Y Position in NED frame in meters
 float z; ///< Z Position in NED frame in meters (note, altitude is negative in NED)
 float vx; ///< X velocity in NED frame in meter / s
 float vy; ///< Y velocity in NED frame in meter / s
 float vz; ///< Z velocity in NED frame in meter / s
 float afx; ///< X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float afy; ///< Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float afz; ///< Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 uint16_t type_mask; ///< Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t coordinate_frame; ///< Valid options are: MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_OFFSET_NED = 5, MAV_FRAME_BODY_NED = 6, MAV_FRAME_BODY_OFFSET_NED = 7
} mavlink_local_ned_position_setpoint_external_t;

#define MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN 45
#define MAVLINK_MSG_ID_83_LEN 45

#define MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_CRC 211
#define MAVLINK_MSG_ID_83_CRC 211



#define MAVLINK_MESSAGE_INFO_LOCAL_NED_POSITION_SETPOINT_EXTERNAL { \
	"LOCAL_NED_POSITION_SETPOINT_EXTERNAL", \
	14, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_local_ned_position_setpoint_external_t, time_boot_ms) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_local_ned_position_setpoint_external_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_local_ned_position_setpoint_external_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_local_ned_position_setpoint_external_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_local_ned_position_setpoint_external_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_local_ned_position_setpoint_external_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_local_ned_position_setpoint_external_t, vz) }, \
         { "afx", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_local_ned_position_setpoint_external_t, afx) }, \
         { "afy", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_local_ned_position_setpoint_external_t, afy) }, \
         { "afz", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_local_ned_position_setpoint_external_t, afz) }, \
         { "type_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_local_ned_position_setpoint_external_t, type_mask) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_local_ned_position_setpoint_external_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_local_ned_position_setpoint_external_t, target_component) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_local_ned_position_setpoint_external_t, coordinate_frame) }, \
         } \
}


/**
 * @brief Pack a local_ned_position_setpoint_external message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_OFFSET_NED = 5, MAV_FRAME_BODY_NED = 6, MAV_FRAME_BODY_OFFSET_NED = 7
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters (note, altitude is negative in NED)
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_ned_position_setpoint_external_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, afx);
	_mav_put_float(buf, 32, afy);
	_mav_put_float(buf, 36, afz);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint8_t(buf, 42, target_system);
	_mav_put_uint8_t(buf, 43, target_component);
	_mav_put_uint8_t(buf, 44, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#else
	mavlink_local_ned_position_setpoint_external_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.afx = afx;
	packet.afy = afy;
	packet.afz = afz;
	packet.type_mask = type_mask;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#endif
}

/**
 * @brief Pack a local_ned_position_setpoint_external message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_OFFSET_NED = 5, MAV_FRAME_BODY_NED = 6, MAV_FRAME_BODY_OFFSET_NED = 7
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters (note, altitude is negative in NED)
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_ned_position_setpoint_external_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t target_system,uint8_t target_component,uint8_t coordinate_frame,uint16_t type_mask,float x,float y,float z,float vx,float vy,float vz,float afx,float afy,float afz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, afx);
	_mav_put_float(buf, 32, afy);
	_mav_put_float(buf, 36, afz);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint8_t(buf, 42, target_system);
	_mav_put_uint8_t(buf, 43, target_component);
	_mav_put_uint8_t(buf, 44, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#else
	mavlink_local_ned_position_setpoint_external_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.afx = afx;
	packet.afy = afy;
	packet.afz = afz;
	packet.type_mask = type_mask;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#endif
}

/**
 * @brief Encode a local_ned_position_setpoint_external struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_ned_position_setpoint_external C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_ned_position_setpoint_external_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_local_ned_position_setpoint_external_t* local_ned_position_setpoint_external)
{
	return mavlink_msg_local_ned_position_setpoint_external_pack(system_id, component_id, msg, local_ned_position_setpoint_external->time_boot_ms, local_ned_position_setpoint_external->target_system, local_ned_position_setpoint_external->target_component, local_ned_position_setpoint_external->coordinate_frame, local_ned_position_setpoint_external->type_mask, local_ned_position_setpoint_external->x, local_ned_position_setpoint_external->y, local_ned_position_setpoint_external->z, local_ned_position_setpoint_external->vx, local_ned_position_setpoint_external->vy, local_ned_position_setpoint_external->vz, local_ned_position_setpoint_external->afx, local_ned_position_setpoint_external->afy, local_ned_position_setpoint_external->afz);
}

/**
 * @brief Encode a local_ned_position_setpoint_external struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param local_ned_position_setpoint_external C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_ned_position_setpoint_external_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_local_ned_position_setpoint_external_t* local_ned_position_setpoint_external)
{
	return mavlink_msg_local_ned_position_setpoint_external_pack_chan(system_id, component_id, chan, msg, local_ned_position_setpoint_external->time_boot_ms, local_ned_position_setpoint_external->target_system, local_ned_position_setpoint_external->target_component, local_ned_position_setpoint_external->coordinate_frame, local_ned_position_setpoint_external->type_mask, local_ned_position_setpoint_external->x, local_ned_position_setpoint_external->y, local_ned_position_setpoint_external->z, local_ned_position_setpoint_external->vx, local_ned_position_setpoint_external->vy, local_ned_position_setpoint_external->vz, local_ned_position_setpoint_external->afx, local_ned_position_setpoint_external->afy, local_ned_position_setpoint_external->afz);
}

/**
 * @brief Send a local_ned_position_setpoint_external message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_OFFSET_NED = 5, MAV_FRAME_BODY_NED = 6, MAV_FRAME_BODY_OFFSET_NED = 7
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint
 * @param x X Position in NED frame in meters
 * @param y Y Position in NED frame in meters
 * @param z Z Position in NED frame in meters (note, altitude is negative in NED)
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param afx X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afy Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 * @param afz Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_ned_position_setpoint_external_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, afx);
	_mav_put_float(buf, 32, afy);
	_mav_put_float(buf, 36, afz);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint8_t(buf, 42, target_system);
	_mav_put_uint8_t(buf, 43, target_component);
	_mav_put_uint8_t(buf, 44, coordinate_frame);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL, buf, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL, buf, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#endif
#else
	mavlink_local_ned_position_setpoint_external_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.afx = afx;
	packet.afy = afy;
	packet.afz = afz;
	packet.type_mask = type_mask;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL, (const char *)&packet, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL, (const char *)&packet, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_local_ned_position_setpoint_external_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, x);
	_mav_put_float(buf, 8, y);
	_mav_put_float(buf, 12, z);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, afx);
	_mav_put_float(buf, 32, afy);
	_mav_put_float(buf, 36, afz);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint8_t(buf, 42, target_system);
	_mav_put_uint8_t(buf, 43, target_component);
	_mav_put_uint8_t(buf, 44, coordinate_frame);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL, buf, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL, buf, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#endif
#else
	mavlink_local_ned_position_setpoint_external_t *packet = (mavlink_local_ned_position_setpoint_external_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	packet->afx = afx;
	packet->afy = afy;
	packet->afz = afz;
	packet->type_mask = type_mask;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->coordinate_frame = coordinate_frame;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL, (const char *)packet, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL, (const char *)packet, MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE LOCAL_NED_POSITION_SETPOINT_EXTERNAL UNPACKING


/**
 * @brief Get field time_boot_ms from local_ned_position_setpoint_external message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint32_t mavlink_msg_local_ned_position_setpoint_external_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field target_system from local_ned_position_setpoint_external message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_local_ned_position_setpoint_external_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field target_component from local_ned_position_setpoint_external message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_local_ned_position_setpoint_external_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field coordinate_frame from local_ned_position_setpoint_external message
 *
 * @return Valid options are: MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_OFFSET_NED = 5, MAV_FRAME_BODY_NED = 6, MAV_FRAME_BODY_OFFSET_NED = 7
 */
static inline uint8_t mavlink_msg_local_ned_position_setpoint_external_get_coordinate_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field type_mask from local_ned_position_setpoint_external message
 *
 * @return Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint
 */
static inline uint16_t mavlink_msg_local_ned_position_setpoint_external_get_type_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field x from local_ned_position_setpoint_external message
 *
 * @return X Position in NED frame in meters
 */
static inline float mavlink_msg_local_ned_position_setpoint_external_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from local_ned_position_setpoint_external message
 *
 * @return Y Position in NED frame in meters
 */
static inline float mavlink_msg_local_ned_position_setpoint_external_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from local_ned_position_setpoint_external message
 *
 * @return Z Position in NED frame in meters (note, altitude is negative in NED)
 */
static inline float mavlink_msg_local_ned_position_setpoint_external_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vx from local_ned_position_setpoint_external message
 *
 * @return X velocity in NED frame in meter / s
 */
static inline float mavlink_msg_local_ned_position_setpoint_external_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vy from local_ned_position_setpoint_external message
 *
 * @return Y velocity in NED frame in meter / s
 */
static inline float mavlink_msg_local_ned_position_setpoint_external_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vz from local_ned_position_setpoint_external message
 *
 * @return Z velocity in NED frame in meter / s
 */
static inline float mavlink_msg_local_ned_position_setpoint_external_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field afx from local_ned_position_setpoint_external message
 *
 * @return X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_local_ned_position_setpoint_external_get_afx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field afy from local_ned_position_setpoint_external message
 *
 * @return Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_local_ned_position_setpoint_external_get_afy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field afz from local_ned_position_setpoint_external message
 *
 * @return Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 */
static inline float mavlink_msg_local_ned_position_setpoint_external_get_afz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a local_ned_position_setpoint_external message into a struct
 *
 * @param msg The message to decode
 * @param local_ned_position_setpoint_external C-struct to decode the message contents into
 */
static inline void mavlink_msg_local_ned_position_setpoint_external_decode(const mavlink_message_t* msg, mavlink_local_ned_position_setpoint_external_t* local_ned_position_setpoint_external)
{
#if MAVLINK_NEED_BYTE_SWAP
	local_ned_position_setpoint_external->time_boot_ms = mavlink_msg_local_ned_position_setpoint_external_get_time_boot_ms(msg);
	local_ned_position_setpoint_external->x = mavlink_msg_local_ned_position_setpoint_external_get_x(msg);
	local_ned_position_setpoint_external->y = mavlink_msg_local_ned_position_setpoint_external_get_y(msg);
	local_ned_position_setpoint_external->z = mavlink_msg_local_ned_position_setpoint_external_get_z(msg);
	local_ned_position_setpoint_external->vx = mavlink_msg_local_ned_position_setpoint_external_get_vx(msg);
	local_ned_position_setpoint_external->vy = mavlink_msg_local_ned_position_setpoint_external_get_vy(msg);
	local_ned_position_setpoint_external->vz = mavlink_msg_local_ned_position_setpoint_external_get_vz(msg);
	local_ned_position_setpoint_external->afx = mavlink_msg_local_ned_position_setpoint_external_get_afx(msg);
	local_ned_position_setpoint_external->afy = mavlink_msg_local_ned_position_setpoint_external_get_afy(msg);
	local_ned_position_setpoint_external->afz = mavlink_msg_local_ned_position_setpoint_external_get_afz(msg);
	local_ned_position_setpoint_external->type_mask = mavlink_msg_local_ned_position_setpoint_external_get_type_mask(msg);
	local_ned_position_setpoint_external->target_system = mavlink_msg_local_ned_position_setpoint_external_get_target_system(msg);
	local_ned_position_setpoint_external->target_component = mavlink_msg_local_ned_position_setpoint_external_get_target_component(msg);
	local_ned_position_setpoint_external->coordinate_frame = mavlink_msg_local_ned_position_setpoint_external_get_coordinate_frame(msg);
#else
	memcpy(local_ned_position_setpoint_external, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL_LEN);
#endif
}

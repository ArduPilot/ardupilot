// MESSAGE SET_ROI_GLOBAL_INT PACKING

#define MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT 88

typedef struct __mavlink_set_roi_global_int_t
{
 uint32_t time_boot_ms; /*< Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.*/
 int32_t lat_int; /*< X Position in WGS84 frame in 1e7 * meters*/
 int32_t lng_int; /*< Y Position in WGS84 frame in 1e7 * meters*/
 float alt; /*< Altitude in meters.  (FRAME_GLOBAL_INT implies AMSL altitude, FRAME_GLOBAL_RELATIVE_ALT_INT implies relative to home, FRAME_GLOBAL_TERRAIN_ALT_INT implies above terrain model).*/
 float vx; /*< X velocity in NED frame in meter / s*/
 float vy; /*< Y velocity in NED frame in meter / s*/
 float vz; /*< Z velocity in NED frame in meter / s*/
 float ax; /*< X acceleration in NED frame in meter / s^2*/
 float ay; /*< Y acceleration in NED frame in meter / s^2*/
 float az; /*< Z acceleration in NED frame in meter / s^2*/
 uint16_t type_mask; /*< Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. Mapping: bit 1: lat, bit 2: lon, bit 3: alt, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az*/
 uint16_t timeout_ms; /*< Maximum time the ROI point will be forward integrated using velocity and acceleration. 0 disables timeout.*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t coordinate_frame; /*< Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11*/
 uint8_t roi_index; /*< Which ROI should be set if the vehicle supports multiple ROI points.*/
} mavlink_set_roi_global_int_t;

#define MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN 48
#define MAVLINK_MSG_ID_88_LEN 48

#define MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_CRC 166
#define MAVLINK_MSG_ID_88_CRC 166



#define MAVLINK_MESSAGE_INFO_SET_ROI_GLOBAL_INT { \
	"SET_ROI_GLOBAL_INT", \
	16, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_set_roi_global_int_t, time_boot_ms) }, \
         { "lat_int", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_set_roi_global_int_t, lat_int) }, \
         { "lng_int", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_set_roi_global_int_t, lng_int) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_roi_global_int_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_set_roi_global_int_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_set_roi_global_int_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_set_roi_global_int_t, vz) }, \
         { "ax", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_set_roi_global_int_t, ax) }, \
         { "ay", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_set_roi_global_int_t, ay) }, \
         { "az", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_set_roi_global_int_t, az) }, \
         { "type_mask", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_set_roi_global_int_t, type_mask) }, \
         { "timeout_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_set_roi_global_int_t, timeout_ms) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_set_roi_global_int_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_set_roi_global_int_t, target_component) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_set_roi_global_int_t, coordinate_frame) }, \
         { "roi_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_set_roi_global_int_t, roi_index) }, \
         } \
}


/**
 * @brief Pack a set_roi_global_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. Mapping: bit 1: lat, bit 2: lon, bit 3: alt, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az
 * @param roi_index Which ROI should be set if the vehicle supports multiple ROI points.
 * @param timeout_ms Maximum time the ROI point will be forward integrated using velocity and acceleration. 0 disables timeout.
 * @param lat_int X Position in WGS84 frame in 1e7 * meters
 * @param lng_int Y Position in WGS84 frame in 1e7 * meters
 * @param alt Altitude in meters.  (FRAME_GLOBAL_INT implies AMSL altitude, FRAME_GLOBAL_RELATIVE_ALT_INT implies relative to home, FRAME_GLOBAL_TERRAIN_ALT_INT implies above terrain model).
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param ax X acceleration in NED frame in meter / s^2
 * @param ay Y acceleration in NED frame in meter / s^2
 * @param az Z acceleration in NED frame in meter / s^2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_roi_global_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, uint8_t roi_index, uint16_t timeout_ms, int32_t lat_int, int32_t lng_int, float alt, float vx, float vy, float vz, float ax, float ay, float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat_int);
	_mav_put_int32_t(buf, 8, lng_int);
	_mav_put_float(buf, 12, alt);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, ax);
	_mav_put_float(buf, 32, ay);
	_mav_put_float(buf, 36, az);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint16_t(buf, 42, timeout_ms);
	_mav_put_uint8_t(buf, 44, target_system);
	_mav_put_uint8_t(buf, 45, target_component);
	_mav_put_uint8_t(buf, 46, coordinate_frame);
	_mav_put_uint8_t(buf, 47, roi_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#else
	mavlink_set_roi_global_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat_int = lat_int;
	packet.lng_int = lng_int;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.ax = ax;
	packet.ay = ay;
	packet.az = az;
	packet.type_mask = type_mask;
	packet.timeout_ms = timeout_ms;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;
	packet.roi_index = roi_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#endif
}

/**
 * @brief Pack a set_roi_global_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. Mapping: bit 1: lat, bit 2: lon, bit 3: alt, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az
 * @param roi_index Which ROI should be set if the vehicle supports multiple ROI points.
 * @param timeout_ms Maximum time the ROI point will be forward integrated using velocity and acceleration. 0 disables timeout.
 * @param lat_int X Position in WGS84 frame in 1e7 * meters
 * @param lng_int Y Position in WGS84 frame in 1e7 * meters
 * @param alt Altitude in meters.  (FRAME_GLOBAL_INT implies AMSL altitude, FRAME_GLOBAL_RELATIVE_ALT_INT implies relative to home, FRAME_GLOBAL_TERRAIN_ALT_INT implies above terrain model).
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param ax X acceleration in NED frame in meter / s^2
 * @param ay Y acceleration in NED frame in meter / s^2
 * @param az Z acceleration in NED frame in meter / s^2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_roi_global_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t target_system,uint8_t target_component,uint8_t coordinate_frame,uint16_t type_mask,uint8_t roi_index,uint16_t timeout_ms,int32_t lat_int,int32_t lng_int,float alt,float vx,float vy,float vz,float ax,float ay,float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat_int);
	_mav_put_int32_t(buf, 8, lng_int);
	_mav_put_float(buf, 12, alt);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, ax);
	_mav_put_float(buf, 32, ay);
	_mav_put_float(buf, 36, az);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint16_t(buf, 42, timeout_ms);
	_mav_put_uint8_t(buf, 44, target_system);
	_mav_put_uint8_t(buf, 45, target_component);
	_mav_put_uint8_t(buf, 46, coordinate_frame);
	_mav_put_uint8_t(buf, 47, roi_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#else
	mavlink_set_roi_global_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat_int = lat_int;
	packet.lng_int = lng_int;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.ax = ax;
	packet.ay = ay;
	packet.az = az;
	packet.type_mask = type_mask;
	packet.timeout_ms = timeout_ms;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;
	packet.roi_index = roi_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#endif
}

/**
 * @brief Encode a set_roi_global_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_roi_global_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_roi_global_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_roi_global_int_t* set_roi_global_int)
{
	return mavlink_msg_set_roi_global_int_pack(system_id, component_id, msg, set_roi_global_int->time_boot_ms, set_roi_global_int->target_system, set_roi_global_int->target_component, set_roi_global_int->coordinate_frame, set_roi_global_int->type_mask, set_roi_global_int->roi_index, set_roi_global_int->timeout_ms, set_roi_global_int->lat_int, set_roi_global_int->lng_int, set_roi_global_int->alt, set_roi_global_int->vx, set_roi_global_int->vy, set_roi_global_int->vz, set_roi_global_int->ax, set_roi_global_int->ay, set_roi_global_int->az);
}

/**
 * @brief Encode a set_roi_global_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_roi_global_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_roi_global_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_roi_global_int_t* set_roi_global_int)
{
	return mavlink_msg_set_roi_global_int_pack_chan(system_id, component_id, chan, msg, set_roi_global_int->time_boot_ms, set_roi_global_int->target_system, set_roi_global_int->target_component, set_roi_global_int->coordinate_frame, set_roi_global_int->type_mask, set_roi_global_int->roi_index, set_roi_global_int->timeout_ms, set_roi_global_int->lat_int, set_roi_global_int->lng_int, set_roi_global_int->alt, set_roi_global_int->vx, set_roi_global_int->vy, set_roi_global_int->vz, set_roi_global_int->ax, set_roi_global_int->ay, set_roi_global_int->az);
}

/**
 * @brief Send a set_roi_global_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
 * @param type_mask Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. Mapping: bit 1: lat, bit 2: lon, bit 3: alt, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az
 * @param roi_index Which ROI should be set if the vehicle supports multiple ROI points.
 * @param timeout_ms Maximum time the ROI point will be forward integrated using velocity and acceleration. 0 disables timeout.
 * @param lat_int X Position in WGS84 frame in 1e7 * meters
 * @param lng_int Y Position in WGS84 frame in 1e7 * meters
 * @param alt Altitude in meters.  (FRAME_GLOBAL_INT implies AMSL altitude, FRAME_GLOBAL_RELATIVE_ALT_INT implies relative to home, FRAME_GLOBAL_TERRAIN_ALT_INT implies above terrain model).
 * @param vx X velocity in NED frame in meter / s
 * @param vy Y velocity in NED frame in meter / s
 * @param vz Z velocity in NED frame in meter / s
 * @param ax X acceleration in NED frame in meter / s^2
 * @param ay Y acceleration in NED frame in meter / s^2
 * @param az Z acceleration in NED frame in meter / s^2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_roi_global_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, uint8_t roi_index, uint16_t timeout_ms, int32_t lat_int, int32_t lng_int, float alt, float vx, float vy, float vz, float ax, float ay, float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat_int);
	_mav_put_int32_t(buf, 8, lng_int);
	_mav_put_float(buf, 12, alt);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, ax);
	_mav_put_float(buf, 32, ay);
	_mav_put_float(buf, 36, az);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint16_t(buf, 42, timeout_ms);
	_mav_put_uint8_t(buf, 44, target_system);
	_mav_put_uint8_t(buf, 45, target_component);
	_mav_put_uint8_t(buf, 46, coordinate_frame);
	_mav_put_uint8_t(buf, 47, roi_index);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT, buf, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT, buf, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#endif
#else
	mavlink_set_roi_global_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat_int = lat_int;
	packet.lng_int = lng_int;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.ax = ax;
	packet.ay = ay;
	packet.az = az;
	packet.type_mask = type_mask;
	packet.timeout_ms = timeout_ms;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;
	packet.roi_index = roi_index;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT, (const char *)&packet, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT, (const char *)&packet, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_roi_global_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t target_system, uint8_t target_component, uint8_t coordinate_frame, uint16_t type_mask, uint8_t roi_index, uint16_t timeout_ms, int32_t lat_int, int32_t lng_int, float alt, float vx, float vy, float vz, float ax, float ay, float az)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, lat_int);
	_mav_put_int32_t(buf, 8, lng_int);
	_mav_put_float(buf, 12, alt);
	_mav_put_float(buf, 16, vx);
	_mav_put_float(buf, 20, vy);
	_mav_put_float(buf, 24, vz);
	_mav_put_float(buf, 28, ax);
	_mav_put_float(buf, 32, ay);
	_mav_put_float(buf, 36, az);
	_mav_put_uint16_t(buf, 40, type_mask);
	_mav_put_uint16_t(buf, 42, timeout_ms);
	_mav_put_uint8_t(buf, 44, target_system);
	_mav_put_uint8_t(buf, 45, target_component);
	_mav_put_uint8_t(buf, 46, coordinate_frame);
	_mav_put_uint8_t(buf, 47, roi_index);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT, buf, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT, buf, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#endif
#else
	mavlink_set_roi_global_int_t *packet = (mavlink_set_roi_global_int_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->lat_int = lat_int;
	packet->lng_int = lng_int;
	packet->alt = alt;
	packet->vx = vx;
	packet->vy = vy;
	packet->vz = vz;
	packet->ax = ax;
	packet->ay = ay;
	packet->az = az;
	packet->type_mask = type_mask;
	packet->timeout_ms = timeout_ms;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->coordinate_frame = coordinate_frame;
	packet->roi_index = roi_index;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT, (const char *)packet, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT, (const char *)packet, MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_ROI_GLOBAL_INT UNPACKING


/**
 * @brief Get field time_boot_ms from set_roi_global_int message
 *
 * @return Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
 */
static inline uint32_t mavlink_msg_set_roi_global_int_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field target_system from set_roi_global_int message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_roi_global_int_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field target_component from set_roi_global_int message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_roi_global_int_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field coordinate_frame from set_roi_global_int message
 *
 * @return Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
 */
static inline uint8_t mavlink_msg_set_roi_global_int_get_coordinate_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field type_mask from set_roi_global_int message
 *
 * @return Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. Mapping: bit 1: lat, bit 2: lon, bit 3: alt, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az
 */
static inline uint16_t mavlink_msg_set_roi_global_int_get_type_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field roi_index from set_roi_global_int message
 *
 * @return Which ROI should be set if the vehicle supports multiple ROI points.
 */
static inline uint8_t mavlink_msg_set_roi_global_int_get_roi_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  47);
}

/**
 * @brief Get field timeout_ms from set_roi_global_int message
 *
 * @return Maximum time the ROI point will be forward integrated using velocity and acceleration. 0 disables timeout.
 */
static inline uint16_t mavlink_msg_set_roi_global_int_get_timeout_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  42);
}

/**
 * @brief Get field lat_int from set_roi_global_int message
 *
 * @return X Position in WGS84 frame in 1e7 * meters
 */
static inline int32_t mavlink_msg_set_roi_global_int_get_lat_int(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lng_int from set_roi_global_int message
 *
 * @return Y Position in WGS84 frame in 1e7 * meters
 */
static inline int32_t mavlink_msg_set_roi_global_int_get_lng_int(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from set_roi_global_int message
 *
 * @return Altitude in meters.  (FRAME_GLOBAL_INT implies AMSL altitude, FRAME_GLOBAL_RELATIVE_ALT_INT implies relative to home, FRAME_GLOBAL_TERRAIN_ALT_INT implies above terrain model).
 */
static inline float mavlink_msg_set_roi_global_int_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field vx from set_roi_global_int message
 *
 * @return X velocity in NED frame in meter / s
 */
static inline float mavlink_msg_set_roi_global_int_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vy from set_roi_global_int message
 *
 * @return Y velocity in NED frame in meter / s
 */
static inline float mavlink_msg_set_roi_global_int_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vz from set_roi_global_int message
 *
 * @return Z velocity in NED frame in meter / s
 */
static inline float mavlink_msg_set_roi_global_int_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field ax from set_roi_global_int message
 *
 * @return X acceleration in NED frame in meter / s^2
 */
static inline float mavlink_msg_set_roi_global_int_get_ax(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field ay from set_roi_global_int message
 *
 * @return Y acceleration in NED frame in meter / s^2
 */
static inline float mavlink_msg_set_roi_global_int_get_ay(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field az from set_roi_global_int message
 *
 * @return Z acceleration in NED frame in meter / s^2
 */
static inline float mavlink_msg_set_roi_global_int_get_az(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a set_roi_global_int message into a struct
 *
 * @param msg The message to decode
 * @param set_roi_global_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_roi_global_int_decode(const mavlink_message_t* msg, mavlink_set_roi_global_int_t* set_roi_global_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_roi_global_int->time_boot_ms = mavlink_msg_set_roi_global_int_get_time_boot_ms(msg);
	set_roi_global_int->lat_int = mavlink_msg_set_roi_global_int_get_lat_int(msg);
	set_roi_global_int->lng_int = mavlink_msg_set_roi_global_int_get_lng_int(msg);
	set_roi_global_int->alt = mavlink_msg_set_roi_global_int_get_alt(msg);
	set_roi_global_int->vx = mavlink_msg_set_roi_global_int_get_vx(msg);
	set_roi_global_int->vy = mavlink_msg_set_roi_global_int_get_vy(msg);
	set_roi_global_int->vz = mavlink_msg_set_roi_global_int_get_vz(msg);
	set_roi_global_int->ax = mavlink_msg_set_roi_global_int_get_ax(msg);
	set_roi_global_int->ay = mavlink_msg_set_roi_global_int_get_ay(msg);
	set_roi_global_int->az = mavlink_msg_set_roi_global_int_get_az(msg);
	set_roi_global_int->type_mask = mavlink_msg_set_roi_global_int_get_type_mask(msg);
	set_roi_global_int->timeout_ms = mavlink_msg_set_roi_global_int_get_timeout_ms(msg);
	set_roi_global_int->target_system = mavlink_msg_set_roi_global_int_get_target_system(msg);
	set_roi_global_int->target_component = mavlink_msg_set_roi_global_int_get_target_component(msg);
	set_roi_global_int->coordinate_frame = mavlink_msg_set_roi_global_int_get_coordinate_frame(msg);
	set_roi_global_int->roi_index = mavlink_msg_set_roi_global_int_get_roi_index(msg);
#else
	memcpy(set_roi_global_int, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_ROI_GLOBAL_INT_LEN);
#endif
}

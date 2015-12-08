// MESSAGE ADSB_VEHICLE PACKING

#define MAVLINK_MSG_ID_ADSB_VEHICLE 246

typedef struct __mavlink_adsb_vehicle_t
{
 uint32_t ICAO_address; /*< ICAO address*/
 int32_t lat; /*< Latitude, expressed as degrees * 1E7*/
 int32_t lon; /*< Longitude, expressed as degrees * 1E7*/
 int32_t altitude; /*< Altitude(ASL) in millimeters*/
 uint16_t heading; /*< Course over ground in centidegrees*/
 uint16_t hor_velocity; /*< The horizontal velocity in centimeters/second*/
 uint16_t ver_velocity; /*< The vertical velocity in centimeters/second, positive is up*/
 uint16_t flags; /*< Flags to indicate various statuses including valid data fields*/
 uint16_t squawk; /*< Squawk code*/
 uint8_t altitude_type; /*< Type from ADSB_ALTITUDE_TYPE enum*/
 char callsign[9]; /*< The callsign, 8+null*/
 uint8_t emitter_type; /*< Type from ADSB_EMITTER_TYPE enum*/
 uint8_t tslc; /*< Time since last communication in seconds*/
} mavlink_adsb_vehicle_t;

#define MAVLINK_MSG_ID_ADSB_VEHICLE_LEN 38
#define MAVLINK_MSG_ID_246_LEN 38

#define MAVLINK_MSG_ID_ADSB_VEHICLE_CRC 158
#define MAVLINK_MSG_ID_246_CRC 158

#define MAVLINK_MSG_ADSB_VEHICLE_FIELD_CALLSIGN_LEN 9

#define MAVLINK_MESSAGE_INFO_ADSB_VEHICLE { \
	"ADSB_VEHICLE", \
	13, \
	{  { "ICAO_address", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_adsb_vehicle_t, ICAO_address) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_adsb_vehicle_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_adsb_vehicle_t, lon) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_adsb_vehicle_t, altitude) }, \
         { "heading", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_adsb_vehicle_t, heading) }, \
         { "hor_velocity", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_adsb_vehicle_t, hor_velocity) }, \
         { "ver_velocity", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_adsb_vehicle_t, ver_velocity) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_adsb_vehicle_t, flags) }, \
         { "squawk", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_adsb_vehicle_t, squawk) }, \
         { "altitude_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_adsb_vehicle_t, altitude_type) }, \
         { "callsign", NULL, MAVLINK_TYPE_CHAR, 9, 27, offsetof(mavlink_adsb_vehicle_t, callsign) }, \
         { "emitter_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_adsb_vehicle_t, emitter_type) }, \
         { "tslc", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_adsb_vehicle_t, tslc) }, \
         } \
}


/**
 * @brief Pack a adsb_vehicle message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ICAO_address ICAO address
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param altitude_type Type from ADSB_ALTITUDE_TYPE enum
 * @param altitude Altitude(ASL) in millimeters
 * @param heading Course over ground in centidegrees
 * @param hor_velocity The horizontal velocity in centimeters/second
 * @param ver_velocity The vertical velocity in centimeters/second, positive is up
 * @param callsign The callsign, 8+null
 * @param emitter_type Type from ADSB_EMITTER_TYPE enum
 * @param tslc Time since last communication in seconds
 * @param flags Flags to indicate various statuses including valid data fields
 * @param squawk Squawk code
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_vehicle_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, uint16_t ver_velocity, const char *callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_VEHICLE_LEN];
	_mav_put_uint32_t(buf, 0, ICAO_address);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_uint16_t(buf, 16, heading);
	_mav_put_uint16_t(buf, 18, hor_velocity);
	_mav_put_uint16_t(buf, 20, ver_velocity);
	_mav_put_uint16_t(buf, 22, flags);
	_mav_put_uint16_t(buf, 24, squawk);
	_mav_put_uint8_t(buf, 26, altitude_type);
	_mav_put_uint8_t(buf, 36, emitter_type);
	_mav_put_uint8_t(buf, 37, tslc);
	_mav_put_char_array(buf, 27, callsign, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#else
	mavlink_adsb_vehicle_t packet;
	packet.ICAO_address = ICAO_address;
	packet.lat = lat;
	packet.lon = lon;
	packet.altitude = altitude;
	packet.heading = heading;
	packet.hor_velocity = hor_velocity;
	packet.ver_velocity = ver_velocity;
	packet.flags = flags;
	packet.squawk = squawk;
	packet.altitude_type = altitude_type;
	packet.emitter_type = emitter_type;
	packet.tslc = tslc;
	mav_array_memcpy(packet.callsign, callsign, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_VEHICLE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN, MAVLINK_MSG_ID_ADSB_VEHICLE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#endif
}

/**
 * @brief Pack a adsb_vehicle message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ICAO_address ICAO address
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param altitude_type Type from ADSB_ALTITUDE_TYPE enum
 * @param altitude Altitude(ASL) in millimeters
 * @param heading Course over ground in centidegrees
 * @param hor_velocity The horizontal velocity in centimeters/second
 * @param ver_velocity The vertical velocity in centimeters/second, positive is up
 * @param callsign The callsign, 8+null
 * @param emitter_type Type from ADSB_EMITTER_TYPE enum
 * @param tslc Time since last communication in seconds
 * @param flags Flags to indicate various statuses including valid data fields
 * @param squawk Squawk code
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_vehicle_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t ICAO_address,int32_t lat,int32_t lon,uint8_t altitude_type,int32_t altitude,uint16_t heading,uint16_t hor_velocity,uint16_t ver_velocity,const char *callsign,uint8_t emitter_type,uint8_t tslc,uint16_t flags,uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_VEHICLE_LEN];
	_mav_put_uint32_t(buf, 0, ICAO_address);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_uint16_t(buf, 16, heading);
	_mav_put_uint16_t(buf, 18, hor_velocity);
	_mav_put_uint16_t(buf, 20, ver_velocity);
	_mav_put_uint16_t(buf, 22, flags);
	_mav_put_uint16_t(buf, 24, squawk);
	_mav_put_uint8_t(buf, 26, altitude_type);
	_mav_put_uint8_t(buf, 36, emitter_type);
	_mav_put_uint8_t(buf, 37, tslc);
	_mav_put_char_array(buf, 27, callsign, 9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#else
	mavlink_adsb_vehicle_t packet;
	packet.ICAO_address = ICAO_address;
	packet.lat = lat;
	packet.lon = lon;
	packet.altitude = altitude;
	packet.heading = heading;
	packet.hor_velocity = hor_velocity;
	packet.ver_velocity = ver_velocity;
	packet.flags = flags;
	packet.squawk = squawk;
	packet.altitude_type = altitude_type;
	packet.emitter_type = emitter_type;
	packet.tslc = tslc;
	mav_array_memcpy(packet.callsign, callsign, sizeof(char)*9);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_VEHICLE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN, MAVLINK_MSG_ID_ADSB_VEHICLE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#endif
}

/**
 * @brief Encode a adsb_vehicle struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adsb_vehicle C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_vehicle_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adsb_vehicle_t* adsb_vehicle)
{
	return mavlink_msg_adsb_vehicle_pack(system_id, component_id, msg, adsb_vehicle->ICAO_address, adsb_vehicle->lat, adsb_vehicle->lon, adsb_vehicle->altitude_type, adsb_vehicle->altitude, adsb_vehicle->heading, adsb_vehicle->hor_velocity, adsb_vehicle->ver_velocity, adsb_vehicle->callsign, adsb_vehicle->emitter_type, adsb_vehicle->tslc, adsb_vehicle->flags, adsb_vehicle->squawk);
}

/**
 * @brief Encode a adsb_vehicle struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adsb_vehicle C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_vehicle_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adsb_vehicle_t* adsb_vehicle)
{
	return mavlink_msg_adsb_vehicle_pack_chan(system_id, component_id, chan, msg, adsb_vehicle->ICAO_address, adsb_vehicle->lat, adsb_vehicle->lon, adsb_vehicle->altitude_type, adsb_vehicle->altitude, adsb_vehicle->heading, adsb_vehicle->hor_velocity, adsb_vehicle->ver_velocity, adsb_vehicle->callsign, adsb_vehicle->emitter_type, adsb_vehicle->tslc, adsb_vehicle->flags, adsb_vehicle->squawk);
}

/**
 * @brief Send a adsb_vehicle message
 * @param chan MAVLink channel to send the message
 *
 * @param ICAO_address ICAO address
 * @param lat Latitude, expressed as degrees * 1E7
 * @param lon Longitude, expressed as degrees * 1E7
 * @param altitude_type Type from ADSB_ALTITUDE_TYPE enum
 * @param altitude Altitude(ASL) in millimeters
 * @param heading Course over ground in centidegrees
 * @param hor_velocity The horizontal velocity in centimeters/second
 * @param ver_velocity The vertical velocity in centimeters/second, positive is up
 * @param callsign The callsign, 8+null
 * @param emitter_type Type from ADSB_EMITTER_TYPE enum
 * @param tslc Time since last communication in seconds
 * @param flags Flags to indicate various statuses including valid data fields
 * @param squawk Squawk code
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adsb_vehicle_send(mavlink_channel_t chan, uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, uint16_t ver_velocity, const char *callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_VEHICLE_LEN];
	_mav_put_uint32_t(buf, 0, ICAO_address);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_uint16_t(buf, 16, heading);
	_mav_put_uint16_t(buf, 18, hor_velocity);
	_mav_put_uint16_t(buf, 20, ver_velocity);
	_mav_put_uint16_t(buf, 22, flags);
	_mav_put_uint16_t(buf, 24, squawk);
	_mav_put_uint8_t(buf, 26, altitude_type);
	_mav_put_uint8_t(buf, 36, emitter_type);
	_mav_put_uint8_t(buf, 37, tslc);
	_mav_put_char_array(buf, 27, callsign, 9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_VEHICLE, buf, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN, MAVLINK_MSG_ID_ADSB_VEHICLE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_VEHICLE, buf, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#endif
#else
	mavlink_adsb_vehicle_t packet;
	packet.ICAO_address = ICAO_address;
	packet.lat = lat;
	packet.lon = lon;
	packet.altitude = altitude;
	packet.heading = heading;
	packet.hor_velocity = hor_velocity;
	packet.ver_velocity = ver_velocity;
	packet.flags = flags;
	packet.squawk = squawk;
	packet.altitude_type = altitude_type;
	packet.emitter_type = emitter_type;
	packet.tslc = tslc;
	mav_array_memcpy(packet.callsign, callsign, sizeof(char)*9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_VEHICLE, (const char *)&packet, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN, MAVLINK_MSG_ID_ADSB_VEHICLE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_VEHICLE, (const char *)&packet, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ADSB_VEHICLE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adsb_vehicle_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t ICAO_address, int32_t lat, int32_t lon, uint8_t altitude_type, int32_t altitude, uint16_t heading, uint16_t hor_velocity, uint16_t ver_velocity, const char *callsign, uint8_t emitter_type, uint8_t tslc, uint16_t flags, uint16_t squawk)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, ICAO_address);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, lon);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_uint16_t(buf, 16, heading);
	_mav_put_uint16_t(buf, 18, hor_velocity);
	_mav_put_uint16_t(buf, 20, ver_velocity);
	_mav_put_uint16_t(buf, 22, flags);
	_mav_put_uint16_t(buf, 24, squawk);
	_mav_put_uint8_t(buf, 26, altitude_type);
	_mav_put_uint8_t(buf, 36, emitter_type);
	_mav_put_uint8_t(buf, 37, tslc);
	_mav_put_char_array(buf, 27, callsign, 9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_VEHICLE, buf, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN, MAVLINK_MSG_ID_ADSB_VEHICLE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_VEHICLE, buf, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#endif
#else
	mavlink_adsb_vehicle_t *packet = (mavlink_adsb_vehicle_t *)msgbuf;
	packet->ICAO_address = ICAO_address;
	packet->lat = lat;
	packet->lon = lon;
	packet->altitude = altitude;
	packet->heading = heading;
	packet->hor_velocity = hor_velocity;
	packet->ver_velocity = ver_velocity;
	packet->flags = flags;
	packet->squawk = squawk;
	packet->altitude_type = altitude_type;
	packet->emitter_type = emitter_type;
	packet->tslc = tslc;
	mav_array_memcpy(packet->callsign, callsign, sizeof(char)*9);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_VEHICLE, (const char *)packet, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN, MAVLINK_MSG_ID_ADSB_VEHICLE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_VEHICLE, (const char *)packet, MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ADSB_VEHICLE UNPACKING


/**
 * @brief Get field ICAO_address from adsb_vehicle message
 *
 * @return ICAO address
 */
static inline uint32_t mavlink_msg_adsb_vehicle_get_ICAO_address(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field lat from adsb_vehicle message
 *
 * @return Latitude, expressed as degrees * 1E7
 */
static inline int32_t mavlink_msg_adsb_vehicle_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from adsb_vehicle message
 *
 * @return Longitude, expressed as degrees * 1E7
 */
static inline int32_t mavlink_msg_adsb_vehicle_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field altitude_type from adsb_vehicle message
 *
 * @return Type from ADSB_ALTITUDE_TYPE enum
 */
static inline uint8_t mavlink_msg_adsb_vehicle_get_altitude_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field altitude from adsb_vehicle message
 *
 * @return Altitude(ASL) in millimeters
 */
static inline int32_t mavlink_msg_adsb_vehicle_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field heading from adsb_vehicle message
 *
 * @return Course over ground in centidegrees
 */
static inline uint16_t mavlink_msg_adsb_vehicle_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field hor_velocity from adsb_vehicle message
 *
 * @return The horizontal velocity in centimeters/second
 */
static inline uint16_t mavlink_msg_adsb_vehicle_get_hor_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field ver_velocity from adsb_vehicle message
 *
 * @return The vertical velocity in centimeters/second, positive is up
 */
static inline uint16_t mavlink_msg_adsb_vehicle_get_ver_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field callsign from adsb_vehicle message
 *
 * @return The callsign, 8+null
 */
static inline uint16_t mavlink_msg_adsb_vehicle_get_callsign(const mavlink_message_t* msg, char *callsign)
{
	return _MAV_RETURN_char_array(msg, callsign, 9,  27);
}

/**
 * @brief Get field emitter_type from adsb_vehicle message
 *
 * @return Type from ADSB_EMITTER_TYPE enum
 */
static inline uint8_t mavlink_msg_adsb_vehicle_get_emitter_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field tslc from adsb_vehicle message
 *
 * @return Time since last communication in seconds
 */
static inline uint8_t mavlink_msg_adsb_vehicle_get_tslc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field flags from adsb_vehicle message
 *
 * @return Flags to indicate various statuses including valid data fields
 */
static inline uint16_t mavlink_msg_adsb_vehicle_get_flags(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field squawk from adsb_vehicle message
 *
 * @return Squawk code
 */
static inline uint16_t mavlink_msg_adsb_vehicle_get_squawk(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Decode a adsb_vehicle message into a struct
 *
 * @param msg The message to decode
 * @param adsb_vehicle C-struct to decode the message contents into
 */
static inline void mavlink_msg_adsb_vehicle_decode(const mavlink_message_t* msg, mavlink_adsb_vehicle_t* adsb_vehicle)
{
#if MAVLINK_NEED_BYTE_SWAP
	adsb_vehicle->ICAO_address = mavlink_msg_adsb_vehicle_get_ICAO_address(msg);
	adsb_vehicle->lat = mavlink_msg_adsb_vehicle_get_lat(msg);
	adsb_vehicle->lon = mavlink_msg_adsb_vehicle_get_lon(msg);
	adsb_vehicle->altitude = mavlink_msg_adsb_vehicle_get_altitude(msg);
	adsb_vehicle->heading = mavlink_msg_adsb_vehicle_get_heading(msg);
	adsb_vehicle->hor_velocity = mavlink_msg_adsb_vehicle_get_hor_velocity(msg);
	adsb_vehicle->ver_velocity = mavlink_msg_adsb_vehicle_get_ver_velocity(msg);
	adsb_vehicle->flags = mavlink_msg_adsb_vehicle_get_flags(msg);
	adsb_vehicle->squawk = mavlink_msg_adsb_vehicle_get_squawk(msg);
	adsb_vehicle->altitude_type = mavlink_msg_adsb_vehicle_get_altitude_type(msg);
	mavlink_msg_adsb_vehicle_get_callsign(msg, adsb_vehicle->callsign);
	adsb_vehicle->emitter_type = mavlink_msg_adsb_vehicle_get_emitter_type(msg);
	adsb_vehicle->tslc = mavlink_msg_adsb_vehicle_get_tslc(msg);
#else
	memcpy(adsb_vehicle, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ADSB_VEHICLE_LEN);
#endif
}

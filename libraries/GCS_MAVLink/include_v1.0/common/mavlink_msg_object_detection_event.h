// MESSAGE OBJECT_DETECTION_EVENT PACKING

#define MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT 140

typedef struct __mavlink_object_detection_event_t
{
 uint32_t time; ///< Timestamp in milliseconds since system boot
 float bearing; ///< Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 float distance; ///< Ground distance in meters
 uint16_t object_id; ///< Object ID
 uint8_t type; ///< Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 char name[20]; ///< Name of the object as defined by the detector
 uint8_t quality; ///< Detection quality / confidence. 0: bad, 255: maximum confidence
} mavlink_object_detection_event_t;

#define MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT_LEN 36
#define MAVLINK_MSG_ID_140_LEN 36

#define MAVLINK_MSG_OBJECT_DETECTION_EVENT_FIELD_NAME_LEN 20

#define MAVLINK_MESSAGE_INFO_OBJECT_DETECTION_EVENT { \
	"OBJECT_DETECTION_EVENT", \
	7, \
	{  { "time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_object_detection_event_t, time) }, \
         { "bearing", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_object_detection_event_t, bearing) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_object_detection_event_t, distance) }, \
         { "object_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_object_detection_event_t, object_id) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_object_detection_event_t, type) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 20, 15, offsetof(mavlink_object_detection_event_t, name) }, \
         { "quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_object_detection_event_t, quality) }, \
         } \
}


/**
 * @brief Pack a object_detection_event message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time Timestamp in milliseconds since system boot
 * @param object_id Object ID
 * @param type Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 * @param name Name of the object as defined by the detector
 * @param quality Detection quality / confidence. 0: bad, 255: maximum confidence
 * @param bearing Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 * @param distance Ground distance in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_object_detection_event_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time, uint16_t object_id, uint8_t type, const char *name, uint8_t quality, float bearing, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_float(buf, 4, bearing);
	_mav_put_float(buf, 8, distance);
	_mav_put_uint16_t(buf, 12, object_id);
	_mav_put_uint8_t(buf, 14, type);
	_mav_put_uint8_t(buf, 35, quality);
	_mav_put_char_array(buf, 15, name, 20);
        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
	mavlink_object_detection_event_t packet;
	packet.time = time;
	packet.bearing = bearing;
	packet.distance = distance;
	packet.object_id = object_id;
	packet.type = type;
	packet.quality = quality;
	mav_array_memcpy(packet.name, name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT;
	return mavlink_finalize_message(msg, system_id, component_id, 36, 179);
}

/**
 * @brief Pack a object_detection_event message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time Timestamp in milliseconds since system boot
 * @param object_id Object ID
 * @param type Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 * @param name Name of the object as defined by the detector
 * @param quality Detection quality / confidence. 0: bad, 255: maximum confidence
 * @param bearing Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 * @param distance Ground distance in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_object_detection_event_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time,uint16_t object_id,uint8_t type,const char *name,uint8_t quality,float bearing,float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_float(buf, 4, bearing);
	_mav_put_float(buf, 8, distance);
	_mav_put_uint16_t(buf, 12, object_id);
	_mav_put_uint8_t(buf, 14, type);
	_mav_put_uint8_t(buf, 35, quality);
	_mav_put_char_array(buf, 15, name, 20);
        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
	mavlink_object_detection_event_t packet;
	packet.time = time;
	packet.bearing = bearing;
	packet.distance = distance;
	packet.object_id = object_id;
	packet.type = type;
	packet.quality = quality;
	mav_array_memcpy(packet.name, name, sizeof(char)*20);
        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 179);
}

/**
 * @brief Encode a object_detection_event struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param object_detection_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_object_detection_event_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_object_detection_event_t* object_detection_event)
{
	return mavlink_msg_object_detection_event_pack(system_id, component_id, msg, object_detection_event->time, object_detection_event->object_id, object_detection_event->type, object_detection_event->name, object_detection_event->quality, object_detection_event->bearing, object_detection_event->distance);
}

/**
 * @brief Send a object_detection_event message
 * @param chan MAVLink channel to send the message
 *
 * @param time Timestamp in milliseconds since system boot
 * @param object_id Object ID
 * @param type Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 * @param name Name of the object as defined by the detector
 * @param quality Detection quality / confidence. 0: bad, 255: maximum confidence
 * @param bearing Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 * @param distance Ground distance in meters
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_object_detection_event_send(mavlink_channel_t chan, uint32_t time, uint16_t object_id, uint8_t type, const char *name, uint8_t quality, float bearing, float distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_uint32_t(buf, 0, time);
	_mav_put_float(buf, 4, bearing);
	_mav_put_float(buf, 8, distance);
	_mav_put_uint16_t(buf, 12, object_id);
	_mav_put_uint8_t(buf, 14, type);
	_mav_put_uint8_t(buf, 35, quality);
	_mav_put_char_array(buf, 15, name, 20);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT, buf, 36, 179);
#else
	mavlink_object_detection_event_t packet;
	packet.time = time;
	packet.bearing = bearing;
	packet.distance = distance;
	packet.object_id = object_id;
	packet.type = type;
	packet.quality = quality;
	mav_array_memcpy(packet.name, name, sizeof(char)*20);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBJECT_DETECTION_EVENT, (const char *)&packet, 36, 179);
#endif
}

#endif

// MESSAGE OBJECT_DETECTION_EVENT UNPACKING


/**
 * @brief Get field time from object_detection_event message
 *
 * @return Timestamp in milliseconds since system boot
 */
static inline uint32_t mavlink_msg_object_detection_event_get_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field object_id from object_detection_event message
 *
 * @return Object ID
 */
static inline uint16_t mavlink_msg_object_detection_event_get_object_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field type from object_detection_event message
 *
 * @return Object type: 0: image, 1: letter, 2: ground vehicle, 3: air vehicle, 4: surface vehicle, 5: sub-surface vehicle, 6: human, 7: animal
 */
static inline uint8_t mavlink_msg_object_detection_event_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field name from object_detection_event message
 *
 * @return Name of the object as defined by the detector
 */
static inline uint16_t mavlink_msg_object_detection_event_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 20,  15);
}

/**
 * @brief Get field quality from object_detection_event message
 *
 * @return Detection quality / confidence. 0: bad, 255: maximum confidence
 */
static inline uint8_t mavlink_msg_object_detection_event_get_quality(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field bearing from object_detection_event message
 *
 * @return Angle of the object with respect to the body frame in NED coordinates in radians. 0: front
 */
static inline float mavlink_msg_object_detection_event_get_bearing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field distance from object_detection_event message
 *
 * @return Ground distance in meters
 */
static inline float mavlink_msg_object_detection_event_get_distance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a object_detection_event message into a struct
 *
 * @param msg The message to decode
 * @param object_detection_event C-struct to decode the message contents into
 */
static inline void mavlink_msg_object_detection_event_decode(const mavlink_message_t* msg, mavlink_object_detection_event_t* object_detection_event)
{
#if MAVLINK_NEED_BYTE_SWAP
	object_detection_event->time = mavlink_msg_object_detection_event_get_time(msg);
	object_detection_event->bearing = mavlink_msg_object_detection_event_get_bearing(msg);
	object_detection_event->distance = mavlink_msg_object_detection_event_get_distance(msg);
	object_detection_event->object_id = mavlink_msg_object_detection_event_get_object_id(msg);
	object_detection_event->type = mavlink_msg_object_detection_event_get_type(msg);
	mavlink_msg_object_detection_event_get_name(msg, object_detection_event->name);
	object_detection_event->quality = mavlink_msg_object_detection_event_get_quality(msg);
#else
	memcpy(object_detection_event, _MAV_PAYLOAD(msg), 36);
#endif
}

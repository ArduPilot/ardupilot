// MESSAGE GPS_RTK PACKING

#define MAVLINK_MSG_ID_GPS_RTK 127

typedef struct __mavlink_gps_rtk_t
{
 uint32_t time_last_baseline_ms; ///< Time since boot of last baseline message received in ms.
 uint32_t tow; ///< GPS Time of Week of last baseline
 int32_t baseline_a_mm; ///< Current baseline in ECEF x or NED north component in mm.
 int32_t baseline_b_mm; ///< Current baseline in ECEF y or NED east component in mm.
 int32_t baseline_c_mm; ///< Current baseline in ECEF z or NED down component in mm.
 uint32_t accuracy; ///< Current estimate of baseline accuracy.
 int32_t iar_num_hypotheses; ///< Current number of integer ambiguity hypotheses.
 uint16_t wn; ///< GPS Week Number of last baseline
 uint8_t rtk_receiver_id; ///< Identification of connected RTK receiver.
 uint8_t rtk_health; ///< GPS-specific health report for RTK data.
 uint8_t rtk_rate; ///< Rate of baseline messages being received by GPS, in HZ
 uint8_t nsats; ///< Current number of sats used for RTK calculation.
 uint8_t baseline_coords_type; ///< Coordinate system of baseline. 0 == ECEF, 1 == NED
} mavlink_gps_rtk_t;

#define MAVLINK_MSG_ID_GPS_RTK_LEN 35
#define MAVLINK_MSG_ID_127_LEN 35

#define MAVLINK_MSG_ID_GPS_RTK_CRC 25
#define MAVLINK_MSG_ID_127_CRC 25



#define MAVLINK_MESSAGE_INFO_GPS_RTK { \
	"GPS_RTK", \
	13, \
	{  { "time_last_baseline_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gps_rtk_t, time_last_baseline_ms) }, \
         { "tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_gps_rtk_t, tow) }, \
         { "baseline_a_mm", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gps_rtk_t, baseline_a_mm) }, \
         { "baseline_b_mm", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gps_rtk_t, baseline_b_mm) }, \
         { "baseline_c_mm", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gps_rtk_t, baseline_c_mm) }, \
         { "accuracy", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_gps_rtk_t, accuracy) }, \
         { "iar_num_hypotheses", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_gps_rtk_t, iar_num_hypotheses) }, \
         { "wn", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_gps_rtk_t, wn) }, \
         { "rtk_receiver_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_gps_rtk_t, rtk_receiver_id) }, \
         { "rtk_health", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_gps_rtk_t, rtk_health) }, \
         { "rtk_rate", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_gps_rtk_t, rtk_rate) }, \
         { "nsats", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_gps_rtk_t, nsats) }, \
         { "baseline_coords_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_gps_rtk_t, baseline_coords_type) }, \
         } \
}


/**
 * @brief Pack a gps_rtk message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_last_baseline_ms Time since boot of last baseline message received in ms.
 * @param rtk_receiver_id Identification of connected RTK receiver.
 * @param wn GPS Week Number of last baseline
 * @param tow GPS Time of Week of last baseline
 * @param rtk_health GPS-specific health report for RTK data.
 * @param rtk_rate Rate of baseline messages being received by GPS, in HZ
 * @param nsats Current number of sats used for RTK calculation.
 * @param baseline_coords_type Coordinate system of baseline. 0 == ECEF, 1 == NED
 * @param baseline_a_mm Current baseline in ECEF x or NED north component in mm.
 * @param baseline_b_mm Current baseline in ECEF y or NED east component in mm.
 * @param baseline_c_mm Current baseline in ECEF z or NED down component in mm.
 * @param accuracy Current estimate of baseline accuracy.
 * @param iar_num_hypotheses Current number of integer ambiguity hypotheses.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_rtk_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_RTK_LEN];
	_mav_put_uint32_t(buf, 0, time_last_baseline_ms);
	_mav_put_uint32_t(buf, 4, tow);
	_mav_put_int32_t(buf, 8, baseline_a_mm);
	_mav_put_int32_t(buf, 12, baseline_b_mm);
	_mav_put_int32_t(buf, 16, baseline_c_mm);
	_mav_put_uint32_t(buf, 20, accuracy);
	_mav_put_int32_t(buf, 24, iar_num_hypotheses);
	_mav_put_uint16_t(buf, 28, wn);
	_mav_put_uint8_t(buf, 30, rtk_receiver_id);
	_mav_put_uint8_t(buf, 31, rtk_health);
	_mav_put_uint8_t(buf, 32, rtk_rate);
	_mav_put_uint8_t(buf, 33, nsats);
	_mav_put_uint8_t(buf, 34, baseline_coords_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_RTK_LEN);
#else
	mavlink_gps_rtk_t packet;
	packet.time_last_baseline_ms = time_last_baseline_ms;
	packet.tow = tow;
	packet.baseline_a_mm = baseline_a_mm;
	packet.baseline_b_mm = baseline_b_mm;
	packet.baseline_c_mm = baseline_c_mm;
	packet.accuracy = accuracy;
	packet.iar_num_hypotheses = iar_num_hypotheses;
	packet.wn = wn;
	packet.rtk_receiver_id = rtk_receiver_id;
	packet.rtk_health = rtk_health;
	packet.rtk_rate = rtk_rate;
	packet.nsats = nsats;
	packet.baseline_coords_type = baseline_coords_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_RTK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_RTK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_RTK_LEN, MAVLINK_MSG_ID_GPS_RTK_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_RTK_LEN);
#endif
}

/**
 * @brief Pack a gps_rtk message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_last_baseline_ms Time since boot of last baseline message received in ms.
 * @param rtk_receiver_id Identification of connected RTK receiver.
 * @param wn GPS Week Number of last baseline
 * @param tow GPS Time of Week of last baseline
 * @param rtk_health GPS-specific health report for RTK data.
 * @param rtk_rate Rate of baseline messages being received by GPS, in HZ
 * @param nsats Current number of sats used for RTK calculation.
 * @param baseline_coords_type Coordinate system of baseline. 0 == ECEF, 1 == NED
 * @param baseline_a_mm Current baseline in ECEF x or NED north component in mm.
 * @param baseline_b_mm Current baseline in ECEF y or NED east component in mm.
 * @param baseline_c_mm Current baseline in ECEF z or NED down component in mm.
 * @param accuracy Current estimate of baseline accuracy.
 * @param iar_num_hypotheses Current number of integer ambiguity hypotheses.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_rtk_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_last_baseline_ms,uint8_t rtk_receiver_id,uint16_t wn,uint32_t tow,uint8_t rtk_health,uint8_t rtk_rate,uint8_t nsats,uint8_t baseline_coords_type,int32_t baseline_a_mm,int32_t baseline_b_mm,int32_t baseline_c_mm,uint32_t accuracy,int32_t iar_num_hypotheses)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_RTK_LEN];
	_mav_put_uint32_t(buf, 0, time_last_baseline_ms);
	_mav_put_uint32_t(buf, 4, tow);
	_mav_put_int32_t(buf, 8, baseline_a_mm);
	_mav_put_int32_t(buf, 12, baseline_b_mm);
	_mav_put_int32_t(buf, 16, baseline_c_mm);
	_mav_put_uint32_t(buf, 20, accuracy);
	_mav_put_int32_t(buf, 24, iar_num_hypotheses);
	_mav_put_uint16_t(buf, 28, wn);
	_mav_put_uint8_t(buf, 30, rtk_receiver_id);
	_mav_put_uint8_t(buf, 31, rtk_health);
	_mav_put_uint8_t(buf, 32, rtk_rate);
	_mav_put_uint8_t(buf, 33, nsats);
	_mav_put_uint8_t(buf, 34, baseline_coords_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_RTK_LEN);
#else
	mavlink_gps_rtk_t packet;
	packet.time_last_baseline_ms = time_last_baseline_ms;
	packet.tow = tow;
	packet.baseline_a_mm = baseline_a_mm;
	packet.baseline_b_mm = baseline_b_mm;
	packet.baseline_c_mm = baseline_c_mm;
	packet.accuracy = accuracy;
	packet.iar_num_hypotheses = iar_num_hypotheses;
	packet.wn = wn;
	packet.rtk_receiver_id = rtk_receiver_id;
	packet.rtk_health = rtk_health;
	packet.rtk_rate = rtk_rate;
	packet.nsats = nsats;
	packet.baseline_coords_type = baseline_coords_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_RTK_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_RTK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_RTK_LEN, MAVLINK_MSG_ID_GPS_RTK_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_RTK_LEN);
#endif
}

/**
 * @brief Encode a gps_rtk struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_rtk C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_rtk_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_rtk_t* gps_rtk)
{
	return mavlink_msg_gps_rtk_pack(system_id, component_id, msg, gps_rtk->time_last_baseline_ms, gps_rtk->rtk_receiver_id, gps_rtk->wn, gps_rtk->tow, gps_rtk->rtk_health, gps_rtk->rtk_rate, gps_rtk->nsats, gps_rtk->baseline_coords_type, gps_rtk->baseline_a_mm, gps_rtk->baseline_b_mm, gps_rtk->baseline_c_mm, gps_rtk->accuracy, gps_rtk->iar_num_hypotheses);
}

/**
 * @brief Encode a gps_rtk struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_rtk C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_rtk_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_rtk_t* gps_rtk)
{
	return mavlink_msg_gps_rtk_pack_chan(system_id, component_id, chan, msg, gps_rtk->time_last_baseline_ms, gps_rtk->rtk_receiver_id, gps_rtk->wn, gps_rtk->tow, gps_rtk->rtk_health, gps_rtk->rtk_rate, gps_rtk->nsats, gps_rtk->baseline_coords_type, gps_rtk->baseline_a_mm, gps_rtk->baseline_b_mm, gps_rtk->baseline_c_mm, gps_rtk->accuracy, gps_rtk->iar_num_hypotheses);
}

/**
 * @brief Send a gps_rtk message
 * @param chan MAVLink channel to send the message
 *
 * @param time_last_baseline_ms Time since boot of last baseline message received in ms.
 * @param rtk_receiver_id Identification of connected RTK receiver.
 * @param wn GPS Week Number of last baseline
 * @param tow GPS Time of Week of last baseline
 * @param rtk_health GPS-specific health report for RTK data.
 * @param rtk_rate Rate of baseline messages being received by GPS, in HZ
 * @param nsats Current number of sats used for RTK calculation.
 * @param baseline_coords_type Coordinate system of baseline. 0 == ECEF, 1 == NED
 * @param baseline_a_mm Current baseline in ECEF x or NED north component in mm.
 * @param baseline_b_mm Current baseline in ECEF y or NED east component in mm.
 * @param baseline_c_mm Current baseline in ECEF z or NED down component in mm.
 * @param accuracy Current estimate of baseline accuracy.
 * @param iar_num_hypotheses Current number of integer ambiguity hypotheses.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_rtk_send(mavlink_channel_t chan, uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_RTK_LEN];
	_mav_put_uint32_t(buf, 0, time_last_baseline_ms);
	_mav_put_uint32_t(buf, 4, tow);
	_mav_put_int32_t(buf, 8, baseline_a_mm);
	_mav_put_int32_t(buf, 12, baseline_b_mm);
	_mav_put_int32_t(buf, 16, baseline_c_mm);
	_mav_put_uint32_t(buf, 20, accuracy);
	_mav_put_int32_t(buf, 24, iar_num_hypotheses);
	_mav_put_uint16_t(buf, 28, wn);
	_mav_put_uint8_t(buf, 30, rtk_receiver_id);
	_mav_put_uint8_t(buf, 31, rtk_health);
	_mav_put_uint8_t(buf, 32, rtk_rate);
	_mav_put_uint8_t(buf, 33, nsats);
	_mav_put_uint8_t(buf, 34, baseline_coords_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTK, buf, MAVLINK_MSG_ID_GPS_RTK_LEN, MAVLINK_MSG_ID_GPS_RTK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTK, buf, MAVLINK_MSG_ID_GPS_RTK_LEN);
#endif
#else
	mavlink_gps_rtk_t packet;
	packet.time_last_baseline_ms = time_last_baseline_ms;
	packet.tow = tow;
	packet.baseline_a_mm = baseline_a_mm;
	packet.baseline_b_mm = baseline_b_mm;
	packet.baseline_c_mm = baseline_c_mm;
	packet.accuracy = accuracy;
	packet.iar_num_hypotheses = iar_num_hypotheses;
	packet.wn = wn;
	packet.rtk_receiver_id = rtk_receiver_id;
	packet.rtk_health = rtk_health;
	packet.rtk_rate = rtk_rate;
	packet.nsats = nsats;
	packet.baseline_coords_type = baseline_coords_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTK, (const char *)&packet, MAVLINK_MSG_ID_GPS_RTK_LEN, MAVLINK_MSG_ID_GPS_RTK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTK, (const char *)&packet, MAVLINK_MSG_ID_GPS_RTK_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GPS_RTK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_rtk_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_last_baseline_ms, uint8_t rtk_receiver_id, uint16_t wn, uint32_t tow, uint8_t rtk_health, uint8_t rtk_rate, uint8_t nsats, uint8_t baseline_coords_type, int32_t baseline_a_mm, int32_t baseline_b_mm, int32_t baseline_c_mm, uint32_t accuracy, int32_t iar_num_hypotheses)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_last_baseline_ms);
	_mav_put_uint32_t(buf, 4, tow);
	_mav_put_int32_t(buf, 8, baseline_a_mm);
	_mav_put_int32_t(buf, 12, baseline_b_mm);
	_mav_put_int32_t(buf, 16, baseline_c_mm);
	_mav_put_uint32_t(buf, 20, accuracy);
	_mav_put_int32_t(buf, 24, iar_num_hypotheses);
	_mav_put_uint16_t(buf, 28, wn);
	_mav_put_uint8_t(buf, 30, rtk_receiver_id);
	_mav_put_uint8_t(buf, 31, rtk_health);
	_mav_put_uint8_t(buf, 32, rtk_rate);
	_mav_put_uint8_t(buf, 33, nsats);
	_mav_put_uint8_t(buf, 34, baseline_coords_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTK, buf, MAVLINK_MSG_ID_GPS_RTK_LEN, MAVLINK_MSG_ID_GPS_RTK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTK, buf, MAVLINK_MSG_ID_GPS_RTK_LEN);
#endif
#else
	mavlink_gps_rtk_t *packet = (mavlink_gps_rtk_t *)msgbuf;
	packet->time_last_baseline_ms = time_last_baseline_ms;
	packet->tow = tow;
	packet->baseline_a_mm = baseline_a_mm;
	packet->baseline_b_mm = baseline_b_mm;
	packet->baseline_c_mm = baseline_c_mm;
	packet->accuracy = accuracy;
	packet->iar_num_hypotheses = iar_num_hypotheses;
	packet->wn = wn;
	packet->rtk_receiver_id = rtk_receiver_id;
	packet->rtk_health = rtk_health;
	packet->rtk_rate = rtk_rate;
	packet->nsats = nsats;
	packet->baseline_coords_type = baseline_coords_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTK, (const char *)packet, MAVLINK_MSG_ID_GPS_RTK_LEN, MAVLINK_MSG_ID_GPS_RTK_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RTK, (const char *)packet, MAVLINK_MSG_ID_GPS_RTK_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GPS_RTK UNPACKING


/**
 * @brief Get field time_last_baseline_ms from gps_rtk message
 *
 * @return Time since boot of last baseline message received in ms.
 */
static inline uint32_t mavlink_msg_gps_rtk_get_time_last_baseline_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field rtk_receiver_id from gps_rtk message
 *
 * @return Identification of connected RTK receiver.
 */
static inline uint8_t mavlink_msg_gps_rtk_get_rtk_receiver_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field wn from gps_rtk message
 *
 * @return GPS Week Number of last baseline
 */
static inline uint16_t mavlink_msg_gps_rtk_get_wn(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field tow from gps_rtk message
 *
 * @return GPS Time of Week of last baseline
 */
static inline uint32_t mavlink_msg_gps_rtk_get_tow(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field rtk_health from gps_rtk message
 *
 * @return GPS-specific health report for RTK data.
 */
static inline uint8_t mavlink_msg_gps_rtk_get_rtk_health(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field rtk_rate from gps_rtk message
 *
 * @return Rate of baseline messages being received by GPS, in HZ
 */
static inline uint8_t mavlink_msg_gps_rtk_get_rtk_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field nsats from gps_rtk message
 *
 * @return Current number of sats used for RTK calculation.
 */
static inline uint8_t mavlink_msg_gps_rtk_get_nsats(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field baseline_coords_type from gps_rtk message
 *
 * @return Coordinate system of baseline. 0 == ECEF, 1 == NED
 */
static inline uint8_t mavlink_msg_gps_rtk_get_baseline_coords_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field baseline_a_mm from gps_rtk message
 *
 * @return Current baseline in ECEF x or NED north component in mm.
 */
static inline int32_t mavlink_msg_gps_rtk_get_baseline_a_mm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field baseline_b_mm from gps_rtk message
 *
 * @return Current baseline in ECEF y or NED east component in mm.
 */
static inline int32_t mavlink_msg_gps_rtk_get_baseline_b_mm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field baseline_c_mm from gps_rtk message
 *
 * @return Current baseline in ECEF z or NED down component in mm.
 */
static inline int32_t mavlink_msg_gps_rtk_get_baseline_c_mm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field accuracy from gps_rtk message
 *
 * @return Current estimate of baseline accuracy.
 */
static inline uint32_t mavlink_msg_gps_rtk_get_accuracy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field iar_num_hypotheses from gps_rtk message
 *
 * @return Current number of integer ambiguity hypotheses.
 */
static inline int32_t mavlink_msg_gps_rtk_get_iar_num_hypotheses(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Decode a gps_rtk message into a struct
 *
 * @param msg The message to decode
 * @param gps_rtk C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_rtk_decode(const mavlink_message_t* msg, mavlink_gps_rtk_t* gps_rtk)
{
#if MAVLINK_NEED_BYTE_SWAP
	gps_rtk->time_last_baseline_ms = mavlink_msg_gps_rtk_get_time_last_baseline_ms(msg);
	gps_rtk->tow = mavlink_msg_gps_rtk_get_tow(msg);
	gps_rtk->baseline_a_mm = mavlink_msg_gps_rtk_get_baseline_a_mm(msg);
	gps_rtk->baseline_b_mm = mavlink_msg_gps_rtk_get_baseline_b_mm(msg);
	gps_rtk->baseline_c_mm = mavlink_msg_gps_rtk_get_baseline_c_mm(msg);
	gps_rtk->accuracy = mavlink_msg_gps_rtk_get_accuracy(msg);
	gps_rtk->iar_num_hypotheses = mavlink_msg_gps_rtk_get_iar_num_hypotheses(msg);
	gps_rtk->wn = mavlink_msg_gps_rtk_get_wn(msg);
	gps_rtk->rtk_receiver_id = mavlink_msg_gps_rtk_get_rtk_receiver_id(msg);
	gps_rtk->rtk_health = mavlink_msg_gps_rtk_get_rtk_health(msg);
	gps_rtk->rtk_rate = mavlink_msg_gps_rtk_get_rtk_rate(msg);
	gps_rtk->nsats = mavlink_msg_gps_rtk_get_nsats(msg);
	gps_rtk->baseline_coords_type = mavlink_msg_gps_rtk_get_baseline_coords_type(msg);
#else
	memcpy(gps_rtk, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GPS_RTK_LEN);
#endif
}

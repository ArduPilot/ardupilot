// MESSAGE GPS_ACCURACY PACKING

#define MAVLINK_MSG_ID_GPS_ACCURACY 225

typedef struct __mavlink_gps_accuracy_t
{
 float h_acc; /*< GPS-reported horizontal accuracy*/
 float s_acc; /*< GPS-reported speed accuracy*/
 float h_vel_filt; /*< GPS-reported, filtered horizontal velocity*/
 float v_vel_filt; /*< GPS-reported, filtered vertical velocity*/
 float p_drift; /*< GPS position drift*/
 uint8_t instance; /*< Which instance of GPS we're reporting on*/
 uint8_t ekf_check_mask; /*< Which fields pass EKF checks*/
} mavlink_gps_accuracy_t;

#define MAVLINK_MSG_ID_GPS_ACCURACY_LEN 22
#define MAVLINK_MSG_ID_225_LEN 22

#define MAVLINK_MSG_ID_GPS_ACCURACY_CRC 76
#define MAVLINK_MSG_ID_225_CRC 76



#define MAVLINK_MESSAGE_INFO_GPS_ACCURACY { \
	"GPS_ACCURACY", \
	7, \
	{  { "h_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gps_accuracy_t, h_acc) }, \
         { "s_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gps_accuracy_t, s_acc) }, \
         { "h_vel_filt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gps_accuracy_t, h_vel_filt) }, \
         { "v_vel_filt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gps_accuracy_t, v_vel_filt) }, \
         { "p_drift", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gps_accuracy_t, p_drift) }, \
         { "instance", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_gps_accuracy_t, instance) }, \
         { "ekf_check_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_gps_accuracy_t, ekf_check_mask) }, \
         } \
}


/**
 * @brief Pack a gps_accuracy message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param instance Which instance of GPS we're reporting on
 * @param h_acc GPS-reported horizontal accuracy
 * @param s_acc GPS-reported speed accuracy
 * @param h_vel_filt GPS-reported, filtered horizontal velocity
 * @param v_vel_filt GPS-reported, filtered vertical velocity
 * @param p_drift GPS position drift
 * @param ekf_check_mask Which fields pass EKF checks
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_accuracy_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t instance, float h_acc, float s_acc, float h_vel_filt, float v_vel_filt, float p_drift, uint8_t ekf_check_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_ACCURACY_LEN];
	_mav_put_float(buf, 0, h_acc);
	_mav_put_float(buf, 4, s_acc);
	_mav_put_float(buf, 8, h_vel_filt);
	_mav_put_float(buf, 12, v_vel_filt);
	_mav_put_float(buf, 16, p_drift);
	_mav_put_uint8_t(buf, 20, instance);
	_mav_put_uint8_t(buf, 21, ekf_check_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#else
	mavlink_gps_accuracy_t packet;
	packet.h_acc = h_acc;
	packet.s_acc = s_acc;
	packet.h_vel_filt = h_vel_filt;
	packet.v_vel_filt = v_vel_filt;
	packet.p_drift = p_drift;
	packet.instance = instance;
	packet.ekf_check_mask = ekf_check_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_ACCURACY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_ACCURACY_LEN, MAVLINK_MSG_ID_GPS_ACCURACY_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#endif
}

/**
 * @brief Pack a gps_accuracy message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param instance Which instance of GPS we're reporting on
 * @param h_acc GPS-reported horizontal accuracy
 * @param s_acc GPS-reported speed accuracy
 * @param h_vel_filt GPS-reported, filtered horizontal velocity
 * @param v_vel_filt GPS-reported, filtered vertical velocity
 * @param p_drift GPS position drift
 * @param ekf_check_mask Which fields pass EKF checks
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_accuracy_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t instance,float h_acc,float s_acc,float h_vel_filt,float v_vel_filt,float p_drift,uint8_t ekf_check_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_ACCURACY_LEN];
	_mav_put_float(buf, 0, h_acc);
	_mav_put_float(buf, 4, s_acc);
	_mav_put_float(buf, 8, h_vel_filt);
	_mav_put_float(buf, 12, v_vel_filt);
	_mav_put_float(buf, 16, p_drift);
	_mav_put_uint8_t(buf, 20, instance);
	_mav_put_uint8_t(buf, 21, ekf_check_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#else
	mavlink_gps_accuracy_t packet;
	packet.h_acc = h_acc;
	packet.s_acc = s_acc;
	packet.h_vel_filt = h_vel_filt;
	packet.v_vel_filt = v_vel_filt;
	packet.p_drift = p_drift;
	packet.instance = instance;
	packet.ekf_check_mask = ekf_check_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_ACCURACY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_ACCURACY_LEN, MAVLINK_MSG_ID_GPS_ACCURACY_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#endif
}

/**
 * @brief Encode a gps_accuracy struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_accuracy C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_accuracy_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_accuracy_t* gps_accuracy)
{
	return mavlink_msg_gps_accuracy_pack(system_id, component_id, msg, gps_accuracy->instance, gps_accuracy->h_acc, gps_accuracy->s_acc, gps_accuracy->h_vel_filt, gps_accuracy->v_vel_filt, gps_accuracy->p_drift, gps_accuracy->ekf_check_mask);
}

/**
 * @brief Encode a gps_accuracy struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_accuracy C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_accuracy_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_accuracy_t* gps_accuracy)
{
	return mavlink_msg_gps_accuracy_pack_chan(system_id, component_id, chan, msg, gps_accuracy->instance, gps_accuracy->h_acc, gps_accuracy->s_acc, gps_accuracy->h_vel_filt, gps_accuracy->v_vel_filt, gps_accuracy->p_drift, gps_accuracy->ekf_check_mask);
}

/**
 * @brief Send a gps_accuracy message
 * @param chan MAVLink channel to send the message
 *
 * @param instance Which instance of GPS we're reporting on
 * @param h_acc GPS-reported horizontal accuracy
 * @param s_acc GPS-reported speed accuracy
 * @param h_vel_filt GPS-reported, filtered horizontal velocity
 * @param v_vel_filt GPS-reported, filtered vertical velocity
 * @param p_drift GPS position drift
 * @param ekf_check_mask Which fields pass EKF checks
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_accuracy_send(mavlink_channel_t chan, uint8_t instance, float h_acc, float s_acc, float h_vel_filt, float v_vel_filt, float p_drift, uint8_t ekf_check_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_ACCURACY_LEN];
	_mav_put_float(buf, 0, h_acc);
	_mav_put_float(buf, 4, s_acc);
	_mav_put_float(buf, 8, h_vel_filt);
	_mav_put_float(buf, 12, v_vel_filt);
	_mav_put_float(buf, 16, p_drift);
	_mav_put_uint8_t(buf, 20, instance);
	_mav_put_uint8_t(buf, 21, ekf_check_mask);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_ACCURACY, buf, MAVLINK_MSG_ID_GPS_ACCURACY_LEN, MAVLINK_MSG_ID_GPS_ACCURACY_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_ACCURACY, buf, MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#endif
#else
	mavlink_gps_accuracy_t packet;
	packet.h_acc = h_acc;
	packet.s_acc = s_acc;
	packet.h_vel_filt = h_vel_filt;
	packet.v_vel_filt = v_vel_filt;
	packet.p_drift = p_drift;
	packet.instance = instance;
	packet.ekf_check_mask = ekf_check_mask;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_ACCURACY, (const char *)&packet, MAVLINK_MSG_ID_GPS_ACCURACY_LEN, MAVLINK_MSG_ID_GPS_ACCURACY_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_ACCURACY, (const char *)&packet, MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GPS_ACCURACY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_accuracy_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t instance, float h_acc, float s_acc, float h_vel_filt, float v_vel_filt, float p_drift, uint8_t ekf_check_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, h_acc);
	_mav_put_float(buf, 4, s_acc);
	_mav_put_float(buf, 8, h_vel_filt);
	_mav_put_float(buf, 12, v_vel_filt);
	_mav_put_float(buf, 16, p_drift);
	_mav_put_uint8_t(buf, 20, instance);
	_mav_put_uint8_t(buf, 21, ekf_check_mask);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_ACCURACY, buf, MAVLINK_MSG_ID_GPS_ACCURACY_LEN, MAVLINK_MSG_ID_GPS_ACCURACY_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_ACCURACY, buf, MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#endif
#else
	mavlink_gps_accuracy_t *packet = (mavlink_gps_accuracy_t *)msgbuf;
	packet->h_acc = h_acc;
	packet->s_acc = s_acc;
	packet->h_vel_filt = h_vel_filt;
	packet->v_vel_filt = v_vel_filt;
	packet->p_drift = p_drift;
	packet->instance = instance;
	packet->ekf_check_mask = ekf_check_mask;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_ACCURACY, (const char *)packet, MAVLINK_MSG_ID_GPS_ACCURACY_LEN, MAVLINK_MSG_ID_GPS_ACCURACY_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_ACCURACY, (const char *)packet, MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GPS_ACCURACY UNPACKING


/**
 * @brief Get field instance from gps_accuracy message
 *
 * @return Which instance of GPS we're reporting on
 */
static inline uint8_t mavlink_msg_gps_accuracy_get_instance(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field h_acc from gps_accuracy message
 *
 * @return GPS-reported horizontal accuracy
 */
static inline float mavlink_msg_gps_accuracy_get_h_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field s_acc from gps_accuracy message
 *
 * @return GPS-reported speed accuracy
 */
static inline float mavlink_msg_gps_accuracy_get_s_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field h_vel_filt from gps_accuracy message
 *
 * @return GPS-reported, filtered horizontal velocity
 */
static inline float mavlink_msg_gps_accuracy_get_h_vel_filt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field v_vel_filt from gps_accuracy message
 *
 * @return GPS-reported, filtered vertical velocity
 */
static inline float mavlink_msg_gps_accuracy_get_v_vel_filt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field p_drift from gps_accuracy message
 *
 * @return GPS position drift
 */
static inline float mavlink_msg_gps_accuracy_get_p_drift(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ekf_check_mask from gps_accuracy message
 *
 * @return Which fields pass EKF checks
 */
static inline uint8_t mavlink_msg_gps_accuracy_get_ekf_check_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Decode a gps_accuracy message into a struct
 *
 * @param msg The message to decode
 * @param gps_accuracy C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_accuracy_decode(const mavlink_message_t* msg, mavlink_gps_accuracy_t* gps_accuracy)
{
#if MAVLINK_NEED_BYTE_SWAP
	gps_accuracy->h_acc = mavlink_msg_gps_accuracy_get_h_acc(msg);
	gps_accuracy->s_acc = mavlink_msg_gps_accuracy_get_s_acc(msg);
	gps_accuracy->h_vel_filt = mavlink_msg_gps_accuracy_get_h_vel_filt(msg);
	gps_accuracy->v_vel_filt = mavlink_msg_gps_accuracy_get_v_vel_filt(msg);
	gps_accuracy->p_drift = mavlink_msg_gps_accuracy_get_p_drift(msg);
	gps_accuracy->instance = mavlink_msg_gps_accuracy_get_instance(msg);
	gps_accuracy->ekf_check_mask = mavlink_msg_gps_accuracy_get_ekf_check_mask(msg);
#else
	memcpy(gps_accuracy, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GPS_ACCURACY_LEN);
#endif
}

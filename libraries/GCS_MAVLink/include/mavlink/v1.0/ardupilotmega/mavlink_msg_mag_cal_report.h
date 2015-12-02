// MESSAGE MAG_CAL_REPORT PACKING

#define MAVLINK_MSG_ID_MAG_CAL_REPORT 192

typedef struct __mavlink_mag_cal_report_t
{
 float fitness; /*< RMS milligauss residuals*/
 float ofs_x; /*< X offset*/
 float ofs_y; /*< Y offset*/
 float ofs_z; /*< Z offset*/
 float diag_x; /*< X diagonal (matrix 11)*/
 float diag_y; /*< Y diagonal (matrix 22)*/
 float diag_z; /*< Z diagonal (matrix 33)*/
 float offdiag_x; /*< X off-diagonal (matrix 12 and 21)*/
 float offdiag_y; /*< Y off-diagonal (matrix 13 and 31)*/
 float offdiag_z; /*< Z off-diagonal (matrix 32 and 23)*/
 uint8_t compass_id; /*< Compass being calibrated*/
 uint8_t cal_mask; /*< Bitmask of compasses being calibrated*/
 uint8_t cal_status; /*< Status (see MAG_CAL_STATUS enum)*/
 uint8_t autosaved; /*< 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters*/
} mavlink_mag_cal_report_t;

#define MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN 44
#define MAVLINK_MSG_ID_192_LEN 44

#define MAVLINK_MSG_ID_MAG_CAL_REPORT_CRC 36
#define MAVLINK_MSG_ID_192_CRC 36



#define MAVLINK_MESSAGE_INFO_MAG_CAL_REPORT { \
	"MAG_CAL_REPORT", \
	14, \
	{  { "fitness", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mag_cal_report_t, fitness) }, \
         { "ofs_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mag_cal_report_t, ofs_x) }, \
         { "ofs_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mag_cal_report_t, ofs_y) }, \
         { "ofs_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mag_cal_report_t, ofs_z) }, \
         { "diag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mag_cal_report_t, diag_x) }, \
         { "diag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mag_cal_report_t, diag_y) }, \
         { "diag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mag_cal_report_t, diag_z) }, \
         { "offdiag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_mag_cal_report_t, offdiag_x) }, \
         { "offdiag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_mag_cal_report_t, offdiag_y) }, \
         { "offdiag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_mag_cal_report_t, offdiag_z) }, \
         { "compass_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_mag_cal_report_t, compass_id) }, \
         { "cal_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_mag_cal_report_t, cal_mask) }, \
         { "cal_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_mag_cal_report_t, cal_status) }, \
         { "autosaved", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_mag_cal_report_t, autosaved) }, \
         } \
}


/**
 * @brief Pack a mag_cal_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param compass_id Compass being calibrated
 * @param cal_mask Bitmask of compasses being calibrated
 * @param cal_status Status (see MAG_CAL_STATUS enum)
 * @param autosaved 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
 * @param fitness RMS milligauss residuals
 * @param ofs_x X offset
 * @param ofs_y Y offset
 * @param ofs_z Z offset
 * @param diag_x X diagonal (matrix 11)
 * @param diag_y Y diagonal (matrix 22)
 * @param diag_z Z diagonal (matrix 33)
 * @param offdiag_x X off-diagonal (matrix 12 and 21)
 * @param offdiag_y Y off-diagonal (matrix 13 and 31)
 * @param offdiag_z Z off-diagonal (matrix 32 and 23)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mag_cal_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN];
	_mav_put_float(buf, 0, fitness);
	_mav_put_float(buf, 4, ofs_x);
	_mav_put_float(buf, 8, ofs_y);
	_mav_put_float(buf, 12, ofs_z);
	_mav_put_float(buf, 16, diag_x);
	_mav_put_float(buf, 20, diag_y);
	_mav_put_float(buf, 24, diag_z);
	_mav_put_float(buf, 28, offdiag_x);
	_mav_put_float(buf, 32, offdiag_y);
	_mav_put_float(buf, 36, offdiag_z);
	_mav_put_uint8_t(buf, 40, compass_id);
	_mav_put_uint8_t(buf, 41, cal_mask);
	_mav_put_uint8_t(buf, 42, cal_status);
	_mav_put_uint8_t(buf, 43, autosaved);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#else
	mavlink_mag_cal_report_t packet;
	packet.fitness = fitness;
	packet.ofs_x = ofs_x;
	packet.ofs_y = ofs_y;
	packet.ofs_z = ofs_z;
	packet.diag_x = diag_x;
	packet.diag_y = diag_y;
	packet.diag_z = diag_z;
	packet.offdiag_x = offdiag_x;
	packet.offdiag_y = offdiag_y;
	packet.offdiag_z = offdiag_z;
	packet.compass_id = compass_id;
	packet.cal_mask = cal_mask;
	packet.cal_status = cal_status;
	packet.autosaved = autosaved;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAG_CAL_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN, MAVLINK_MSG_ID_MAG_CAL_REPORT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#endif
}

/**
 * @brief Pack a mag_cal_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param compass_id Compass being calibrated
 * @param cal_mask Bitmask of compasses being calibrated
 * @param cal_status Status (see MAG_CAL_STATUS enum)
 * @param autosaved 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
 * @param fitness RMS milligauss residuals
 * @param ofs_x X offset
 * @param ofs_y Y offset
 * @param ofs_z Z offset
 * @param diag_x X diagonal (matrix 11)
 * @param diag_y Y diagonal (matrix 22)
 * @param diag_z Z diagonal (matrix 33)
 * @param offdiag_x X off-diagonal (matrix 12 and 21)
 * @param offdiag_y Y off-diagonal (matrix 13 and 31)
 * @param offdiag_z Z off-diagonal (matrix 32 and 23)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mag_cal_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t compass_id,uint8_t cal_mask,uint8_t cal_status,uint8_t autosaved,float fitness,float ofs_x,float ofs_y,float ofs_z,float diag_x,float diag_y,float diag_z,float offdiag_x,float offdiag_y,float offdiag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN];
	_mav_put_float(buf, 0, fitness);
	_mav_put_float(buf, 4, ofs_x);
	_mav_put_float(buf, 8, ofs_y);
	_mav_put_float(buf, 12, ofs_z);
	_mav_put_float(buf, 16, diag_x);
	_mav_put_float(buf, 20, diag_y);
	_mav_put_float(buf, 24, diag_z);
	_mav_put_float(buf, 28, offdiag_x);
	_mav_put_float(buf, 32, offdiag_y);
	_mav_put_float(buf, 36, offdiag_z);
	_mav_put_uint8_t(buf, 40, compass_id);
	_mav_put_uint8_t(buf, 41, cal_mask);
	_mav_put_uint8_t(buf, 42, cal_status);
	_mav_put_uint8_t(buf, 43, autosaved);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#else
	mavlink_mag_cal_report_t packet;
	packet.fitness = fitness;
	packet.ofs_x = ofs_x;
	packet.ofs_y = ofs_y;
	packet.ofs_z = ofs_z;
	packet.diag_x = diag_x;
	packet.diag_y = diag_y;
	packet.diag_z = diag_z;
	packet.offdiag_x = offdiag_x;
	packet.offdiag_y = offdiag_y;
	packet.offdiag_z = offdiag_z;
	packet.compass_id = compass_id;
	packet.cal_mask = cal_mask;
	packet.cal_status = cal_status;
	packet.autosaved = autosaved;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAG_CAL_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN, MAVLINK_MSG_ID_MAG_CAL_REPORT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#endif
}

/**
 * @brief Encode a mag_cal_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mag_cal_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mag_cal_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mag_cal_report_t* mag_cal_report)
{
	return mavlink_msg_mag_cal_report_pack(system_id, component_id, msg, mag_cal_report->compass_id, mag_cal_report->cal_mask, mag_cal_report->cal_status, mag_cal_report->autosaved, mag_cal_report->fitness, mag_cal_report->ofs_x, mag_cal_report->ofs_y, mag_cal_report->ofs_z, mag_cal_report->diag_x, mag_cal_report->diag_y, mag_cal_report->diag_z, mag_cal_report->offdiag_x, mag_cal_report->offdiag_y, mag_cal_report->offdiag_z);
}

/**
 * @brief Encode a mag_cal_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mag_cal_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mag_cal_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mag_cal_report_t* mag_cal_report)
{
	return mavlink_msg_mag_cal_report_pack_chan(system_id, component_id, chan, msg, mag_cal_report->compass_id, mag_cal_report->cal_mask, mag_cal_report->cal_status, mag_cal_report->autosaved, mag_cal_report->fitness, mag_cal_report->ofs_x, mag_cal_report->ofs_y, mag_cal_report->ofs_z, mag_cal_report->diag_x, mag_cal_report->diag_y, mag_cal_report->diag_z, mag_cal_report->offdiag_x, mag_cal_report->offdiag_y, mag_cal_report->offdiag_z);
}

/**
 * @brief Send a mag_cal_report message
 * @param chan MAVLink channel to send the message
 *
 * @param compass_id Compass being calibrated
 * @param cal_mask Bitmask of compasses being calibrated
 * @param cal_status Status (see MAG_CAL_STATUS enum)
 * @param autosaved 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
 * @param fitness RMS milligauss residuals
 * @param ofs_x X offset
 * @param ofs_y Y offset
 * @param ofs_z Z offset
 * @param diag_x X diagonal (matrix 11)
 * @param diag_y Y diagonal (matrix 22)
 * @param diag_z Z diagonal (matrix 33)
 * @param offdiag_x X off-diagonal (matrix 12 and 21)
 * @param offdiag_y Y off-diagonal (matrix 13 and 31)
 * @param offdiag_z Z off-diagonal (matrix 32 and 23)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mag_cal_report_send(mavlink_channel_t chan, uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN];
	_mav_put_float(buf, 0, fitness);
	_mav_put_float(buf, 4, ofs_x);
	_mav_put_float(buf, 8, ofs_y);
	_mav_put_float(buf, 12, ofs_z);
	_mav_put_float(buf, 16, diag_x);
	_mav_put_float(buf, 20, diag_y);
	_mav_put_float(buf, 24, diag_z);
	_mav_put_float(buf, 28, offdiag_x);
	_mav_put_float(buf, 32, offdiag_y);
	_mav_put_float(buf, 36, offdiag_z);
	_mav_put_uint8_t(buf, 40, compass_id);
	_mav_put_uint8_t(buf, 41, cal_mask);
	_mav_put_uint8_t(buf, 42, cal_status);
	_mav_put_uint8_t(buf, 43, autosaved);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_REPORT, buf, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN, MAVLINK_MSG_ID_MAG_CAL_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_REPORT, buf, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#endif
#else
	mavlink_mag_cal_report_t packet;
	packet.fitness = fitness;
	packet.ofs_x = ofs_x;
	packet.ofs_y = ofs_y;
	packet.ofs_z = ofs_z;
	packet.diag_x = diag_x;
	packet.diag_y = diag_y;
	packet.diag_z = diag_z;
	packet.offdiag_x = offdiag_x;
	packet.offdiag_y = offdiag_y;
	packet.offdiag_z = offdiag_z;
	packet.compass_id = compass_id;
	packet.cal_mask = cal_mask;
	packet.cal_status = cal_status;
	packet.autosaved = autosaved;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_REPORT, (const char *)&packet, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN, MAVLINK_MSG_ID_MAG_CAL_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_REPORT, (const char *)&packet, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mag_cal_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t compass_id, uint8_t cal_mask, uint8_t cal_status, uint8_t autosaved, float fitness, float ofs_x, float ofs_y, float ofs_z, float diag_x, float diag_y, float diag_z, float offdiag_x, float offdiag_y, float offdiag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, fitness);
	_mav_put_float(buf, 4, ofs_x);
	_mav_put_float(buf, 8, ofs_y);
	_mav_put_float(buf, 12, ofs_z);
	_mav_put_float(buf, 16, diag_x);
	_mav_put_float(buf, 20, diag_y);
	_mav_put_float(buf, 24, diag_z);
	_mav_put_float(buf, 28, offdiag_x);
	_mav_put_float(buf, 32, offdiag_y);
	_mav_put_float(buf, 36, offdiag_z);
	_mav_put_uint8_t(buf, 40, compass_id);
	_mav_put_uint8_t(buf, 41, cal_mask);
	_mav_put_uint8_t(buf, 42, cal_status);
	_mav_put_uint8_t(buf, 43, autosaved);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_REPORT, buf, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN, MAVLINK_MSG_ID_MAG_CAL_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_REPORT, buf, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#endif
#else
	mavlink_mag_cal_report_t *packet = (mavlink_mag_cal_report_t *)msgbuf;
	packet->fitness = fitness;
	packet->ofs_x = ofs_x;
	packet->ofs_y = ofs_y;
	packet->ofs_z = ofs_z;
	packet->diag_x = diag_x;
	packet->diag_y = diag_y;
	packet->diag_z = diag_z;
	packet->offdiag_x = offdiag_x;
	packet->offdiag_y = offdiag_y;
	packet->offdiag_z = offdiag_z;
	packet->compass_id = compass_id;
	packet->cal_mask = cal_mask;
	packet->cal_status = cal_status;
	packet->autosaved = autosaved;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_REPORT, (const char *)packet, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN, MAVLINK_MSG_ID_MAG_CAL_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAG_CAL_REPORT, (const char *)packet, MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MAG_CAL_REPORT UNPACKING


/**
 * @brief Get field compass_id from mag_cal_report message
 *
 * @return Compass being calibrated
 */
static inline uint8_t mavlink_msg_mag_cal_report_get_compass_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field cal_mask from mag_cal_report message
 *
 * @return Bitmask of compasses being calibrated
 */
static inline uint8_t mavlink_msg_mag_cal_report_get_cal_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field cal_status from mag_cal_report message
 *
 * @return Status (see MAG_CAL_STATUS enum)
 */
static inline uint8_t mavlink_msg_mag_cal_report_get_cal_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field autosaved from mag_cal_report message
 *
 * @return 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
 */
static inline uint8_t mavlink_msg_mag_cal_report_get_autosaved(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field fitness from mag_cal_report message
 *
 * @return RMS milligauss residuals
 */
static inline float mavlink_msg_mag_cal_report_get_fitness(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ofs_x from mag_cal_report message
 *
 * @return X offset
 */
static inline float mavlink_msg_mag_cal_report_get_ofs_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field ofs_y from mag_cal_report message
 *
 * @return Y offset
 */
static inline float mavlink_msg_mag_cal_report_get_ofs_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field ofs_z from mag_cal_report message
 *
 * @return Z offset
 */
static inline float mavlink_msg_mag_cal_report_get_ofs_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field diag_x from mag_cal_report message
 *
 * @return X diagonal (matrix 11)
 */
static inline float mavlink_msg_mag_cal_report_get_diag_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field diag_y from mag_cal_report message
 *
 * @return Y diagonal (matrix 22)
 */
static inline float mavlink_msg_mag_cal_report_get_diag_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field diag_z from mag_cal_report message
 *
 * @return Z diagonal (matrix 33)
 */
static inline float mavlink_msg_mag_cal_report_get_diag_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field offdiag_x from mag_cal_report message
 *
 * @return X off-diagonal (matrix 12 and 21)
 */
static inline float mavlink_msg_mag_cal_report_get_offdiag_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field offdiag_y from mag_cal_report message
 *
 * @return Y off-diagonal (matrix 13 and 31)
 */
static inline float mavlink_msg_mag_cal_report_get_offdiag_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field offdiag_z from mag_cal_report message
 *
 * @return Z off-diagonal (matrix 32 and 23)
 */
static inline float mavlink_msg_mag_cal_report_get_offdiag_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a mag_cal_report message into a struct
 *
 * @param msg The message to decode
 * @param mag_cal_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_mag_cal_report_decode(const mavlink_message_t* msg, mavlink_mag_cal_report_t* mag_cal_report)
{
#if MAVLINK_NEED_BYTE_SWAP
	mag_cal_report->fitness = mavlink_msg_mag_cal_report_get_fitness(msg);
	mag_cal_report->ofs_x = mavlink_msg_mag_cal_report_get_ofs_x(msg);
	mag_cal_report->ofs_y = mavlink_msg_mag_cal_report_get_ofs_y(msg);
	mag_cal_report->ofs_z = mavlink_msg_mag_cal_report_get_ofs_z(msg);
	mag_cal_report->diag_x = mavlink_msg_mag_cal_report_get_diag_x(msg);
	mag_cal_report->diag_y = mavlink_msg_mag_cal_report_get_diag_y(msg);
	mag_cal_report->diag_z = mavlink_msg_mag_cal_report_get_diag_z(msg);
	mag_cal_report->offdiag_x = mavlink_msg_mag_cal_report_get_offdiag_x(msg);
	mag_cal_report->offdiag_y = mavlink_msg_mag_cal_report_get_offdiag_y(msg);
	mag_cal_report->offdiag_z = mavlink_msg_mag_cal_report_get_offdiag_z(msg);
	mag_cal_report->compass_id = mavlink_msg_mag_cal_report_get_compass_id(msg);
	mag_cal_report->cal_mask = mavlink_msg_mag_cal_report_get_cal_mask(msg);
	mag_cal_report->cal_status = mavlink_msg_mag_cal_report_get_cal_status(msg);
	mag_cal_report->autosaved = mavlink_msg_mag_cal_report_get_autosaved(msg);
#else
	memcpy(mag_cal_report, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MAG_CAL_REPORT_LEN);
#endif
}

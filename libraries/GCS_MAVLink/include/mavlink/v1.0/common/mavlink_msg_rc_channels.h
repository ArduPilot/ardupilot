// MESSAGE RC_CHANNELS PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS 65

typedef struct __mavlink_rc_channels_t
{
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 uint16_t chan1_raw; /*< RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan2_raw; /*< RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan3_raw; /*< RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan4_raw; /*< RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan5_raw; /*< RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan6_raw; /*< RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan7_raw; /*< RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan8_raw; /*< RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan9_raw; /*< RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan10_raw; /*< RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan11_raw; /*< RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan12_raw; /*< RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan13_raw; /*< RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan14_raw; /*< RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan15_raw; /*< RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan16_raw; /*< RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan17_raw; /*< RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan18_raw; /*< RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint8_t chancount; /*< Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.*/
 uint8_t rssi; /*< Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.*/
} mavlink_rc_channels_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_LEN 42
#define MAVLINK_MSG_ID_65_LEN 42

#define MAVLINK_MSG_ID_RC_CHANNELS_CRC 118
#define MAVLINK_MSG_ID_65_CRC 118



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS { \
	"RC_CHANNELS", \
	21, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_rc_channels_t, time_boot_ms) }, \
         { "chan1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_rc_channels_t, chan1_raw) }, \
         { "chan2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_rc_channels_t, chan2_raw) }, \
         { "chan3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_rc_channels_t, chan3_raw) }, \
         { "chan4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_rc_channels_t, chan4_raw) }, \
         { "chan5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_rc_channels_t, chan5_raw) }, \
         { "chan6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_rc_channels_t, chan6_raw) }, \
         { "chan7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_rc_channels_t, chan7_raw) }, \
         { "chan8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_rc_channels_t, chan8_raw) }, \
         { "chan9_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_rc_channels_t, chan9_raw) }, \
         { "chan10_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_rc_channels_t, chan10_raw) }, \
         { "chan11_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_rc_channels_t, chan11_raw) }, \
         { "chan12_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_rc_channels_t, chan12_raw) }, \
         { "chan13_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_rc_channels_t, chan13_raw) }, \
         { "chan14_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_rc_channels_t, chan14_raw) }, \
         { "chan15_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_rc_channels_t, chan15_raw) }, \
         { "chan16_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_rc_channels_t, chan16_raw) }, \
         { "chan17_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_rc_channels_t, chan17_raw) }, \
         { "chan18_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_rc_channels_t, chan18_raw) }, \
         { "chancount", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_rc_channels_t, chancount) }, \
         { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_rc_channels_t, rssi) }, \
         } \
}


/**
 * @brief Pack a rc_channels message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param chancount Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan9_raw RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan10_raw RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan11_raw RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan12_raw RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan13_raw RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan14_raw RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan15_raw RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan16_raw RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan17_raw RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan18_raw RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param rssi Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, uint8_t chancount, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw, uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint16_t(buf, 4, chan1_raw);
	_mav_put_uint16_t(buf, 6, chan2_raw);
	_mav_put_uint16_t(buf, 8, chan3_raw);
	_mav_put_uint16_t(buf, 10, chan4_raw);
	_mav_put_uint16_t(buf, 12, chan5_raw);
	_mav_put_uint16_t(buf, 14, chan6_raw);
	_mav_put_uint16_t(buf, 16, chan7_raw);
	_mav_put_uint16_t(buf, 18, chan8_raw);
	_mav_put_uint16_t(buf, 20, chan9_raw);
	_mav_put_uint16_t(buf, 22, chan10_raw);
	_mav_put_uint16_t(buf, 24, chan11_raw);
	_mav_put_uint16_t(buf, 26, chan12_raw);
	_mav_put_uint16_t(buf, 28, chan13_raw);
	_mav_put_uint16_t(buf, 30, chan14_raw);
	_mav_put_uint16_t(buf, 32, chan15_raw);
	_mav_put_uint16_t(buf, 34, chan16_raw);
	_mav_put_uint16_t(buf, 36, chan17_raw);
	_mav_put_uint16_t(buf, 38, chan18_raw);
	_mav_put_uint8_t(buf, 40, chancount);
	_mav_put_uint8_t(buf, 41, rssi);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#else
	mavlink_rc_channels_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.chan9_raw = chan9_raw;
	packet.chan10_raw = chan10_raw;
	packet.chan11_raw = chan11_raw;
	packet.chan12_raw = chan12_raw;
	packet.chan13_raw = chan13_raw;
	packet.chan14_raw = chan14_raw;
	packet.chan15_raw = chan15_raw;
	packet.chan16_raw = chan16_raw;
	packet.chan17_raw = chan17_raw;
	packet.chan18_raw = chan18_raw;
	packet.chancount = chancount;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif
}

/**
 * @brief Pack a rc_channels message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param chancount Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan9_raw RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan10_raw RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan11_raw RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan12_raw RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan13_raw RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan14_raw RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan15_raw RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan16_raw RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan17_raw RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan18_raw RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param rssi Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,uint8_t chancount,uint16_t chan1_raw,uint16_t chan2_raw,uint16_t chan3_raw,uint16_t chan4_raw,uint16_t chan5_raw,uint16_t chan6_raw,uint16_t chan7_raw,uint16_t chan8_raw,uint16_t chan9_raw,uint16_t chan10_raw,uint16_t chan11_raw,uint16_t chan12_raw,uint16_t chan13_raw,uint16_t chan14_raw,uint16_t chan15_raw,uint16_t chan16_raw,uint16_t chan17_raw,uint16_t chan18_raw,uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint16_t(buf, 4, chan1_raw);
	_mav_put_uint16_t(buf, 6, chan2_raw);
	_mav_put_uint16_t(buf, 8, chan3_raw);
	_mav_put_uint16_t(buf, 10, chan4_raw);
	_mav_put_uint16_t(buf, 12, chan5_raw);
	_mav_put_uint16_t(buf, 14, chan6_raw);
	_mav_put_uint16_t(buf, 16, chan7_raw);
	_mav_put_uint16_t(buf, 18, chan8_raw);
	_mav_put_uint16_t(buf, 20, chan9_raw);
	_mav_put_uint16_t(buf, 22, chan10_raw);
	_mav_put_uint16_t(buf, 24, chan11_raw);
	_mav_put_uint16_t(buf, 26, chan12_raw);
	_mav_put_uint16_t(buf, 28, chan13_raw);
	_mav_put_uint16_t(buf, 30, chan14_raw);
	_mav_put_uint16_t(buf, 32, chan15_raw);
	_mav_put_uint16_t(buf, 34, chan16_raw);
	_mav_put_uint16_t(buf, 36, chan17_raw);
	_mav_put_uint16_t(buf, 38, chan18_raw);
	_mav_put_uint8_t(buf, 40, chancount);
	_mav_put_uint8_t(buf, 41, rssi);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#else
	mavlink_rc_channels_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.chan9_raw = chan9_raw;
	packet.chan10_raw = chan10_raw;
	packet.chan11_raw = chan11_raw;
	packet.chan12_raw = chan12_raw;
	packet.chan13_raw = chan13_raw;
	packet.chan14_raw = chan14_raw;
	packet.chan15_raw = chan15_raw;
	packet.chan16_raw = chan16_raw;
	packet.chan17_raw = chan17_raw;
	packet.chan18_raw = chan18_raw;
	packet.chancount = chancount;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif
}

/**
 * @brief Encode a rc_channels struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_t* rc_channels)
{
	return mavlink_msg_rc_channels_pack(system_id, component_id, msg, rc_channels->time_boot_ms, rc_channels->chancount, rc_channels->chan1_raw, rc_channels->chan2_raw, rc_channels->chan3_raw, rc_channels->chan4_raw, rc_channels->chan5_raw, rc_channels->chan6_raw, rc_channels->chan7_raw, rc_channels->chan8_raw, rc_channels->chan9_raw, rc_channels->chan10_raw, rc_channels->chan11_raw, rc_channels->chan12_raw, rc_channels->chan13_raw, rc_channels->chan14_raw, rc_channels->chan15_raw, rc_channels->chan16_raw, rc_channels->chan17_raw, rc_channels->chan18_raw, rc_channels->rssi);
}

/**
 * @brief Encode a rc_channels struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rc_channels_t* rc_channels)
{
	return mavlink_msg_rc_channels_pack_chan(system_id, component_id, chan, msg, rc_channels->time_boot_ms, rc_channels->chancount, rc_channels->chan1_raw, rc_channels->chan2_raw, rc_channels->chan3_raw, rc_channels->chan4_raw, rc_channels->chan5_raw, rc_channels->chan6_raw, rc_channels->chan7_raw, rc_channels->chan8_raw, rc_channels->chan9_raw, rc_channels->chan10_raw, rc_channels->chan11_raw, rc_channels->chan12_raw, rc_channels->chan13_raw, rc_channels->chan14_raw, rc_channels->chan15_raw, rc_channels->chan16_raw, rc_channels->chan17_raw, rc_channels->chan18_raw, rc_channels->rssi);
}

/**
 * @brief Send a rc_channels message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param chancount Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan9_raw RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan10_raw RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan11_raw RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan12_raw RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan13_raw RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan14_raw RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan15_raw RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan16_raw RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan17_raw RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan18_raw RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param rssi Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t chancount, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw, uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_RC_CHANNELS_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint16_t(buf, 4, chan1_raw);
	_mav_put_uint16_t(buf, 6, chan2_raw);
	_mav_put_uint16_t(buf, 8, chan3_raw);
	_mav_put_uint16_t(buf, 10, chan4_raw);
	_mav_put_uint16_t(buf, 12, chan5_raw);
	_mav_put_uint16_t(buf, 14, chan6_raw);
	_mav_put_uint16_t(buf, 16, chan7_raw);
	_mav_put_uint16_t(buf, 18, chan8_raw);
	_mav_put_uint16_t(buf, 20, chan9_raw);
	_mav_put_uint16_t(buf, 22, chan10_raw);
	_mav_put_uint16_t(buf, 24, chan11_raw);
	_mav_put_uint16_t(buf, 26, chan12_raw);
	_mav_put_uint16_t(buf, 28, chan13_raw);
	_mav_put_uint16_t(buf, 30, chan14_raw);
	_mav_put_uint16_t(buf, 32, chan15_raw);
	_mav_put_uint16_t(buf, 34, chan16_raw);
	_mav_put_uint16_t(buf, 36, chan17_raw);
	_mav_put_uint16_t(buf, 38, chan18_raw);
	_mav_put_uint8_t(buf, 40, chancount);
	_mav_put_uint8_t(buf, 41, rssi);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, buf, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, buf, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif
#else
	mavlink_rc_channels_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.chan9_raw = chan9_raw;
	packet.chan10_raw = chan10_raw;
	packet.chan11_raw = chan11_raw;
	packet.chan12_raw = chan12_raw;
	packet.chan13_raw = chan13_raw;
	packet.chan14_raw = chan14_raw;
	packet.chan15_raw = chan15_raw;
	packet.chan16_raw = chan16_raw;
	packet.chan17_raw = chan17_raw;
	packet.chan18_raw = chan18_raw;
	packet.chancount = chancount;
	packet.rssi = rssi;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, (const char *)&packet, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, (const char *)&packet, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_RC_CHANNELS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rc_channels_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t chancount, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint16_t chan13_raw, uint16_t chan14_raw, uint16_t chan15_raw, uint16_t chan16_raw, uint16_t chan17_raw, uint16_t chan18_raw, uint8_t rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_uint16_t(buf, 4, chan1_raw);
	_mav_put_uint16_t(buf, 6, chan2_raw);
	_mav_put_uint16_t(buf, 8, chan3_raw);
	_mav_put_uint16_t(buf, 10, chan4_raw);
	_mav_put_uint16_t(buf, 12, chan5_raw);
	_mav_put_uint16_t(buf, 14, chan6_raw);
	_mav_put_uint16_t(buf, 16, chan7_raw);
	_mav_put_uint16_t(buf, 18, chan8_raw);
	_mav_put_uint16_t(buf, 20, chan9_raw);
	_mav_put_uint16_t(buf, 22, chan10_raw);
	_mav_put_uint16_t(buf, 24, chan11_raw);
	_mav_put_uint16_t(buf, 26, chan12_raw);
	_mav_put_uint16_t(buf, 28, chan13_raw);
	_mav_put_uint16_t(buf, 30, chan14_raw);
	_mav_put_uint16_t(buf, 32, chan15_raw);
	_mav_put_uint16_t(buf, 34, chan16_raw);
	_mav_put_uint16_t(buf, 36, chan17_raw);
	_mav_put_uint16_t(buf, 38, chan18_raw);
	_mav_put_uint8_t(buf, 40, chancount);
	_mav_put_uint8_t(buf, 41, rssi);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, buf, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, buf, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif
#else
	mavlink_rc_channels_t *packet = (mavlink_rc_channels_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->chan1_raw = chan1_raw;
	packet->chan2_raw = chan2_raw;
	packet->chan3_raw = chan3_raw;
	packet->chan4_raw = chan4_raw;
	packet->chan5_raw = chan5_raw;
	packet->chan6_raw = chan6_raw;
	packet->chan7_raw = chan7_raw;
	packet->chan8_raw = chan8_raw;
	packet->chan9_raw = chan9_raw;
	packet->chan10_raw = chan10_raw;
	packet->chan11_raw = chan11_raw;
	packet->chan12_raw = chan12_raw;
	packet->chan13_raw = chan13_raw;
	packet->chan14_raw = chan14_raw;
	packet->chan15_raw = chan15_raw;
	packet->chan16_raw = chan16_raw;
	packet->chan17_raw = chan17_raw;
	packet->chan18_raw = chan18_raw;
	packet->chancount = chancount;
	packet->rssi = rssi;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, (const char *)packet, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS, (const char *)packet, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE RC_CHANNELS UNPACKING


/**
 * @brief Get field time_boot_ms from rc_channels message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_rc_channels_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field chancount from rc_channels message
 *
 * @return Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
 */
static inline uint8_t mavlink_msg_rc_channels_get_chancount(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field chan1_raw from rc_channels message
 *
 * @return RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan1_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field chan2_raw from rc_channels message
 *
 * @return RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan2_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field chan3_raw from rc_channels message
 *
 * @return RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan3_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field chan4_raw from rc_channels message
 *
 * @return RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan4_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field chan5_raw from rc_channels message
 *
 * @return RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan5_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field chan6_raw from rc_channels message
 *
 * @return RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan6_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field chan7_raw from rc_channels message
 *
 * @return RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan7_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field chan8_raw from rc_channels message
 *
 * @return RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan8_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field chan9_raw from rc_channels message
 *
 * @return RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan9_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field chan10_raw from rc_channels message
 *
 * @return RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan10_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field chan11_raw from rc_channels message
 *
 * @return RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan11_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field chan12_raw from rc_channels message
 *
 * @return RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan12_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field chan13_raw from rc_channels message
 *
 * @return RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan13_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field chan14_raw from rc_channels message
 *
 * @return RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan14_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field chan15_raw from rc_channels message
 *
 * @return RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan15_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field chan16_raw from rc_channels message
 *
 * @return RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan16_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field chan17_raw from rc_channels message
 *
 * @return RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan17_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field chan18_raw from rc_channels message
 *
 * @return RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 */
static inline uint16_t mavlink_msg_rc_channels_get_chan18_raw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  38);
}

/**
 * @brief Get field rssi from rc_channels message
 *
 * @return Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 */
static inline uint8_t mavlink_msg_rc_channels_get_rssi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Decode a rc_channels message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_decode(const mavlink_message_t* msg, mavlink_rc_channels_t* rc_channels)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_channels->time_boot_ms = mavlink_msg_rc_channels_get_time_boot_ms(msg);
	rc_channels->chan1_raw = mavlink_msg_rc_channels_get_chan1_raw(msg);
	rc_channels->chan2_raw = mavlink_msg_rc_channels_get_chan2_raw(msg);
	rc_channels->chan3_raw = mavlink_msg_rc_channels_get_chan3_raw(msg);
	rc_channels->chan4_raw = mavlink_msg_rc_channels_get_chan4_raw(msg);
	rc_channels->chan5_raw = mavlink_msg_rc_channels_get_chan5_raw(msg);
	rc_channels->chan6_raw = mavlink_msg_rc_channels_get_chan6_raw(msg);
	rc_channels->chan7_raw = mavlink_msg_rc_channels_get_chan7_raw(msg);
	rc_channels->chan8_raw = mavlink_msg_rc_channels_get_chan8_raw(msg);
	rc_channels->chan9_raw = mavlink_msg_rc_channels_get_chan9_raw(msg);
	rc_channels->chan10_raw = mavlink_msg_rc_channels_get_chan10_raw(msg);
	rc_channels->chan11_raw = mavlink_msg_rc_channels_get_chan11_raw(msg);
	rc_channels->chan12_raw = mavlink_msg_rc_channels_get_chan12_raw(msg);
	rc_channels->chan13_raw = mavlink_msg_rc_channels_get_chan13_raw(msg);
	rc_channels->chan14_raw = mavlink_msg_rc_channels_get_chan14_raw(msg);
	rc_channels->chan15_raw = mavlink_msg_rc_channels_get_chan15_raw(msg);
	rc_channels->chan16_raw = mavlink_msg_rc_channels_get_chan16_raw(msg);
	rc_channels->chan17_raw = mavlink_msg_rc_channels_get_chan17_raw(msg);
	rc_channels->chan18_raw = mavlink_msg_rc_channels_get_chan18_raw(msg);
	rc_channels->chancount = mavlink_msg_rc_channels_get_chancount(msg);
	rc_channels->rssi = mavlink_msg_rc_channels_get_rssi(msg);
#else
	memcpy(rc_channels, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_RC_CHANNELS_LEN);
#endif
}

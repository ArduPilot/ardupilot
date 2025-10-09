#pragma once
// MESSAGE HERELINK_TELEM PACKING

#define MAVLINK_MSG_ID_HERELINK_TELEM 50003


typedef struct __mavlink_herelink_telem_t {
 uint32_t rf_freq; /*<  */
 uint32_t link_bw; /*<  */
 uint32_t link_rate; /*<  */
 int16_t snr; /*<  */
 int16_t cpu_temp; /*<  */
 int16_t board_temp; /*<  */
 uint8_t rssi; /*<  */
} mavlink_herelink_telem_t;

#define MAVLINK_MSG_ID_HERELINK_TELEM_LEN 19
#define MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN 19
#define MAVLINK_MSG_ID_50003_LEN 19
#define MAVLINK_MSG_ID_50003_MIN_LEN 19

#define MAVLINK_MSG_ID_HERELINK_TELEM_CRC 62
#define MAVLINK_MSG_ID_50003_CRC 62



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HERELINK_TELEM { \
    50003, \
    "HERELINK_TELEM", \
    7, \
    {  { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_herelink_telem_t, rssi) }, \
         { "snr", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_herelink_telem_t, snr) }, \
         { "rf_freq", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_herelink_telem_t, rf_freq) }, \
         { "link_bw", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_herelink_telem_t, link_bw) }, \
         { "link_rate", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_herelink_telem_t, link_rate) }, \
         { "cpu_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_herelink_telem_t, cpu_temp) }, \
         { "board_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_herelink_telem_t, board_temp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HERELINK_TELEM { \
    "HERELINK_TELEM", \
    7, \
    {  { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_herelink_telem_t, rssi) }, \
         { "snr", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_herelink_telem_t, snr) }, \
         { "rf_freq", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_herelink_telem_t, rf_freq) }, \
         { "link_bw", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_herelink_telem_t, link_bw) }, \
         { "link_rate", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_herelink_telem_t, link_rate) }, \
         { "cpu_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_herelink_telem_t, cpu_temp) }, \
         { "board_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_herelink_telem_t, board_temp) }, \
         } \
}
#endif

/**
 * @brief Pack a herelink_telem message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rssi  
 * @param snr  
 * @param rf_freq  
 * @param link_bw  
 * @param link_rate  
 * @param cpu_temp  
 * @param board_temp  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_herelink_telem_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t rssi, int16_t snr, uint32_t rf_freq, uint32_t link_bw, uint32_t link_rate, int16_t cpu_temp, int16_t board_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HERELINK_TELEM_LEN];
    _mav_put_uint32_t(buf, 0, rf_freq);
    _mav_put_uint32_t(buf, 4, link_bw);
    _mav_put_uint32_t(buf, 8, link_rate);
    _mav_put_int16_t(buf, 12, snr);
    _mav_put_int16_t(buf, 14, cpu_temp);
    _mav_put_int16_t(buf, 16, board_temp);
    _mav_put_uint8_t(buf, 18, rssi);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HERELINK_TELEM_LEN);
#else
    mavlink_herelink_telem_t packet;
    packet.rf_freq = rf_freq;
    packet.link_bw = link_bw;
    packet.link_rate = link_rate;
    packet.snr = snr;
    packet.cpu_temp = cpu_temp;
    packet.board_temp = board_temp;
    packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HERELINK_TELEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HERELINK_TELEM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_CRC);
}

/**
 * @brief Pack a herelink_telem message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param rssi  
 * @param snr  
 * @param rf_freq  
 * @param link_bw  
 * @param link_rate  
 * @param cpu_temp  
 * @param board_temp  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_herelink_telem_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t rssi, int16_t snr, uint32_t rf_freq, uint32_t link_bw, uint32_t link_rate, int16_t cpu_temp, int16_t board_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HERELINK_TELEM_LEN];
    _mav_put_uint32_t(buf, 0, rf_freq);
    _mav_put_uint32_t(buf, 4, link_bw);
    _mav_put_uint32_t(buf, 8, link_rate);
    _mav_put_int16_t(buf, 12, snr);
    _mav_put_int16_t(buf, 14, cpu_temp);
    _mav_put_int16_t(buf, 16, board_temp);
    _mav_put_uint8_t(buf, 18, rssi);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HERELINK_TELEM_LEN);
#else
    mavlink_herelink_telem_t packet;
    packet.rf_freq = rf_freq;
    packet.link_bw = link_bw;
    packet.link_rate = link_rate;
    packet.snr = snr;
    packet.cpu_temp = cpu_temp;
    packet.board_temp = board_temp;
    packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HERELINK_TELEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HERELINK_TELEM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_LEN);
#endif
}

/**
 * @brief Pack a herelink_telem message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rssi  
 * @param snr  
 * @param rf_freq  
 * @param link_bw  
 * @param link_rate  
 * @param cpu_temp  
 * @param board_temp  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_herelink_telem_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t rssi,int16_t snr,uint32_t rf_freq,uint32_t link_bw,uint32_t link_rate,int16_t cpu_temp,int16_t board_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HERELINK_TELEM_LEN];
    _mav_put_uint32_t(buf, 0, rf_freq);
    _mav_put_uint32_t(buf, 4, link_bw);
    _mav_put_uint32_t(buf, 8, link_rate);
    _mav_put_int16_t(buf, 12, snr);
    _mav_put_int16_t(buf, 14, cpu_temp);
    _mav_put_int16_t(buf, 16, board_temp);
    _mav_put_uint8_t(buf, 18, rssi);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HERELINK_TELEM_LEN);
#else
    mavlink_herelink_telem_t packet;
    packet.rf_freq = rf_freq;
    packet.link_bw = link_bw;
    packet.link_rate = link_rate;
    packet.snr = snr;
    packet.cpu_temp = cpu_temp;
    packet.board_temp = board_temp;
    packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HERELINK_TELEM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HERELINK_TELEM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_CRC);
}

/**
 * @brief Encode a herelink_telem struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param herelink_telem C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_herelink_telem_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_herelink_telem_t* herelink_telem)
{
    return mavlink_msg_herelink_telem_pack(system_id, component_id, msg, herelink_telem->rssi, herelink_telem->snr, herelink_telem->rf_freq, herelink_telem->link_bw, herelink_telem->link_rate, herelink_telem->cpu_temp, herelink_telem->board_temp);
}

/**
 * @brief Encode a herelink_telem struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param herelink_telem C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_herelink_telem_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_herelink_telem_t* herelink_telem)
{
    return mavlink_msg_herelink_telem_pack_chan(system_id, component_id, chan, msg, herelink_telem->rssi, herelink_telem->snr, herelink_telem->rf_freq, herelink_telem->link_bw, herelink_telem->link_rate, herelink_telem->cpu_temp, herelink_telem->board_temp);
}

/**
 * @brief Encode a herelink_telem struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param herelink_telem C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_herelink_telem_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_herelink_telem_t* herelink_telem)
{
    return mavlink_msg_herelink_telem_pack_status(system_id, component_id, _status, msg,  herelink_telem->rssi, herelink_telem->snr, herelink_telem->rf_freq, herelink_telem->link_bw, herelink_telem->link_rate, herelink_telem->cpu_temp, herelink_telem->board_temp);
}

/**
 * @brief Send a herelink_telem message
 * @param chan MAVLink channel to send the message
 *
 * @param rssi  
 * @param snr  
 * @param rf_freq  
 * @param link_bw  
 * @param link_rate  
 * @param cpu_temp  
 * @param board_temp  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_herelink_telem_send(mavlink_channel_t chan, uint8_t rssi, int16_t snr, uint32_t rf_freq, uint32_t link_bw, uint32_t link_rate, int16_t cpu_temp, int16_t board_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HERELINK_TELEM_LEN];
    _mav_put_uint32_t(buf, 0, rf_freq);
    _mav_put_uint32_t(buf, 4, link_bw);
    _mav_put_uint32_t(buf, 8, link_rate);
    _mav_put_int16_t(buf, 12, snr);
    _mav_put_int16_t(buf, 14, cpu_temp);
    _mav_put_int16_t(buf, 16, board_temp);
    _mav_put_uint8_t(buf, 18, rssi);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HERELINK_TELEM, buf, MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_CRC);
#else
    mavlink_herelink_telem_t packet;
    packet.rf_freq = rf_freq;
    packet.link_bw = link_bw;
    packet.link_rate = link_rate;
    packet.snr = snr;
    packet.cpu_temp = cpu_temp;
    packet.board_temp = board_temp;
    packet.rssi = rssi;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HERELINK_TELEM, (const char *)&packet, MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_CRC);
#endif
}

/**
 * @brief Send a herelink_telem message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_herelink_telem_send_struct(mavlink_channel_t chan, const mavlink_herelink_telem_t* herelink_telem)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_herelink_telem_send(chan, herelink_telem->rssi, herelink_telem->snr, herelink_telem->rf_freq, herelink_telem->link_bw, herelink_telem->link_rate, herelink_telem->cpu_temp, herelink_telem->board_temp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HERELINK_TELEM, (const char *)herelink_telem, MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_CRC);
#endif
}

#if MAVLINK_MSG_ID_HERELINK_TELEM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_herelink_telem_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t rssi, int16_t snr, uint32_t rf_freq, uint32_t link_bw, uint32_t link_rate, int16_t cpu_temp, int16_t board_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, rf_freq);
    _mav_put_uint32_t(buf, 4, link_bw);
    _mav_put_uint32_t(buf, 8, link_rate);
    _mav_put_int16_t(buf, 12, snr);
    _mav_put_int16_t(buf, 14, cpu_temp);
    _mav_put_int16_t(buf, 16, board_temp);
    _mav_put_uint8_t(buf, 18, rssi);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HERELINK_TELEM, buf, MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_CRC);
#else
    mavlink_herelink_telem_t *packet = (mavlink_herelink_telem_t *)msgbuf;
    packet->rf_freq = rf_freq;
    packet->link_bw = link_bw;
    packet->link_rate = link_rate;
    packet->snr = snr;
    packet->cpu_temp = cpu_temp;
    packet->board_temp = board_temp;
    packet->rssi = rssi;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HERELINK_TELEM, (const char *)packet, MAVLINK_MSG_ID_HERELINK_TELEM_MIN_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_LEN, MAVLINK_MSG_ID_HERELINK_TELEM_CRC);
#endif
}
#endif

#endif

// MESSAGE HERELINK_TELEM UNPACKING


/**
 * @brief Get field rssi from herelink_telem message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_herelink_telem_get_rssi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field snr from herelink_telem message
 *
 * @return  
 */
static inline int16_t mavlink_msg_herelink_telem_get_snr(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field rf_freq from herelink_telem message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_herelink_telem_get_rf_freq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field link_bw from herelink_telem message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_herelink_telem_get_link_bw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field link_rate from herelink_telem message
 *
 * @return  
 */
static inline uint32_t mavlink_msg_herelink_telem_get_link_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field cpu_temp from herelink_telem message
 *
 * @return  
 */
static inline int16_t mavlink_msg_herelink_telem_get_cpu_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field board_temp from herelink_telem message
 *
 * @return  
 */
static inline int16_t mavlink_msg_herelink_telem_get_board_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Decode a herelink_telem message into a struct
 *
 * @param msg The message to decode
 * @param herelink_telem C-struct to decode the message contents into
 */
static inline void mavlink_msg_herelink_telem_decode(const mavlink_message_t* msg, mavlink_herelink_telem_t* herelink_telem)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    herelink_telem->rf_freq = mavlink_msg_herelink_telem_get_rf_freq(msg);
    herelink_telem->link_bw = mavlink_msg_herelink_telem_get_link_bw(msg);
    herelink_telem->link_rate = mavlink_msg_herelink_telem_get_link_rate(msg);
    herelink_telem->snr = mavlink_msg_herelink_telem_get_snr(msg);
    herelink_telem->cpu_temp = mavlink_msg_herelink_telem_get_cpu_temp(msg);
    herelink_telem->board_temp = mavlink_msg_herelink_telem_get_board_temp(msg);
    herelink_telem->rssi = mavlink_msg_herelink_telem_get_rssi(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HERELINK_TELEM_LEN? msg->len : MAVLINK_MSG_ID_HERELINK_TELEM_LEN;
        memset(herelink_telem, 0, MAVLINK_MSG_ID_HERELINK_TELEM_LEN);
    memcpy(herelink_telem, _MAV_PAYLOAD(msg), len);
#endif
}

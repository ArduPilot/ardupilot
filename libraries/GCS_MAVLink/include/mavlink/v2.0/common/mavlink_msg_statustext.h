#pragma once
// MESSAGE STATUSTEXT PACKING

#define MAVLINK_MSG_ID_STATUSTEXT 253

MAVPACKED(
typedef struct __mavlink_statustext_t {
 uint8_t severity; /*<  Severity of status. Relies on the definitions within RFC-5424.*/
 char text[50]; /*<  Status text message, without null termination character*/
 uint16_t id; /*<  Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence and the message can be emitted immediately.*/
 uint8_t chunk_seq; /*<  This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to mean this was the last chunk.*/
}) mavlink_statustext_t;

#define MAVLINK_MSG_ID_STATUSTEXT_LEN 54
#define MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN 51
#define MAVLINK_MSG_ID_253_LEN 54
#define MAVLINK_MSG_ID_253_MIN_LEN 51

#define MAVLINK_MSG_ID_STATUSTEXT_CRC 83
#define MAVLINK_MSG_ID_253_CRC 83

#define MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STATUSTEXT { \
    253, \
    "STATUSTEXT", \
    4, \
    {  { "severity", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_statustext_t, severity) }, \
         { "text", NULL, MAVLINK_TYPE_CHAR, 50, 1, offsetof(mavlink_statustext_t, text) }, \
         { "id", NULL, MAVLINK_TYPE_UINT16_T, 0, 51, offsetof(mavlink_statustext_t, id) }, \
         { "chunk_seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_statustext_t, chunk_seq) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STATUSTEXT { \
    "STATUSTEXT", \
    4, \
    {  { "severity", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_statustext_t, severity) }, \
         { "text", NULL, MAVLINK_TYPE_CHAR, 50, 1, offsetof(mavlink_statustext_t, text) }, \
         { "id", NULL, MAVLINK_TYPE_UINT16_T, 0, 51, offsetof(mavlink_statustext_t, id) }, \
         { "chunk_seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_statustext_t, chunk_seq) }, \
         } \
}
#endif

/**
 * @brief Pack a statustext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param severity  Severity of status. Relies on the definitions within RFC-5424.
 * @param text  Status text message, without null termination character
 * @param id  Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence and the message can be emitted immediately.
 * @param chunk_seq  This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to mean this was the last chunk.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t severity, const char *text, uint16_t id, uint8_t chunk_seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];
    _mav_put_uint8_t(buf, 0, severity);
    _mav_put_uint16_t(buf, 51, id);
    _mav_put_uint8_t(buf, 53, chunk_seq);
    _mav_put_char_array(buf, 1, text, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#else
    mavlink_statustext_t packet;
    packet.severity = severity;
    packet.id = id;
    packet.chunk_seq = chunk_seq;
    mav_array_memcpy(packet.text, text, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
}

/**
 * @brief Pack a statustext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param severity  Severity of status. Relies on the definitions within RFC-5424.
 * @param text  Status text message, without null termination character
 * @param id  Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence and the message can be emitted immediately.
 * @param chunk_seq  This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to mean this was the last chunk.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t severity, const char *text, uint16_t id, uint8_t chunk_seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];
    _mav_put_uint8_t(buf, 0, severity);
    _mav_put_uint16_t(buf, 51, id);
    _mav_put_uint8_t(buf, 53, chunk_seq);
    _mav_put_char_array(buf, 1, text, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#else
    mavlink_statustext_t packet;
    packet.severity = severity;
    packet.id = id;
    packet.chunk_seq = chunk_seq;
    mav_array_memcpy(packet.text, text, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#endif
}

/**
 * @brief Pack a statustext message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param severity  Severity of status. Relies on the definitions within RFC-5424.
 * @param text  Status text message, without null termination character
 * @param id  Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence and the message can be emitted immediately.
 * @param chunk_seq  This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to mean this was the last chunk.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t severity,const char *text,uint16_t id,uint8_t chunk_seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];
    _mav_put_uint8_t(buf, 0, severity);
    _mav_put_uint16_t(buf, 51, id);
    _mav_put_uint8_t(buf, 53, chunk_seq);
    _mav_put_char_array(buf, 1, text, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#else
    mavlink_statustext_t packet;
    packet.severity = severity;
    packet.id = id;
    packet.chunk_seq = chunk_seq;
    mav_array_memcpy(packet.text, text, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUSTEXT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
}

/**
 * @brief Encode a statustext struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_statustext_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_statustext_t* statustext)
{
    return mavlink_msg_statustext_pack(system_id, component_id, msg, statustext->severity, statustext->text, statustext->id, statustext->chunk_seq);
}

/**
 * @brief Encode a statustext struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_statustext_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_statustext_t* statustext)
{
    return mavlink_msg_statustext_pack_chan(system_id, component_id, chan, msg, statustext->severity, statustext->text, statustext->id, statustext->chunk_seq);
}

/**
 * @brief Encode a statustext struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_statustext_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_statustext_t* statustext)
{
    return mavlink_msg_statustext_pack_status(system_id, component_id, _status, msg,  statustext->severity, statustext->text, statustext->id, statustext->chunk_seq);
}

/**
 * @brief Send a statustext message
 * @param chan MAVLink channel to send the message
 *
 * @param severity  Severity of status. Relies on the definitions within RFC-5424.
 * @param text  Status text message, without null termination character
 * @param id  Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence and the message can be emitted immediately.
 * @param chunk_seq  This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to mean this was the last chunk.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_statustext_send(mavlink_channel_t chan, uint8_t severity, const char *text, uint16_t id, uint8_t chunk_seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];
    _mav_put_uint8_t(buf, 0, severity);
    _mav_put_uint16_t(buf, 51, id);
    _mav_put_uint8_t(buf, 53, chunk_seq);
    _mav_put_char_array(buf, 1, text, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, buf, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#else
    mavlink_statustext_t packet;
    packet.severity = severity;
    packet.id = id;
    packet.chunk_seq = chunk_seq;
    mav_array_memcpy(packet.text, text, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, (const char *)&packet, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#endif
}

/**
 * @brief Send a statustext message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_statustext_send_struct(mavlink_channel_t chan, const mavlink_statustext_t* statustext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_statustext_send(chan, statustext->severity, statustext->text, statustext->id, statustext->chunk_seq);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, (const char *)statustext, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#endif
}

#if MAVLINK_MSG_ID_STATUSTEXT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_statustext_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t severity, const char *text, uint16_t id, uint8_t chunk_seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, severity);
    _mav_put_uint16_t(buf, 51, id);
    _mav_put_uint8_t(buf, 53, chunk_seq);
    _mav_put_char_array(buf, 1, text, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, buf, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#else
    mavlink_statustext_t *packet = (mavlink_statustext_t *)msgbuf;
    packet->severity = severity;
    packet->id = id;
    packet->chunk_seq = chunk_seq;
    mav_array_memcpy(packet->text, text, sizeof(char)*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, (const char *)packet, MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN, MAVLINK_MSG_ID_STATUSTEXT_LEN, MAVLINK_MSG_ID_STATUSTEXT_CRC);
#endif
}
#endif

#endif

// MESSAGE STATUSTEXT UNPACKING


/**
 * @brief Get field severity from statustext message
 *
 * @return  Severity of status. Relies on the definitions within RFC-5424.
 */
static inline uint8_t mavlink_msg_statustext_get_severity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field text from statustext message
 *
 * @return  Status text message, without null termination character
 */
static inline uint16_t mavlink_msg_statustext_get_text(const mavlink_message_t* msg, char *text)
{
    return _MAV_RETURN_char_array(msg, text, 50,  1);
}

/**
 * @brief Get field id from statustext message
 *
 * @return  Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence and the message can be emitted immediately.
 */
static inline uint16_t mavlink_msg_statustext_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  51);
}

/**
 * @brief Get field chunk_seq from statustext message
 *
 * @return  This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to mean this was the last chunk.
 */
static inline uint8_t mavlink_msg_statustext_get_chunk_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Decode a statustext message into a struct
 *
 * @param msg The message to decode
 * @param statustext C-struct to decode the message contents into
 */
static inline void mavlink_msg_statustext_decode(const mavlink_message_t* msg, mavlink_statustext_t* statustext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    statustext->severity = mavlink_msg_statustext_get_severity(msg);
    mavlink_msg_statustext_get_text(msg, statustext->text);
    statustext->id = mavlink_msg_statustext_get_id(msg);
    statustext->chunk_seq = mavlink_msg_statustext_get_chunk_seq(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STATUSTEXT_LEN? msg->len : MAVLINK_MSG_ID_STATUSTEXT_LEN;
        memset(statustext, 0, MAVLINK_MSG_ID_STATUSTEXT_LEN);
    memcpy(statustext, _MAV_PAYLOAD(msg), len);
#endif
}

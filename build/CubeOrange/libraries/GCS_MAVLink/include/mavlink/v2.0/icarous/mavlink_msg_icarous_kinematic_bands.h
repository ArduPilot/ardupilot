#pragma once
// MESSAGE ICAROUS_KINEMATIC_BANDS PACKING

#define MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS 42001


typedef struct __mavlink_icarous_kinematic_bands_t {
 float min1; /*< [deg] min angle (degrees)*/
 float max1; /*< [deg] max angle (degrees)*/
 float min2; /*< [deg] min angle (degrees)*/
 float max2; /*< [deg] max angle (degrees)*/
 float min3; /*< [deg] min angle (degrees)*/
 float max3; /*< [deg] max angle (degrees)*/
 float min4; /*< [deg] min angle (degrees)*/
 float max4; /*< [deg] max angle (degrees)*/
 float min5; /*< [deg] min angle (degrees)*/
 float max5; /*< [deg] max angle (degrees)*/
 int8_t numBands; /*<  Number of track bands*/
 uint8_t type1; /*<  See the TRACK_BAND_TYPES enum.*/
 uint8_t type2; /*<  See the TRACK_BAND_TYPES enum.*/
 uint8_t type3; /*<  See the TRACK_BAND_TYPES enum.*/
 uint8_t type4; /*<  See the TRACK_BAND_TYPES enum.*/
 uint8_t type5; /*<  See the TRACK_BAND_TYPES enum.*/
} mavlink_icarous_kinematic_bands_t;

#define MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN 46
#define MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN 46
#define MAVLINK_MSG_ID_42001_LEN 46
#define MAVLINK_MSG_ID_42001_MIN_LEN 46

#define MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_CRC 239
#define MAVLINK_MSG_ID_42001_CRC 239



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ICAROUS_KINEMATIC_BANDS { \
    42001, \
    "ICAROUS_KINEMATIC_BANDS", \
    16, \
    {  { "numBands", NULL, MAVLINK_TYPE_INT8_T, 0, 40, offsetof(mavlink_icarous_kinematic_bands_t, numBands) }, \
         { "type1", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_icarous_kinematic_bands_t, type1) }, \
         { "min1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_icarous_kinematic_bands_t, min1) }, \
         { "max1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_icarous_kinematic_bands_t, max1) }, \
         { "type2", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_icarous_kinematic_bands_t, type2) }, \
         { "min2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_icarous_kinematic_bands_t, min2) }, \
         { "max2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_icarous_kinematic_bands_t, max2) }, \
         { "type3", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_icarous_kinematic_bands_t, type3) }, \
         { "min3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_icarous_kinematic_bands_t, min3) }, \
         { "max3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_icarous_kinematic_bands_t, max3) }, \
         { "type4", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_icarous_kinematic_bands_t, type4) }, \
         { "min4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_icarous_kinematic_bands_t, min4) }, \
         { "max4", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_icarous_kinematic_bands_t, max4) }, \
         { "type5", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_icarous_kinematic_bands_t, type5) }, \
         { "min5", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_icarous_kinematic_bands_t, min5) }, \
         { "max5", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_icarous_kinematic_bands_t, max5) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ICAROUS_KINEMATIC_BANDS { \
    "ICAROUS_KINEMATIC_BANDS", \
    16, \
    {  { "numBands", NULL, MAVLINK_TYPE_INT8_T, 0, 40, offsetof(mavlink_icarous_kinematic_bands_t, numBands) }, \
         { "type1", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_icarous_kinematic_bands_t, type1) }, \
         { "min1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_icarous_kinematic_bands_t, min1) }, \
         { "max1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_icarous_kinematic_bands_t, max1) }, \
         { "type2", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_icarous_kinematic_bands_t, type2) }, \
         { "min2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_icarous_kinematic_bands_t, min2) }, \
         { "max2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_icarous_kinematic_bands_t, max2) }, \
         { "type3", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_icarous_kinematic_bands_t, type3) }, \
         { "min3", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_icarous_kinematic_bands_t, min3) }, \
         { "max3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_icarous_kinematic_bands_t, max3) }, \
         { "type4", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_icarous_kinematic_bands_t, type4) }, \
         { "min4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_icarous_kinematic_bands_t, min4) }, \
         { "max4", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_icarous_kinematic_bands_t, max4) }, \
         { "type5", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_icarous_kinematic_bands_t, type5) }, \
         { "min5", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_icarous_kinematic_bands_t, min5) }, \
         { "max5", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_icarous_kinematic_bands_t, max5) }, \
         } \
}
#endif

/**
 * @brief Pack a icarous_kinematic_bands message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param numBands  Number of track bands
 * @param type1  See the TRACK_BAND_TYPES enum.
 * @param min1 [deg] min angle (degrees)
 * @param max1 [deg] max angle (degrees)
 * @param type2  See the TRACK_BAND_TYPES enum.
 * @param min2 [deg] min angle (degrees)
 * @param max2 [deg] max angle (degrees)
 * @param type3  See the TRACK_BAND_TYPES enum.
 * @param min3 [deg] min angle (degrees)
 * @param max3 [deg] max angle (degrees)
 * @param type4  See the TRACK_BAND_TYPES enum.
 * @param min4 [deg] min angle (degrees)
 * @param max4 [deg] max angle (degrees)
 * @param type5  See the TRACK_BAND_TYPES enum.
 * @param min5 [deg] min angle (degrees)
 * @param max5 [deg] max angle (degrees)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_icarous_kinematic_bands_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN];
    _mav_put_float(buf, 0, min1);
    _mav_put_float(buf, 4, max1);
    _mav_put_float(buf, 8, min2);
    _mav_put_float(buf, 12, max2);
    _mav_put_float(buf, 16, min3);
    _mav_put_float(buf, 20, max3);
    _mav_put_float(buf, 24, min4);
    _mav_put_float(buf, 28, max4);
    _mav_put_float(buf, 32, min5);
    _mav_put_float(buf, 36, max5);
    _mav_put_int8_t(buf, 40, numBands);
    _mav_put_uint8_t(buf, 41, type1);
    _mav_put_uint8_t(buf, 42, type2);
    _mav_put_uint8_t(buf, 43, type3);
    _mav_put_uint8_t(buf, 44, type4);
    _mav_put_uint8_t(buf, 45, type5);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN);
#else
    mavlink_icarous_kinematic_bands_t packet;
    packet.min1 = min1;
    packet.max1 = max1;
    packet.min2 = min2;
    packet.max2 = max2;
    packet.min3 = min3;
    packet.max3 = max3;
    packet.min4 = min4;
    packet.max4 = max4;
    packet.min5 = min5;
    packet.max5 = max5;
    packet.numBands = numBands;
    packet.type1 = type1;
    packet.type2 = type2;
    packet.type3 = type3;
    packet.type4 = type4;
    packet.type5 = type5;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_CRC);
}

/**
 * @brief Pack a icarous_kinematic_bands message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param numBands  Number of track bands
 * @param type1  See the TRACK_BAND_TYPES enum.
 * @param min1 [deg] min angle (degrees)
 * @param max1 [deg] max angle (degrees)
 * @param type2  See the TRACK_BAND_TYPES enum.
 * @param min2 [deg] min angle (degrees)
 * @param max2 [deg] max angle (degrees)
 * @param type3  See the TRACK_BAND_TYPES enum.
 * @param min3 [deg] min angle (degrees)
 * @param max3 [deg] max angle (degrees)
 * @param type4  See the TRACK_BAND_TYPES enum.
 * @param min4 [deg] min angle (degrees)
 * @param max4 [deg] max angle (degrees)
 * @param type5  See the TRACK_BAND_TYPES enum.
 * @param min5 [deg] min angle (degrees)
 * @param max5 [deg] max angle (degrees)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_icarous_kinematic_bands_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int8_t numBands,uint8_t type1,float min1,float max1,uint8_t type2,float min2,float max2,uint8_t type3,float min3,float max3,uint8_t type4,float min4,float max4,uint8_t type5,float min5,float max5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN];
    _mav_put_float(buf, 0, min1);
    _mav_put_float(buf, 4, max1);
    _mav_put_float(buf, 8, min2);
    _mav_put_float(buf, 12, max2);
    _mav_put_float(buf, 16, min3);
    _mav_put_float(buf, 20, max3);
    _mav_put_float(buf, 24, min4);
    _mav_put_float(buf, 28, max4);
    _mav_put_float(buf, 32, min5);
    _mav_put_float(buf, 36, max5);
    _mav_put_int8_t(buf, 40, numBands);
    _mav_put_uint8_t(buf, 41, type1);
    _mav_put_uint8_t(buf, 42, type2);
    _mav_put_uint8_t(buf, 43, type3);
    _mav_put_uint8_t(buf, 44, type4);
    _mav_put_uint8_t(buf, 45, type5);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN);
#else
    mavlink_icarous_kinematic_bands_t packet;
    packet.min1 = min1;
    packet.max1 = max1;
    packet.min2 = min2;
    packet.max2 = max2;
    packet.min3 = min3;
    packet.max3 = max3;
    packet.min4 = min4;
    packet.max4 = max4;
    packet.min5 = min5;
    packet.max5 = max5;
    packet.numBands = numBands;
    packet.type1 = type1;
    packet.type2 = type2;
    packet.type3 = type3;
    packet.type4 = type4;
    packet.type5 = type5;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_CRC);
}

/**
 * @brief Encode a icarous_kinematic_bands struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param icarous_kinematic_bands C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_icarous_kinematic_bands_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_icarous_kinematic_bands_t* icarous_kinematic_bands)
{
    return mavlink_msg_icarous_kinematic_bands_pack(system_id, component_id, msg, icarous_kinematic_bands->numBands, icarous_kinematic_bands->type1, icarous_kinematic_bands->min1, icarous_kinematic_bands->max1, icarous_kinematic_bands->type2, icarous_kinematic_bands->min2, icarous_kinematic_bands->max2, icarous_kinematic_bands->type3, icarous_kinematic_bands->min3, icarous_kinematic_bands->max3, icarous_kinematic_bands->type4, icarous_kinematic_bands->min4, icarous_kinematic_bands->max4, icarous_kinematic_bands->type5, icarous_kinematic_bands->min5, icarous_kinematic_bands->max5);
}

/**
 * @brief Encode a icarous_kinematic_bands struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param icarous_kinematic_bands C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_icarous_kinematic_bands_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_icarous_kinematic_bands_t* icarous_kinematic_bands)
{
    return mavlink_msg_icarous_kinematic_bands_pack_chan(system_id, component_id, chan, msg, icarous_kinematic_bands->numBands, icarous_kinematic_bands->type1, icarous_kinematic_bands->min1, icarous_kinematic_bands->max1, icarous_kinematic_bands->type2, icarous_kinematic_bands->min2, icarous_kinematic_bands->max2, icarous_kinematic_bands->type3, icarous_kinematic_bands->min3, icarous_kinematic_bands->max3, icarous_kinematic_bands->type4, icarous_kinematic_bands->min4, icarous_kinematic_bands->max4, icarous_kinematic_bands->type5, icarous_kinematic_bands->min5, icarous_kinematic_bands->max5);
}

/**
 * @brief Send a icarous_kinematic_bands message
 * @param chan MAVLink channel to send the message
 *
 * @param numBands  Number of track bands
 * @param type1  See the TRACK_BAND_TYPES enum.
 * @param min1 [deg] min angle (degrees)
 * @param max1 [deg] max angle (degrees)
 * @param type2  See the TRACK_BAND_TYPES enum.
 * @param min2 [deg] min angle (degrees)
 * @param max2 [deg] max angle (degrees)
 * @param type3  See the TRACK_BAND_TYPES enum.
 * @param min3 [deg] min angle (degrees)
 * @param max3 [deg] max angle (degrees)
 * @param type4  See the TRACK_BAND_TYPES enum.
 * @param min4 [deg] min angle (degrees)
 * @param max4 [deg] max angle (degrees)
 * @param type5  See the TRACK_BAND_TYPES enum.
 * @param min5 [deg] min angle (degrees)
 * @param max5 [deg] max angle (degrees)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_icarous_kinematic_bands_send(mavlink_channel_t chan, int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN];
    _mav_put_float(buf, 0, min1);
    _mav_put_float(buf, 4, max1);
    _mav_put_float(buf, 8, min2);
    _mav_put_float(buf, 12, max2);
    _mav_put_float(buf, 16, min3);
    _mav_put_float(buf, 20, max3);
    _mav_put_float(buf, 24, min4);
    _mav_put_float(buf, 28, max4);
    _mav_put_float(buf, 32, min5);
    _mav_put_float(buf, 36, max5);
    _mav_put_int8_t(buf, 40, numBands);
    _mav_put_uint8_t(buf, 41, type1);
    _mav_put_uint8_t(buf, 42, type2);
    _mav_put_uint8_t(buf, 43, type3);
    _mav_put_uint8_t(buf, 44, type4);
    _mav_put_uint8_t(buf, 45, type5);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS, buf, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_CRC);
#else
    mavlink_icarous_kinematic_bands_t packet;
    packet.min1 = min1;
    packet.max1 = max1;
    packet.min2 = min2;
    packet.max2 = max2;
    packet.min3 = min3;
    packet.max3 = max3;
    packet.min4 = min4;
    packet.max4 = max4;
    packet.min5 = min5;
    packet.max5 = max5;
    packet.numBands = numBands;
    packet.type1 = type1;
    packet.type2 = type2;
    packet.type3 = type3;
    packet.type4 = type4;
    packet.type5 = type5;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS, (const char *)&packet, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_CRC);
#endif
}

/**
 * @brief Send a icarous_kinematic_bands message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_icarous_kinematic_bands_send_struct(mavlink_channel_t chan, const mavlink_icarous_kinematic_bands_t* icarous_kinematic_bands)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_icarous_kinematic_bands_send(chan, icarous_kinematic_bands->numBands, icarous_kinematic_bands->type1, icarous_kinematic_bands->min1, icarous_kinematic_bands->max1, icarous_kinematic_bands->type2, icarous_kinematic_bands->min2, icarous_kinematic_bands->max2, icarous_kinematic_bands->type3, icarous_kinematic_bands->min3, icarous_kinematic_bands->max3, icarous_kinematic_bands->type4, icarous_kinematic_bands->min4, icarous_kinematic_bands->max4, icarous_kinematic_bands->type5, icarous_kinematic_bands->min5, icarous_kinematic_bands->max5);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS, (const char *)icarous_kinematic_bands, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_icarous_kinematic_bands_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int8_t numBands, uint8_t type1, float min1, float max1, uint8_t type2, float min2, float max2, uint8_t type3, float min3, float max3, uint8_t type4, float min4, float max4, uint8_t type5, float min5, float max5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, min1);
    _mav_put_float(buf, 4, max1);
    _mav_put_float(buf, 8, min2);
    _mav_put_float(buf, 12, max2);
    _mav_put_float(buf, 16, min3);
    _mav_put_float(buf, 20, max3);
    _mav_put_float(buf, 24, min4);
    _mav_put_float(buf, 28, max4);
    _mav_put_float(buf, 32, min5);
    _mav_put_float(buf, 36, max5);
    _mav_put_int8_t(buf, 40, numBands);
    _mav_put_uint8_t(buf, 41, type1);
    _mav_put_uint8_t(buf, 42, type2);
    _mav_put_uint8_t(buf, 43, type3);
    _mav_put_uint8_t(buf, 44, type4);
    _mav_put_uint8_t(buf, 45, type5);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS, buf, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_CRC);
#else
    mavlink_icarous_kinematic_bands_t *packet = (mavlink_icarous_kinematic_bands_t *)msgbuf;
    packet->min1 = min1;
    packet->max1 = max1;
    packet->min2 = min2;
    packet->max2 = max2;
    packet->min3 = min3;
    packet->max3 = max3;
    packet->min4 = min4;
    packet->max4 = max4;
    packet->min5 = min5;
    packet->max5 = max5;
    packet->numBands = numBands;
    packet->type1 = type1;
    packet->type2 = type2;
    packet->type3 = type3;
    packet->type4 = type4;
    packet->type5 = type5;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS, (const char *)packet, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_MIN_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_CRC);
#endif
}
#endif

#endif

// MESSAGE ICAROUS_KINEMATIC_BANDS UNPACKING


/**
 * @brief Get field numBands from icarous_kinematic_bands message
 *
 * @return  Number of track bands
 */
static inline int8_t mavlink_msg_icarous_kinematic_bands_get_numBands(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  40);
}

/**
 * @brief Get field type1 from icarous_kinematic_bands message
 *
 * @return  See the TRACK_BAND_TYPES enum.
 */
static inline uint8_t mavlink_msg_icarous_kinematic_bands_get_type1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field min1 from icarous_kinematic_bands message
 *
 * @return [deg] min angle (degrees)
 */
static inline float mavlink_msg_icarous_kinematic_bands_get_min1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field max1 from icarous_kinematic_bands message
 *
 * @return [deg] max angle (degrees)
 */
static inline float mavlink_msg_icarous_kinematic_bands_get_max1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field type2 from icarous_kinematic_bands message
 *
 * @return  See the TRACK_BAND_TYPES enum.
 */
static inline uint8_t mavlink_msg_icarous_kinematic_bands_get_type2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field min2 from icarous_kinematic_bands message
 *
 * @return [deg] min angle (degrees)
 */
static inline float mavlink_msg_icarous_kinematic_bands_get_min2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field max2 from icarous_kinematic_bands message
 *
 * @return [deg] max angle (degrees)
 */
static inline float mavlink_msg_icarous_kinematic_bands_get_max2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field type3 from icarous_kinematic_bands message
 *
 * @return  See the TRACK_BAND_TYPES enum.
 */
static inline uint8_t mavlink_msg_icarous_kinematic_bands_get_type3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field min3 from icarous_kinematic_bands message
 *
 * @return [deg] min angle (degrees)
 */
static inline float mavlink_msg_icarous_kinematic_bands_get_min3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field max3 from icarous_kinematic_bands message
 *
 * @return [deg] max angle (degrees)
 */
static inline float mavlink_msg_icarous_kinematic_bands_get_max3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field type4 from icarous_kinematic_bands message
 *
 * @return  See the TRACK_BAND_TYPES enum.
 */
static inline uint8_t mavlink_msg_icarous_kinematic_bands_get_type4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field min4 from icarous_kinematic_bands message
 *
 * @return [deg] min angle (degrees)
 */
static inline float mavlink_msg_icarous_kinematic_bands_get_min4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field max4 from icarous_kinematic_bands message
 *
 * @return [deg] max angle (degrees)
 */
static inline float mavlink_msg_icarous_kinematic_bands_get_max4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field type5 from icarous_kinematic_bands message
 *
 * @return  See the TRACK_BAND_TYPES enum.
 */
static inline uint8_t mavlink_msg_icarous_kinematic_bands_get_type5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field min5 from icarous_kinematic_bands message
 *
 * @return [deg] min angle (degrees)
 */
static inline float mavlink_msg_icarous_kinematic_bands_get_min5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field max5 from icarous_kinematic_bands message
 *
 * @return [deg] max angle (degrees)
 */
static inline float mavlink_msg_icarous_kinematic_bands_get_max5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a icarous_kinematic_bands message into a struct
 *
 * @param msg The message to decode
 * @param icarous_kinematic_bands C-struct to decode the message contents into
 */
static inline void mavlink_msg_icarous_kinematic_bands_decode(const mavlink_message_t* msg, mavlink_icarous_kinematic_bands_t* icarous_kinematic_bands)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    icarous_kinematic_bands->min1 = mavlink_msg_icarous_kinematic_bands_get_min1(msg);
    icarous_kinematic_bands->max1 = mavlink_msg_icarous_kinematic_bands_get_max1(msg);
    icarous_kinematic_bands->min2 = mavlink_msg_icarous_kinematic_bands_get_min2(msg);
    icarous_kinematic_bands->max2 = mavlink_msg_icarous_kinematic_bands_get_max2(msg);
    icarous_kinematic_bands->min3 = mavlink_msg_icarous_kinematic_bands_get_min3(msg);
    icarous_kinematic_bands->max3 = mavlink_msg_icarous_kinematic_bands_get_max3(msg);
    icarous_kinematic_bands->min4 = mavlink_msg_icarous_kinematic_bands_get_min4(msg);
    icarous_kinematic_bands->max4 = mavlink_msg_icarous_kinematic_bands_get_max4(msg);
    icarous_kinematic_bands->min5 = mavlink_msg_icarous_kinematic_bands_get_min5(msg);
    icarous_kinematic_bands->max5 = mavlink_msg_icarous_kinematic_bands_get_max5(msg);
    icarous_kinematic_bands->numBands = mavlink_msg_icarous_kinematic_bands_get_numBands(msg);
    icarous_kinematic_bands->type1 = mavlink_msg_icarous_kinematic_bands_get_type1(msg);
    icarous_kinematic_bands->type2 = mavlink_msg_icarous_kinematic_bands_get_type2(msg);
    icarous_kinematic_bands->type3 = mavlink_msg_icarous_kinematic_bands_get_type3(msg);
    icarous_kinematic_bands->type4 = mavlink_msg_icarous_kinematic_bands_get_type4(msg);
    icarous_kinematic_bands->type5 = mavlink_msg_icarous_kinematic_bands_get_type5(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN? msg->len : MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN;
        memset(icarous_kinematic_bands, 0, MAVLINK_MSG_ID_ICAROUS_KINEMATIC_BANDS_LEN);
    memcpy(icarous_kinematic_bands, _MAV_PAYLOAD(msg), len);
#endif
}

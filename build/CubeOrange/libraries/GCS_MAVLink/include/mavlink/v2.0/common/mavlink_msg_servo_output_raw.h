#pragma once
// MESSAGE SERVO_OUTPUT_RAW PACKING

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 36

MAVPACKED(
typedef struct __mavlink_servo_output_raw_t {
 uint32_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 uint16_t servo1_raw; /*< [us] Servo output 1 value*/
 uint16_t servo2_raw; /*< [us] Servo output 2 value*/
 uint16_t servo3_raw; /*< [us] Servo output 3 value*/
 uint16_t servo4_raw; /*< [us] Servo output 4 value*/
 uint16_t servo5_raw; /*< [us] Servo output 5 value*/
 uint16_t servo6_raw; /*< [us] Servo output 6 value*/
 uint16_t servo7_raw; /*< [us] Servo output 7 value*/
 uint16_t servo8_raw; /*< [us] Servo output 8 value*/
 uint8_t port; /*<  Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.*/
 uint16_t servo9_raw; /*< [us] Servo output 9 value*/
 uint16_t servo10_raw; /*< [us] Servo output 10 value*/
 uint16_t servo11_raw; /*< [us] Servo output 11 value*/
 uint16_t servo12_raw; /*< [us] Servo output 12 value*/
 uint16_t servo13_raw; /*< [us] Servo output 13 value*/
 uint16_t servo14_raw; /*< [us] Servo output 14 value*/
 uint16_t servo15_raw; /*< [us] Servo output 15 value*/
 uint16_t servo16_raw; /*< [us] Servo output 16 value*/
}) mavlink_servo_output_raw_t;

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN 37
#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN 21
#define MAVLINK_MSG_ID_36_LEN 37
#define MAVLINK_MSG_ID_36_MIN_LEN 21

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC 222
#define MAVLINK_MSG_ID_36_CRC 222



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERVO_OUTPUT_RAW { \
    36, \
    "SERVO_OUTPUT_RAW", \
    18, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_servo_output_raw_t, time_usec) }, \
         { "port", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_servo_output_raw_t, port) }, \
         { "servo1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_servo_output_raw_t, servo1_raw) }, \
         { "servo2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_servo_output_raw_t, servo2_raw) }, \
         { "servo3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_servo_output_raw_t, servo3_raw) }, \
         { "servo4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_servo_output_raw_t, servo4_raw) }, \
         { "servo5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_servo_output_raw_t, servo5_raw) }, \
         { "servo6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_servo_output_raw_t, servo6_raw) }, \
         { "servo7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_servo_output_raw_t, servo7_raw) }, \
         { "servo8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_servo_output_raw_t, servo8_raw) }, \
         { "servo9_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 21, offsetof(mavlink_servo_output_raw_t, servo9_raw) }, \
         { "servo10_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 23, offsetof(mavlink_servo_output_raw_t, servo10_raw) }, \
         { "servo11_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 25, offsetof(mavlink_servo_output_raw_t, servo11_raw) }, \
         { "servo12_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 27, offsetof(mavlink_servo_output_raw_t, servo12_raw) }, \
         { "servo13_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 29, offsetof(mavlink_servo_output_raw_t, servo13_raw) }, \
         { "servo14_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 31, offsetof(mavlink_servo_output_raw_t, servo14_raw) }, \
         { "servo15_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 33, offsetof(mavlink_servo_output_raw_t, servo15_raw) }, \
         { "servo16_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 35, offsetof(mavlink_servo_output_raw_t, servo16_raw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERVO_OUTPUT_RAW { \
    "SERVO_OUTPUT_RAW", \
    18, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_servo_output_raw_t, time_usec) }, \
         { "port", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_servo_output_raw_t, port) }, \
         { "servo1_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_servo_output_raw_t, servo1_raw) }, \
         { "servo2_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_servo_output_raw_t, servo2_raw) }, \
         { "servo3_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_servo_output_raw_t, servo3_raw) }, \
         { "servo4_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_servo_output_raw_t, servo4_raw) }, \
         { "servo5_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_servo_output_raw_t, servo5_raw) }, \
         { "servo6_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_servo_output_raw_t, servo6_raw) }, \
         { "servo7_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_servo_output_raw_t, servo7_raw) }, \
         { "servo8_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_servo_output_raw_t, servo8_raw) }, \
         { "servo9_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 21, offsetof(mavlink_servo_output_raw_t, servo9_raw) }, \
         { "servo10_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 23, offsetof(mavlink_servo_output_raw_t, servo10_raw) }, \
         { "servo11_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 25, offsetof(mavlink_servo_output_raw_t, servo11_raw) }, \
         { "servo12_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 27, offsetof(mavlink_servo_output_raw_t, servo12_raw) }, \
         { "servo13_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 29, offsetof(mavlink_servo_output_raw_t, servo13_raw) }, \
         { "servo14_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 31, offsetof(mavlink_servo_output_raw_t, servo14_raw) }, \
         { "servo15_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 33, offsetof(mavlink_servo_output_raw_t, servo15_raw) }, \
         { "servo16_raw", NULL, MAVLINK_TYPE_UINT16_T, 0, 35, offsetof(mavlink_servo_output_raw_t, servo16_raw) }, \
         } \
}
#endif

/**
 * @brief Pack a servo_output_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param port  Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
 * @param servo1_raw [us] Servo output 1 value
 * @param servo2_raw [us] Servo output 2 value
 * @param servo3_raw [us] Servo output 3 value
 * @param servo4_raw [us] Servo output 4 value
 * @param servo5_raw [us] Servo output 5 value
 * @param servo6_raw [us] Servo output 6 value
 * @param servo7_raw [us] Servo output 7 value
 * @param servo8_raw [us] Servo output 8 value
 * @param servo9_raw [us] Servo output 9 value
 * @param servo10_raw [us] Servo output 10 value
 * @param servo11_raw [us] Servo output 11 value
 * @param servo12_raw [us] Servo output 12 value
 * @param servo13_raw [us] Servo output 13 value
 * @param servo14_raw [us] Servo output 14 value
 * @param servo15_raw [us] Servo output 15 value
 * @param servo16_raw [us] Servo output 16 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_servo_output_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw, uint16_t servo9_raw, uint16_t servo10_raw, uint16_t servo11_raw, uint16_t servo12_raw, uint16_t servo13_raw, uint16_t servo14_raw, uint16_t servo15_raw, uint16_t servo16_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 4, servo1_raw);
    _mav_put_uint16_t(buf, 6, servo2_raw);
    _mav_put_uint16_t(buf, 8, servo3_raw);
    _mav_put_uint16_t(buf, 10, servo4_raw);
    _mav_put_uint16_t(buf, 12, servo5_raw);
    _mav_put_uint16_t(buf, 14, servo6_raw);
    _mav_put_uint16_t(buf, 16, servo7_raw);
    _mav_put_uint16_t(buf, 18, servo8_raw);
    _mav_put_uint8_t(buf, 20, port);
    _mav_put_uint16_t(buf, 21, servo9_raw);
    _mav_put_uint16_t(buf, 23, servo10_raw);
    _mav_put_uint16_t(buf, 25, servo11_raw);
    _mav_put_uint16_t(buf, 27, servo12_raw);
    _mav_put_uint16_t(buf, 29, servo13_raw);
    _mav_put_uint16_t(buf, 31, servo14_raw);
    _mav_put_uint16_t(buf, 33, servo15_raw);
    _mav_put_uint16_t(buf, 35, servo16_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN);
#else
    mavlink_servo_output_raw_t packet;
    packet.time_usec = time_usec;
    packet.servo1_raw = servo1_raw;
    packet.servo2_raw = servo2_raw;
    packet.servo3_raw = servo3_raw;
    packet.servo4_raw = servo4_raw;
    packet.servo5_raw = servo5_raw;
    packet.servo6_raw = servo6_raw;
    packet.servo7_raw = servo7_raw;
    packet.servo8_raw = servo8_raw;
    packet.port = port;
    packet.servo9_raw = servo9_raw;
    packet.servo10_raw = servo10_raw;
    packet.servo11_raw = servo11_raw;
    packet.servo12_raw = servo12_raw;
    packet.servo13_raw = servo13_raw;
    packet.servo14_raw = servo14_raw;
    packet.servo15_raw = servo15_raw;
    packet.servo16_raw = servo16_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC);
}

/**
 * @brief Pack a servo_output_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param port  Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
 * @param servo1_raw [us] Servo output 1 value
 * @param servo2_raw [us] Servo output 2 value
 * @param servo3_raw [us] Servo output 3 value
 * @param servo4_raw [us] Servo output 4 value
 * @param servo5_raw [us] Servo output 5 value
 * @param servo6_raw [us] Servo output 6 value
 * @param servo7_raw [us] Servo output 7 value
 * @param servo8_raw [us] Servo output 8 value
 * @param servo9_raw [us] Servo output 9 value
 * @param servo10_raw [us] Servo output 10 value
 * @param servo11_raw [us] Servo output 11 value
 * @param servo12_raw [us] Servo output 12 value
 * @param servo13_raw [us] Servo output 13 value
 * @param servo14_raw [us] Servo output 14 value
 * @param servo15_raw [us] Servo output 15 value
 * @param servo16_raw [us] Servo output 16 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_servo_output_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_usec,uint8_t port,uint16_t servo1_raw,uint16_t servo2_raw,uint16_t servo3_raw,uint16_t servo4_raw,uint16_t servo5_raw,uint16_t servo6_raw,uint16_t servo7_raw,uint16_t servo8_raw,uint16_t servo9_raw,uint16_t servo10_raw,uint16_t servo11_raw,uint16_t servo12_raw,uint16_t servo13_raw,uint16_t servo14_raw,uint16_t servo15_raw,uint16_t servo16_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 4, servo1_raw);
    _mav_put_uint16_t(buf, 6, servo2_raw);
    _mav_put_uint16_t(buf, 8, servo3_raw);
    _mav_put_uint16_t(buf, 10, servo4_raw);
    _mav_put_uint16_t(buf, 12, servo5_raw);
    _mav_put_uint16_t(buf, 14, servo6_raw);
    _mav_put_uint16_t(buf, 16, servo7_raw);
    _mav_put_uint16_t(buf, 18, servo8_raw);
    _mav_put_uint8_t(buf, 20, port);
    _mav_put_uint16_t(buf, 21, servo9_raw);
    _mav_put_uint16_t(buf, 23, servo10_raw);
    _mav_put_uint16_t(buf, 25, servo11_raw);
    _mav_put_uint16_t(buf, 27, servo12_raw);
    _mav_put_uint16_t(buf, 29, servo13_raw);
    _mav_put_uint16_t(buf, 31, servo14_raw);
    _mav_put_uint16_t(buf, 33, servo15_raw);
    _mav_put_uint16_t(buf, 35, servo16_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN);
#else
    mavlink_servo_output_raw_t packet;
    packet.time_usec = time_usec;
    packet.servo1_raw = servo1_raw;
    packet.servo2_raw = servo2_raw;
    packet.servo3_raw = servo3_raw;
    packet.servo4_raw = servo4_raw;
    packet.servo5_raw = servo5_raw;
    packet.servo6_raw = servo6_raw;
    packet.servo7_raw = servo7_raw;
    packet.servo8_raw = servo8_raw;
    packet.port = port;
    packet.servo9_raw = servo9_raw;
    packet.servo10_raw = servo10_raw;
    packet.servo11_raw = servo11_raw;
    packet.servo12_raw = servo12_raw;
    packet.servo13_raw = servo13_raw;
    packet.servo14_raw = servo14_raw;
    packet.servo15_raw = servo15_raw;
    packet.servo16_raw = servo16_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC);
}

/**
 * @brief Encode a servo_output_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param servo_output_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_servo_output_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_servo_output_raw_t* servo_output_raw)
{
    return mavlink_msg_servo_output_raw_pack(system_id, component_id, msg, servo_output_raw->time_usec, servo_output_raw->port, servo_output_raw->servo1_raw, servo_output_raw->servo2_raw, servo_output_raw->servo3_raw, servo_output_raw->servo4_raw, servo_output_raw->servo5_raw, servo_output_raw->servo6_raw, servo_output_raw->servo7_raw, servo_output_raw->servo8_raw, servo_output_raw->servo9_raw, servo_output_raw->servo10_raw, servo_output_raw->servo11_raw, servo_output_raw->servo12_raw, servo_output_raw->servo13_raw, servo_output_raw->servo14_raw, servo_output_raw->servo15_raw, servo_output_raw->servo16_raw);
}

/**
 * @brief Encode a servo_output_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param servo_output_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_servo_output_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_servo_output_raw_t* servo_output_raw)
{
    return mavlink_msg_servo_output_raw_pack_chan(system_id, component_id, chan, msg, servo_output_raw->time_usec, servo_output_raw->port, servo_output_raw->servo1_raw, servo_output_raw->servo2_raw, servo_output_raw->servo3_raw, servo_output_raw->servo4_raw, servo_output_raw->servo5_raw, servo_output_raw->servo6_raw, servo_output_raw->servo7_raw, servo_output_raw->servo8_raw, servo_output_raw->servo9_raw, servo_output_raw->servo10_raw, servo_output_raw->servo11_raw, servo_output_raw->servo12_raw, servo_output_raw->servo13_raw, servo_output_raw->servo14_raw, servo_output_raw->servo15_raw, servo_output_raw->servo16_raw);
}

/**
 * @brief Send a servo_output_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param port  Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
 * @param servo1_raw [us] Servo output 1 value
 * @param servo2_raw [us] Servo output 2 value
 * @param servo3_raw [us] Servo output 3 value
 * @param servo4_raw [us] Servo output 4 value
 * @param servo5_raw [us] Servo output 5 value
 * @param servo6_raw [us] Servo output 6 value
 * @param servo7_raw [us] Servo output 7 value
 * @param servo8_raw [us] Servo output 8 value
 * @param servo9_raw [us] Servo output 9 value
 * @param servo10_raw [us] Servo output 10 value
 * @param servo11_raw [us] Servo output 11 value
 * @param servo12_raw [us] Servo output 12 value
 * @param servo13_raw [us] Servo output 13 value
 * @param servo14_raw [us] Servo output 14 value
 * @param servo15_raw [us] Servo output 15 value
 * @param servo16_raw [us] Servo output 16 value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_servo_output_raw_send(mavlink_channel_t chan, uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw, uint16_t servo9_raw, uint16_t servo10_raw, uint16_t servo11_raw, uint16_t servo12_raw, uint16_t servo13_raw, uint16_t servo14_raw, uint16_t servo15_raw, uint16_t servo16_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 4, servo1_raw);
    _mav_put_uint16_t(buf, 6, servo2_raw);
    _mav_put_uint16_t(buf, 8, servo3_raw);
    _mav_put_uint16_t(buf, 10, servo4_raw);
    _mav_put_uint16_t(buf, 12, servo5_raw);
    _mav_put_uint16_t(buf, 14, servo6_raw);
    _mav_put_uint16_t(buf, 16, servo7_raw);
    _mav_put_uint16_t(buf, 18, servo8_raw);
    _mav_put_uint8_t(buf, 20, port);
    _mav_put_uint16_t(buf, 21, servo9_raw);
    _mav_put_uint16_t(buf, 23, servo10_raw);
    _mav_put_uint16_t(buf, 25, servo11_raw);
    _mav_put_uint16_t(buf, 27, servo12_raw);
    _mav_put_uint16_t(buf, 29, servo13_raw);
    _mav_put_uint16_t(buf, 31, servo14_raw);
    _mav_put_uint16_t(buf, 33, servo15_raw);
    _mav_put_uint16_t(buf, 35, servo16_raw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, buf, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC);
#else
    mavlink_servo_output_raw_t packet;
    packet.time_usec = time_usec;
    packet.servo1_raw = servo1_raw;
    packet.servo2_raw = servo2_raw;
    packet.servo3_raw = servo3_raw;
    packet.servo4_raw = servo4_raw;
    packet.servo5_raw = servo5_raw;
    packet.servo6_raw = servo6_raw;
    packet.servo7_raw = servo7_raw;
    packet.servo8_raw = servo8_raw;
    packet.port = port;
    packet.servo9_raw = servo9_raw;
    packet.servo10_raw = servo10_raw;
    packet.servo11_raw = servo11_raw;
    packet.servo12_raw = servo12_raw;
    packet.servo13_raw = servo13_raw;
    packet.servo14_raw = servo14_raw;
    packet.servo15_raw = servo15_raw;
    packet.servo16_raw = servo16_raw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, (const char *)&packet, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC);
#endif
}

/**
 * @brief Send a servo_output_raw message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_servo_output_raw_send_struct(mavlink_channel_t chan, const mavlink_servo_output_raw_t* servo_output_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_servo_output_raw_send(chan, servo_output_raw->time_usec, servo_output_raw->port, servo_output_raw->servo1_raw, servo_output_raw->servo2_raw, servo_output_raw->servo3_raw, servo_output_raw->servo4_raw, servo_output_raw->servo5_raw, servo_output_raw->servo6_raw, servo_output_raw->servo7_raw, servo_output_raw->servo8_raw, servo_output_raw->servo9_raw, servo_output_raw->servo10_raw, servo_output_raw->servo11_raw, servo_output_raw->servo12_raw, servo_output_raw->servo13_raw, servo_output_raw->servo14_raw, servo_output_raw->servo15_raw, servo_output_raw->servo16_raw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, (const char *)servo_output_raw, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_servo_output_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw, uint16_t servo9_raw, uint16_t servo10_raw, uint16_t servo11_raw, uint16_t servo12_raw, uint16_t servo13_raw, uint16_t servo14_raw, uint16_t servo15_raw, uint16_t servo16_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_uint16_t(buf, 4, servo1_raw);
    _mav_put_uint16_t(buf, 6, servo2_raw);
    _mav_put_uint16_t(buf, 8, servo3_raw);
    _mav_put_uint16_t(buf, 10, servo4_raw);
    _mav_put_uint16_t(buf, 12, servo5_raw);
    _mav_put_uint16_t(buf, 14, servo6_raw);
    _mav_put_uint16_t(buf, 16, servo7_raw);
    _mav_put_uint16_t(buf, 18, servo8_raw);
    _mav_put_uint8_t(buf, 20, port);
    _mav_put_uint16_t(buf, 21, servo9_raw);
    _mav_put_uint16_t(buf, 23, servo10_raw);
    _mav_put_uint16_t(buf, 25, servo11_raw);
    _mav_put_uint16_t(buf, 27, servo12_raw);
    _mav_put_uint16_t(buf, 29, servo13_raw);
    _mav_put_uint16_t(buf, 31, servo14_raw);
    _mav_put_uint16_t(buf, 33, servo15_raw);
    _mav_put_uint16_t(buf, 35, servo16_raw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, buf, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC);
#else
    mavlink_servo_output_raw_t *packet = (mavlink_servo_output_raw_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->servo1_raw = servo1_raw;
    packet->servo2_raw = servo2_raw;
    packet->servo3_raw = servo3_raw;
    packet->servo4_raw = servo4_raw;
    packet->servo5_raw = servo5_raw;
    packet->servo6_raw = servo6_raw;
    packet->servo7_raw = servo7_raw;
    packet->servo8_raw = servo8_raw;
    packet->port = port;
    packet->servo9_raw = servo9_raw;
    packet->servo10_raw = servo10_raw;
    packet->servo11_raw = servo11_raw;
    packet->servo12_raw = servo12_raw;
    packet->servo13_raw = servo13_raw;
    packet->servo14_raw = servo14_raw;
    packet->servo15_raw = servo15_raw;
    packet->servo16_raw = servo16_raw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, (const char *)packet, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC);
#endif
}
#endif

#endif

// MESSAGE SERVO_OUTPUT_RAW UNPACKING


/**
 * @brief Get field time_usec from servo_output_raw message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint32_t mavlink_msg_servo_output_raw_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field port from servo_output_raw message
 *
 * @return  Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
 */
static inline uint8_t mavlink_msg_servo_output_raw_get_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field servo1_raw from servo_output_raw message
 *
 * @return [us] Servo output 1 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo1_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field servo2_raw from servo_output_raw message
 *
 * @return [us] Servo output 2 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo2_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field servo3_raw from servo_output_raw message
 *
 * @return [us] Servo output 3 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo3_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field servo4_raw from servo_output_raw message
 *
 * @return [us] Servo output 4 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo4_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field servo5_raw from servo_output_raw message
 *
 * @return [us] Servo output 5 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo5_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field servo6_raw from servo_output_raw message
 *
 * @return [us] Servo output 6 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo6_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field servo7_raw from servo_output_raw message
 *
 * @return [us] Servo output 7 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo7_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field servo8_raw from servo_output_raw message
 *
 * @return [us] Servo output 8 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo8_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field servo9_raw from servo_output_raw message
 *
 * @return [us] Servo output 9 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo9_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  21);
}

/**
 * @brief Get field servo10_raw from servo_output_raw message
 *
 * @return [us] Servo output 10 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo10_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  23);
}

/**
 * @brief Get field servo11_raw from servo_output_raw message
 *
 * @return [us] Servo output 11 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo11_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  25);
}

/**
 * @brief Get field servo12_raw from servo_output_raw message
 *
 * @return [us] Servo output 12 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo12_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  27);
}

/**
 * @brief Get field servo13_raw from servo_output_raw message
 *
 * @return [us] Servo output 13 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo13_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  29);
}

/**
 * @brief Get field servo14_raw from servo_output_raw message
 *
 * @return [us] Servo output 14 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo14_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  31);
}

/**
 * @brief Get field servo15_raw from servo_output_raw message
 *
 * @return [us] Servo output 15 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo15_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  33);
}

/**
 * @brief Get field servo16_raw from servo_output_raw message
 *
 * @return [us] Servo output 16 value
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo16_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  35);
}

/**
 * @brief Decode a servo_output_raw message into a struct
 *
 * @param msg The message to decode
 * @param servo_output_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_servo_output_raw_decode(const mavlink_message_t* msg, mavlink_servo_output_raw_t* servo_output_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    servo_output_raw->time_usec = mavlink_msg_servo_output_raw_get_time_usec(msg);
    servo_output_raw->servo1_raw = mavlink_msg_servo_output_raw_get_servo1_raw(msg);
    servo_output_raw->servo2_raw = mavlink_msg_servo_output_raw_get_servo2_raw(msg);
    servo_output_raw->servo3_raw = mavlink_msg_servo_output_raw_get_servo3_raw(msg);
    servo_output_raw->servo4_raw = mavlink_msg_servo_output_raw_get_servo4_raw(msg);
    servo_output_raw->servo5_raw = mavlink_msg_servo_output_raw_get_servo5_raw(msg);
    servo_output_raw->servo6_raw = mavlink_msg_servo_output_raw_get_servo6_raw(msg);
    servo_output_raw->servo7_raw = mavlink_msg_servo_output_raw_get_servo7_raw(msg);
    servo_output_raw->servo8_raw = mavlink_msg_servo_output_raw_get_servo8_raw(msg);
    servo_output_raw->port = mavlink_msg_servo_output_raw_get_port(msg);
    servo_output_raw->servo9_raw = mavlink_msg_servo_output_raw_get_servo9_raw(msg);
    servo_output_raw->servo10_raw = mavlink_msg_servo_output_raw_get_servo10_raw(msg);
    servo_output_raw->servo11_raw = mavlink_msg_servo_output_raw_get_servo11_raw(msg);
    servo_output_raw->servo12_raw = mavlink_msg_servo_output_raw_get_servo12_raw(msg);
    servo_output_raw->servo13_raw = mavlink_msg_servo_output_raw_get_servo13_raw(msg);
    servo_output_raw->servo14_raw = mavlink_msg_servo_output_raw_get_servo14_raw(msg);
    servo_output_raw->servo15_raw = mavlink_msg_servo_output_raw_get_servo15_raw(msg);
    servo_output_raw->servo16_raw = mavlink_msg_servo_output_raw_get_servo16_raw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN? msg->len : MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN;
        memset(servo_output_raw, 0, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN);
    memcpy(servo_output_raw, _MAV_PAYLOAD(msg), len);
#endif
}

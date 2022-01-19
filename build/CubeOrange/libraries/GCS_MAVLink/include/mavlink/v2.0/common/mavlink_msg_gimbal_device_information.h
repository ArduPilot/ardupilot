#pragma once
// MESSAGE GIMBAL_DEVICE_INFORMATION PACKING

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION 283


typedef struct __mavlink_gimbal_device_information_t {
 uint64_t uid; /*<  UID of gimbal hardware (0 if unknown).*/
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 uint32_t firmware_version; /*<  Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).*/
 uint32_t hardware_version; /*<  Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).*/
 float roll_min; /*< [rad] Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)*/
 float roll_max; /*< [rad] Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)*/
 float pitch_min; /*< [rad] Minimum hardware pitch angle (positive: up, negative: down)*/
 float pitch_max; /*< [rad] Maximum hardware pitch angle (positive: up, negative: down)*/
 float yaw_min; /*< [rad] Minimum hardware yaw angle (positive: to the right, negative: to the left)*/
 float yaw_max; /*< [rad] Maximum hardware yaw angle (positive: to the right, negative: to the left)*/
 uint16_t cap_flags; /*<  Bitmap of gimbal capability flags.*/
 uint16_t custom_cap_flags; /*<  Bitmap for use for gimbal-specific capability flags.*/
 char vendor_name[32]; /*<  Name of the gimbal vendor.*/
 char model_name[32]; /*<  Name of the gimbal model.*/
 char custom_name[32]; /*<  Custom name of the gimbal given to it by the user.*/
} mavlink_gimbal_device_information_t;

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN 144
#define MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN 144
#define MAVLINK_MSG_ID_283_LEN 144
#define MAVLINK_MSG_ID_283_MIN_LEN 144

#define MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_CRC 74
#define MAVLINK_MSG_ID_283_CRC 74

#define MAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_VENDOR_NAME_LEN 32
#define MAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_MODEL_NAME_LEN 32
#define MAVLINK_MSG_GIMBAL_DEVICE_INFORMATION_FIELD_CUSTOM_NAME_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_DEVICE_INFORMATION { \
    283, \
    "GIMBAL_DEVICE_INFORMATION", \
    15, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gimbal_device_information_t, time_boot_ms) }, \
         { "vendor_name", NULL, MAVLINK_TYPE_CHAR, 32, 48, offsetof(mavlink_gimbal_device_information_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_CHAR, 32, 80, offsetof(mavlink_gimbal_device_information_t, model_name) }, \
         { "custom_name", NULL, MAVLINK_TYPE_CHAR, 32, 112, offsetof(mavlink_gimbal_device_information_t, custom_name) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_gimbal_device_information_t, firmware_version) }, \
         { "hardware_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_gimbal_device_information_t, hardware_version) }, \
         { "uid", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gimbal_device_information_t, uid) }, \
         { "cap_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_gimbal_device_information_t, cap_flags) }, \
         { "custom_cap_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 46, offsetof(mavlink_gimbal_device_information_t, custom_cap_flags) }, \
         { "roll_min", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_device_information_t, roll_min) }, \
         { "roll_max", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_device_information_t, roll_max) }, \
         { "pitch_min", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gimbal_device_information_t, pitch_min) }, \
         { "pitch_max", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gimbal_device_information_t, pitch_max) }, \
         { "yaw_min", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gimbal_device_information_t, yaw_min) }, \
         { "yaw_max", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gimbal_device_information_t, yaw_max) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_DEVICE_INFORMATION { \
    "GIMBAL_DEVICE_INFORMATION", \
    15, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gimbal_device_information_t, time_boot_ms) }, \
         { "vendor_name", NULL, MAVLINK_TYPE_CHAR, 32, 48, offsetof(mavlink_gimbal_device_information_t, vendor_name) }, \
         { "model_name", NULL, MAVLINK_TYPE_CHAR, 32, 80, offsetof(mavlink_gimbal_device_information_t, model_name) }, \
         { "custom_name", NULL, MAVLINK_TYPE_CHAR, 32, 112, offsetof(mavlink_gimbal_device_information_t, custom_name) }, \
         { "firmware_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_gimbal_device_information_t, firmware_version) }, \
         { "hardware_version", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_gimbal_device_information_t, hardware_version) }, \
         { "uid", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gimbal_device_information_t, uid) }, \
         { "cap_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_gimbal_device_information_t, cap_flags) }, \
         { "custom_cap_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 46, offsetof(mavlink_gimbal_device_information_t, custom_cap_flags) }, \
         { "roll_min", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gimbal_device_information_t, roll_min) }, \
         { "roll_max", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gimbal_device_information_t, roll_max) }, \
         { "pitch_min", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gimbal_device_information_t, pitch_min) }, \
         { "pitch_max", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_gimbal_device_information_t, pitch_max) }, \
         { "yaw_min", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_gimbal_device_information_t, yaw_min) }, \
         { "yaw_max", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_gimbal_device_information_t, yaw_max) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_device_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param vendor_name  Name of the gimbal vendor.
 * @param model_name  Name of the gimbal model.
 * @param custom_name  Custom name of the gimbal given to it by the user.
 * @param firmware_version  Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
 * @param hardware_version  Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
 * @param uid  UID of gimbal hardware (0 if unknown).
 * @param cap_flags  Bitmap of gimbal capability flags.
 * @param custom_cap_flags  Bitmap for use for gimbal-specific capability flags.
 * @param roll_min [rad] Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
 * @param roll_max [rad] Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
 * @param pitch_min [rad] Minimum hardware pitch angle (positive: up, negative: down)
 * @param pitch_max [rad] Maximum hardware pitch angle (positive: up, negative: down)
 * @param yaw_min [rad] Minimum hardware yaw angle (positive: to the right, negative: to the left)
 * @param yaw_max [rad] Maximum hardware yaw angle (positive: to the right, negative: to the left)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_device_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const char *vendor_name, const char *model_name, const char *custom_name, uint32_t firmware_version, uint32_t hardware_version, uint64_t uid, uint16_t cap_flags, uint16_t custom_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN];
    _mav_put_uint64_t(buf, 0, uid);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_uint32_t(buf, 12, firmware_version);
    _mav_put_uint32_t(buf, 16, hardware_version);
    _mav_put_float(buf, 20, roll_min);
    _mav_put_float(buf, 24, roll_max);
    _mav_put_float(buf, 28, pitch_min);
    _mav_put_float(buf, 32, pitch_max);
    _mav_put_float(buf, 36, yaw_min);
    _mav_put_float(buf, 40, yaw_max);
    _mav_put_uint16_t(buf, 44, cap_flags);
    _mav_put_uint16_t(buf, 46, custom_cap_flags);
    _mav_put_char_array(buf, 48, vendor_name, 32);
    _mav_put_char_array(buf, 80, model_name, 32);
    _mav_put_char_array(buf, 112, custom_name, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN);
#else
    mavlink_gimbal_device_information_t packet;
    packet.uid = uid;
    packet.time_boot_ms = time_boot_ms;
    packet.firmware_version = firmware_version;
    packet.hardware_version = hardware_version;
    packet.roll_min = roll_min;
    packet.roll_max = roll_max;
    packet.pitch_min = pitch_min;
    packet.pitch_max = pitch_max;
    packet.yaw_min = yaw_min;
    packet.yaw_max = yaw_max;
    packet.cap_flags = cap_flags;
    packet.custom_cap_flags = custom_cap_flags;
    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.custom_name, custom_name, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_CRC);
}

/**
 * @brief Pack a gimbal_device_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param vendor_name  Name of the gimbal vendor.
 * @param model_name  Name of the gimbal model.
 * @param custom_name  Custom name of the gimbal given to it by the user.
 * @param firmware_version  Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
 * @param hardware_version  Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
 * @param uid  UID of gimbal hardware (0 if unknown).
 * @param cap_flags  Bitmap of gimbal capability flags.
 * @param custom_cap_flags  Bitmap for use for gimbal-specific capability flags.
 * @param roll_min [rad] Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
 * @param roll_max [rad] Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
 * @param pitch_min [rad] Minimum hardware pitch angle (positive: up, negative: down)
 * @param pitch_max [rad] Maximum hardware pitch angle (positive: up, negative: down)
 * @param yaw_min [rad] Minimum hardware yaw angle (positive: to the right, negative: to the left)
 * @param yaw_max [rad] Maximum hardware yaw angle (positive: to the right, negative: to the left)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_device_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,const char *vendor_name,const char *model_name,const char *custom_name,uint32_t firmware_version,uint32_t hardware_version,uint64_t uid,uint16_t cap_flags,uint16_t custom_cap_flags,float roll_min,float roll_max,float pitch_min,float pitch_max,float yaw_min,float yaw_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN];
    _mav_put_uint64_t(buf, 0, uid);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_uint32_t(buf, 12, firmware_version);
    _mav_put_uint32_t(buf, 16, hardware_version);
    _mav_put_float(buf, 20, roll_min);
    _mav_put_float(buf, 24, roll_max);
    _mav_put_float(buf, 28, pitch_min);
    _mav_put_float(buf, 32, pitch_max);
    _mav_put_float(buf, 36, yaw_min);
    _mav_put_float(buf, 40, yaw_max);
    _mav_put_uint16_t(buf, 44, cap_flags);
    _mav_put_uint16_t(buf, 46, custom_cap_flags);
    _mav_put_char_array(buf, 48, vendor_name, 32);
    _mav_put_char_array(buf, 80, model_name, 32);
    _mav_put_char_array(buf, 112, custom_name, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN);
#else
    mavlink_gimbal_device_information_t packet;
    packet.uid = uid;
    packet.time_boot_ms = time_boot_ms;
    packet.firmware_version = firmware_version;
    packet.hardware_version = hardware_version;
    packet.roll_min = roll_min;
    packet.roll_max = roll_max;
    packet.pitch_min = pitch_min;
    packet.pitch_max = pitch_max;
    packet.yaw_min = yaw_min;
    packet.yaw_max = yaw_max;
    packet.cap_flags = cap_flags;
    packet.custom_cap_flags = custom_cap_flags;
    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.custom_name, custom_name, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_CRC);
}

/**
 * @brief Encode a gimbal_device_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_device_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_device_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_device_information_t* gimbal_device_information)
{
    return mavlink_msg_gimbal_device_information_pack(system_id, component_id, msg, gimbal_device_information->time_boot_ms, gimbal_device_information->vendor_name, gimbal_device_information->model_name, gimbal_device_information->custom_name, gimbal_device_information->firmware_version, gimbal_device_information->hardware_version, gimbal_device_information->uid, gimbal_device_information->cap_flags, gimbal_device_information->custom_cap_flags, gimbal_device_information->roll_min, gimbal_device_information->roll_max, gimbal_device_information->pitch_min, gimbal_device_information->pitch_max, gimbal_device_information->yaw_min, gimbal_device_information->yaw_max);
}

/**
 * @brief Encode a gimbal_device_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_device_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_device_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_device_information_t* gimbal_device_information)
{
    return mavlink_msg_gimbal_device_information_pack_chan(system_id, component_id, chan, msg, gimbal_device_information->time_boot_ms, gimbal_device_information->vendor_name, gimbal_device_information->model_name, gimbal_device_information->custom_name, gimbal_device_information->firmware_version, gimbal_device_information->hardware_version, gimbal_device_information->uid, gimbal_device_information->cap_flags, gimbal_device_information->custom_cap_flags, gimbal_device_information->roll_min, gimbal_device_information->roll_max, gimbal_device_information->pitch_min, gimbal_device_information->pitch_max, gimbal_device_information->yaw_min, gimbal_device_information->yaw_max);
}

/**
 * @brief Send a gimbal_device_information message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param vendor_name  Name of the gimbal vendor.
 * @param model_name  Name of the gimbal model.
 * @param custom_name  Custom name of the gimbal given to it by the user.
 * @param firmware_version  Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
 * @param hardware_version  Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
 * @param uid  UID of gimbal hardware (0 if unknown).
 * @param cap_flags  Bitmap of gimbal capability flags.
 * @param custom_cap_flags  Bitmap for use for gimbal-specific capability flags.
 * @param roll_min [rad] Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
 * @param roll_max [rad] Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
 * @param pitch_min [rad] Minimum hardware pitch angle (positive: up, negative: down)
 * @param pitch_max [rad] Maximum hardware pitch angle (positive: up, negative: down)
 * @param yaw_min [rad] Minimum hardware yaw angle (positive: to the right, negative: to the left)
 * @param yaw_max [rad] Maximum hardware yaw angle (positive: to the right, negative: to the left)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_device_information_send(mavlink_channel_t chan, uint32_t time_boot_ms, const char *vendor_name, const char *model_name, const char *custom_name, uint32_t firmware_version, uint32_t hardware_version, uint64_t uid, uint16_t cap_flags, uint16_t custom_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN];
    _mav_put_uint64_t(buf, 0, uid);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_uint32_t(buf, 12, firmware_version);
    _mav_put_uint32_t(buf, 16, hardware_version);
    _mav_put_float(buf, 20, roll_min);
    _mav_put_float(buf, 24, roll_max);
    _mav_put_float(buf, 28, pitch_min);
    _mav_put_float(buf, 32, pitch_max);
    _mav_put_float(buf, 36, yaw_min);
    _mav_put_float(buf, 40, yaw_max);
    _mav_put_uint16_t(buf, 44, cap_flags);
    _mav_put_uint16_t(buf, 46, custom_cap_flags);
    _mav_put_char_array(buf, 48, vendor_name, 32);
    _mav_put_char_array(buf, 80, model_name, 32);
    _mav_put_char_array(buf, 112, custom_name, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_CRC);
#else
    mavlink_gimbal_device_information_t packet;
    packet.uid = uid;
    packet.time_boot_ms = time_boot_ms;
    packet.firmware_version = firmware_version;
    packet.hardware_version = hardware_version;
    packet.roll_min = roll_min;
    packet.roll_max = roll_max;
    packet.pitch_min = pitch_min;
    packet.pitch_max = pitch_max;
    packet.yaw_min = yaw_min;
    packet.yaw_max = yaw_max;
    packet.cap_flags = cap_flags;
    packet.custom_cap_flags = custom_cap_flags;
    mav_array_memcpy(packet.vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet.model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet.custom_name, custom_name, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a gimbal_device_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_device_information_send_struct(mavlink_channel_t chan, const mavlink_gimbal_device_information_t* gimbal_device_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_device_information_send(chan, gimbal_device_information->time_boot_ms, gimbal_device_information->vendor_name, gimbal_device_information->model_name, gimbal_device_information->custom_name, gimbal_device_information->firmware_version, gimbal_device_information->hardware_version, gimbal_device_information->uid, gimbal_device_information->cap_flags, gimbal_device_information->custom_cap_flags, gimbal_device_information->roll_min, gimbal_device_information->roll_max, gimbal_device_information->pitch_min, gimbal_device_information->pitch_max, gimbal_device_information->yaw_min, gimbal_device_information->yaw_max);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, (const char *)gimbal_device_information, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_device_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const char *vendor_name, const char *model_name, const char *custom_name, uint32_t firmware_version, uint32_t hardware_version, uint64_t uid, uint16_t cap_flags, uint16_t custom_cap_flags, float roll_min, float roll_max, float pitch_min, float pitch_max, float yaw_min, float yaw_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, uid);
    _mav_put_uint32_t(buf, 8, time_boot_ms);
    _mav_put_uint32_t(buf, 12, firmware_version);
    _mav_put_uint32_t(buf, 16, hardware_version);
    _mav_put_float(buf, 20, roll_min);
    _mav_put_float(buf, 24, roll_max);
    _mav_put_float(buf, 28, pitch_min);
    _mav_put_float(buf, 32, pitch_max);
    _mav_put_float(buf, 36, yaw_min);
    _mav_put_float(buf, 40, yaw_max);
    _mav_put_uint16_t(buf, 44, cap_flags);
    _mav_put_uint16_t(buf, 46, custom_cap_flags);
    _mav_put_char_array(buf, 48, vendor_name, 32);
    _mav_put_char_array(buf, 80, model_name, 32);
    _mav_put_char_array(buf, 112, custom_name, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, buf, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_CRC);
#else
    mavlink_gimbal_device_information_t *packet = (mavlink_gimbal_device_information_t *)msgbuf;
    packet->uid = uid;
    packet->time_boot_ms = time_boot_ms;
    packet->firmware_version = firmware_version;
    packet->hardware_version = hardware_version;
    packet->roll_min = roll_min;
    packet->roll_max = roll_max;
    packet->pitch_min = pitch_min;
    packet->pitch_max = pitch_max;
    packet->yaw_min = yaw_min;
    packet->yaw_max = yaw_max;
    packet->cap_flags = cap_flags;
    packet->custom_cap_flags = custom_cap_flags;
    mav_array_memcpy(packet->vendor_name, vendor_name, sizeof(char)*32);
    mav_array_memcpy(packet->model_name, model_name, sizeof(char)*32);
    mav_array_memcpy(packet->custom_name, custom_name, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_DEVICE_INFORMATION UNPACKING


/**
 * @brief Get field time_boot_ms from gimbal_device_information message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_gimbal_device_information_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field vendor_name from gimbal_device_information message
 *
 * @return  Name of the gimbal vendor.
 */
static inline uint16_t mavlink_msg_gimbal_device_information_get_vendor_name(const mavlink_message_t* msg, char *vendor_name)
{
    return _MAV_RETURN_char_array(msg, vendor_name, 32,  48);
}

/**
 * @brief Get field model_name from gimbal_device_information message
 *
 * @return  Name of the gimbal model.
 */
static inline uint16_t mavlink_msg_gimbal_device_information_get_model_name(const mavlink_message_t* msg, char *model_name)
{
    return _MAV_RETURN_char_array(msg, model_name, 32,  80);
}

/**
 * @brief Get field custom_name from gimbal_device_information message
 *
 * @return  Custom name of the gimbal given to it by the user.
 */
static inline uint16_t mavlink_msg_gimbal_device_information_get_custom_name(const mavlink_message_t* msg, char *custom_name)
{
    return _MAV_RETURN_char_array(msg, custom_name, 32,  112);
}

/**
 * @brief Get field firmware_version from gimbal_device_information message
 *
 * @return  Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
 */
static inline uint32_t mavlink_msg_gimbal_device_information_get_firmware_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field hardware_version from gimbal_device_information message
 *
 * @return  Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
 */
static inline uint32_t mavlink_msg_gimbal_device_information_get_hardware_version(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field uid from gimbal_device_information message
 *
 * @return  UID of gimbal hardware (0 if unknown).
 */
static inline uint64_t mavlink_msg_gimbal_device_information_get_uid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field cap_flags from gimbal_device_information message
 *
 * @return  Bitmap of gimbal capability flags.
 */
static inline uint16_t mavlink_msg_gimbal_device_information_get_cap_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  44);
}

/**
 * @brief Get field custom_cap_flags from gimbal_device_information message
 *
 * @return  Bitmap for use for gimbal-specific capability flags.
 */
static inline uint16_t mavlink_msg_gimbal_device_information_get_custom_cap_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  46);
}

/**
 * @brief Get field roll_min from gimbal_device_information message
 *
 * @return [rad] Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
 */
static inline float mavlink_msg_gimbal_device_information_get_roll_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field roll_max from gimbal_device_information message
 *
 * @return [rad] Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
 */
static inline float mavlink_msg_gimbal_device_information_get_roll_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pitch_min from gimbal_device_information message
 *
 * @return [rad] Minimum hardware pitch angle (positive: up, negative: down)
 */
static inline float mavlink_msg_gimbal_device_information_get_pitch_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field pitch_max from gimbal_device_information message
 *
 * @return [rad] Maximum hardware pitch angle (positive: up, negative: down)
 */
static inline float mavlink_msg_gimbal_device_information_get_pitch_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field yaw_min from gimbal_device_information message
 *
 * @return [rad] Minimum hardware yaw angle (positive: to the right, negative: to the left)
 */
static inline float mavlink_msg_gimbal_device_information_get_yaw_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field yaw_max from gimbal_device_information message
 *
 * @return [rad] Maximum hardware yaw angle (positive: to the right, negative: to the left)
 */
static inline float mavlink_msg_gimbal_device_information_get_yaw_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a gimbal_device_information message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_device_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_device_information_decode(const mavlink_message_t* msg, mavlink_gimbal_device_information_t* gimbal_device_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_device_information->uid = mavlink_msg_gimbal_device_information_get_uid(msg);
    gimbal_device_information->time_boot_ms = mavlink_msg_gimbal_device_information_get_time_boot_ms(msg);
    gimbal_device_information->firmware_version = mavlink_msg_gimbal_device_information_get_firmware_version(msg);
    gimbal_device_information->hardware_version = mavlink_msg_gimbal_device_information_get_hardware_version(msg);
    gimbal_device_information->roll_min = mavlink_msg_gimbal_device_information_get_roll_min(msg);
    gimbal_device_information->roll_max = mavlink_msg_gimbal_device_information_get_roll_max(msg);
    gimbal_device_information->pitch_min = mavlink_msg_gimbal_device_information_get_pitch_min(msg);
    gimbal_device_information->pitch_max = mavlink_msg_gimbal_device_information_get_pitch_max(msg);
    gimbal_device_information->yaw_min = mavlink_msg_gimbal_device_information_get_yaw_min(msg);
    gimbal_device_information->yaw_max = mavlink_msg_gimbal_device_information_get_yaw_max(msg);
    gimbal_device_information->cap_flags = mavlink_msg_gimbal_device_information_get_cap_flags(msg);
    gimbal_device_information->custom_cap_flags = mavlink_msg_gimbal_device_information_get_custom_cap_flags(msg);
    mavlink_msg_gimbal_device_information_get_vendor_name(msg, gimbal_device_information->vendor_name);
    mavlink_msg_gimbal_device_information_get_model_name(msg, gimbal_device_information->model_name);
    mavlink_msg_gimbal_device_information_get_custom_name(msg, gimbal_device_information->custom_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN;
        memset(gimbal_device_information, 0, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION_LEN);
    memcpy(gimbal_device_information, _MAV_PAYLOAD(msg), len);
#endif
}

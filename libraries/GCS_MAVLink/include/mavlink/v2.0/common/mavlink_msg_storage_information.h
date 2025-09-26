#pragma once
// MESSAGE STORAGE_INFORMATION PACKING

#define MAVLINK_MSG_ID_STORAGE_INFORMATION 261


typedef struct __mavlink_storage_information_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float total_capacity; /*< [MiB] Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.*/
 float used_capacity; /*< [MiB] Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.*/
 float available_capacity; /*< [MiB] Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.*/
 float read_speed; /*< [MiB/s] Read speed.*/
 float write_speed; /*< [MiB/s] Write speed.*/
 uint8_t storage_id; /*<  Storage ID (1 for first, 2 for second, etc.)*/
 uint8_t storage_count; /*<  Number of storage devices*/
 uint8_t status; /*<  Status of storage*/
 uint8_t type; /*<  Type of storage*/
 char name[32]; /*<  Textual storage name to be used in UI (microSD 1, Internal Memory, etc.) This is a NULL terminated string. If it is exactly 32 characters long, add a terminating NULL. If this string is empty, the generic type is shown to the user.*/
} mavlink_storage_information_t;

#define MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN 60
#define MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN 27
#define MAVLINK_MSG_ID_261_LEN 60
#define MAVLINK_MSG_ID_261_MIN_LEN 27

#define MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC 179
#define MAVLINK_MSG_ID_261_CRC 179

#define MAVLINK_MSG_STORAGE_INFORMATION_FIELD_NAME_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STORAGE_INFORMATION { \
    261, \
    "STORAGE_INFORMATION", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_storage_information_t, time_boot_ms) }, \
         { "storage_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_storage_information_t, storage_id) }, \
         { "storage_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_storage_information_t, storage_count) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_storage_information_t, status) }, \
         { "total_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_storage_information_t, total_capacity) }, \
         { "used_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_storage_information_t, used_capacity) }, \
         { "available_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_storage_information_t, available_capacity) }, \
         { "read_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_storage_information_t, read_speed) }, \
         { "write_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_storage_information_t, write_speed) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_storage_information_t, type) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 32, 28, offsetof(mavlink_storage_information_t, name) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STORAGE_INFORMATION { \
    "STORAGE_INFORMATION", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_storage_information_t, time_boot_ms) }, \
         { "storage_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_storage_information_t, storage_id) }, \
         { "storage_count", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_storage_information_t, storage_count) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_storage_information_t, status) }, \
         { "total_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_storage_information_t, total_capacity) }, \
         { "used_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_storage_information_t, used_capacity) }, \
         { "available_capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_storage_information_t, available_capacity) }, \
         { "read_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_storage_information_t, read_speed) }, \
         { "write_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_storage_information_t, write_speed) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_storage_information_t, type) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 32, 28, offsetof(mavlink_storage_information_t, name) }, \
         } \
}
#endif

/**
 * @brief Pack a storage_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param storage_id  Storage ID (1 for first, 2 for second, etc.)
 * @param storage_count  Number of storage devices
 * @param status  Status of storage
 * @param total_capacity [MiB] Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param used_capacity [MiB] Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param available_capacity [MiB] Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param read_speed [MiB/s] Read speed.
 * @param write_speed [MiB/s] Write speed.
 * @param type  Type of storage
 * @param name  Textual storage name to be used in UI (microSD 1, Internal Memory, etc.) This is a NULL terminated string. If it is exactly 32 characters long, add a terminating NULL. If this string is empty, the generic type is shown to the user.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storage_information_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, total_capacity);
    _mav_put_float(buf, 8, used_capacity);
    _mav_put_float(buf, 12, available_capacity);
    _mav_put_float(buf, 16, read_speed);
    _mav_put_float(buf, 20, write_speed);
    _mav_put_uint8_t(buf, 24, storage_id);
    _mav_put_uint8_t(buf, 25, storage_count);
    _mav_put_uint8_t(buf, 26, status);
    _mav_put_uint8_t(buf, 27, type);
    _mav_put_char_array(buf, 28, name, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN);
#else
    mavlink_storage_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.total_capacity = total_capacity;
    packet.used_capacity = used_capacity;
    packet.available_capacity = available_capacity;
    packet.read_speed = read_speed;
    packet.write_speed = write_speed;
    packet.storage_id = storage_id;
    packet.storage_count = storage_count;
    packet.status = status;
    packet.type = type;
    mav_array_memcpy(packet.name, name, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORAGE_INFORMATION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC);
}

/**
 * @brief Pack a storage_information message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param storage_id  Storage ID (1 for first, 2 for second, etc.)
 * @param storage_count  Number of storage devices
 * @param status  Status of storage
 * @param total_capacity [MiB] Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param used_capacity [MiB] Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param available_capacity [MiB] Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param read_speed [MiB/s] Read speed.
 * @param write_speed [MiB/s] Write speed.
 * @param type  Type of storage
 * @param name  Textual storage name to be used in UI (microSD 1, Internal Memory, etc.) This is a NULL terminated string. If it is exactly 32 characters long, add a terminating NULL. If this string is empty, the generic type is shown to the user.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storage_information_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, total_capacity);
    _mav_put_float(buf, 8, used_capacity);
    _mav_put_float(buf, 12, available_capacity);
    _mav_put_float(buf, 16, read_speed);
    _mav_put_float(buf, 20, write_speed);
    _mav_put_uint8_t(buf, 24, storage_id);
    _mav_put_uint8_t(buf, 25, storage_count);
    _mav_put_uint8_t(buf, 26, status);
    _mav_put_uint8_t(buf, 27, type);
    _mav_put_char_array(buf, 28, name, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN);
#else
    mavlink_storage_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.total_capacity = total_capacity;
    packet.used_capacity = used_capacity;
    packet.available_capacity = available_capacity;
    packet.read_speed = read_speed;
    packet.write_speed = write_speed;
    packet.storage_id = storage_id;
    packet.storage_count = storage_count;
    packet.status = status;
    packet.type = type;
    mav_array_memcpy(packet.name, name, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORAGE_INFORMATION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN);
#endif
}

/**
 * @brief Pack a storage_information message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param storage_id  Storage ID (1 for first, 2 for second, etc.)
 * @param storage_count  Number of storage devices
 * @param status  Status of storage
 * @param total_capacity [MiB] Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param used_capacity [MiB] Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param available_capacity [MiB] Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param read_speed [MiB/s] Read speed.
 * @param write_speed [MiB/s] Write speed.
 * @param type  Type of storage
 * @param name  Textual storage name to be used in UI (microSD 1, Internal Memory, etc.) This is a NULL terminated string. If it is exactly 32 characters long, add a terminating NULL. If this string is empty, the generic type is shown to the user.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_storage_information_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t storage_id,uint8_t storage_count,uint8_t status,float total_capacity,float used_capacity,float available_capacity,float read_speed,float write_speed,uint8_t type,const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, total_capacity);
    _mav_put_float(buf, 8, used_capacity);
    _mav_put_float(buf, 12, available_capacity);
    _mav_put_float(buf, 16, read_speed);
    _mav_put_float(buf, 20, write_speed);
    _mav_put_uint8_t(buf, 24, storage_id);
    _mav_put_uint8_t(buf, 25, storage_count);
    _mav_put_uint8_t(buf, 26, status);
    _mav_put_uint8_t(buf, 27, type);
    _mav_put_char_array(buf, 28, name, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN);
#else
    mavlink_storage_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.total_capacity = total_capacity;
    packet.used_capacity = used_capacity;
    packet.available_capacity = available_capacity;
    packet.read_speed = read_speed;
    packet.write_speed = write_speed;
    packet.storage_id = storage_id;
    packet.storage_count = storage_count;
    packet.status = status;
    packet.type = type;
    mav_array_memcpy(packet.name, name, sizeof(char)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STORAGE_INFORMATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC);
}

/**
 * @brief Encode a storage_information struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param storage_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storage_information_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_storage_information_t* storage_information)
{
    return mavlink_msg_storage_information_pack(system_id, component_id, msg, storage_information->time_boot_ms, storage_information->storage_id, storage_information->storage_count, storage_information->status, storage_information->total_capacity, storage_information->used_capacity, storage_information->available_capacity, storage_information->read_speed, storage_information->write_speed, storage_information->type, storage_information->name);
}

/**
 * @brief Encode a storage_information struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param storage_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storage_information_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_storage_information_t* storage_information)
{
    return mavlink_msg_storage_information_pack_chan(system_id, component_id, chan, msg, storage_information->time_boot_ms, storage_information->storage_id, storage_information->storage_count, storage_information->status, storage_information->total_capacity, storage_information->used_capacity, storage_information->available_capacity, storage_information->read_speed, storage_information->write_speed, storage_information->type, storage_information->name);
}

/**
 * @brief Encode a storage_information struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param storage_information C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_storage_information_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_storage_information_t* storage_information)
{
    return mavlink_msg_storage_information_pack_status(system_id, component_id, _status, msg,  storage_information->time_boot_ms, storage_information->storage_id, storage_information->storage_count, storage_information->status, storage_information->total_capacity, storage_information->used_capacity, storage_information->available_capacity, storage_information->read_speed, storage_information->write_speed, storage_information->type, storage_information->name);
}

/**
 * @brief Send a storage_information message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param storage_id  Storage ID (1 for first, 2 for second, etc.)
 * @param storage_count  Number of storage devices
 * @param status  Status of storage
 * @param total_capacity [MiB] Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param used_capacity [MiB] Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param available_capacity [MiB] Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 * @param read_speed [MiB/s] Read speed.
 * @param write_speed [MiB/s] Write speed.
 * @param type  Type of storage
 * @param name  Textual storage name to be used in UI (microSD 1, Internal Memory, etc.) This is a NULL terminated string. If it is exactly 32 characters long, add a terminating NULL. If this string is empty, the generic type is shown to the user.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_storage_information_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, total_capacity);
    _mav_put_float(buf, 8, used_capacity);
    _mav_put_float(buf, 12, available_capacity);
    _mav_put_float(buf, 16, read_speed);
    _mav_put_float(buf, 20, write_speed);
    _mav_put_uint8_t(buf, 24, storage_id);
    _mav_put_uint8_t(buf, 25, storage_count);
    _mav_put_uint8_t(buf, 26, status);
    _mav_put_uint8_t(buf, 27, type);
    _mav_put_char_array(buf, 28, name, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORAGE_INFORMATION, buf, MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC);
#else
    mavlink_storage_information_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.total_capacity = total_capacity;
    packet.used_capacity = used_capacity;
    packet.available_capacity = available_capacity;
    packet.read_speed = read_speed;
    packet.write_speed = write_speed;
    packet.storage_id = storage_id;
    packet.storage_count = storage_count;
    packet.status = status;
    packet.type = type;
    mav_array_memcpy(packet.name, name, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORAGE_INFORMATION, (const char *)&packet, MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC);
#endif
}

/**
 * @brief Send a storage_information message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_storage_information_send_struct(mavlink_channel_t chan, const mavlink_storage_information_t* storage_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_storage_information_send(chan, storage_information->time_boot_ms, storage_information->storage_id, storage_information->storage_count, storage_information->status, storage_information->total_capacity, storage_information->used_capacity, storage_information->available_capacity, storage_information->read_speed, storage_information->write_speed, storage_information->type, storage_information->name);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORAGE_INFORMATION, (const char *)storage_information, MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC);
#endif
}

#if MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_storage_information_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t storage_id, uint8_t storage_count, uint8_t status, float total_capacity, float used_capacity, float available_capacity, float read_speed, float write_speed, uint8_t type, const char *name)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, total_capacity);
    _mav_put_float(buf, 8, used_capacity);
    _mav_put_float(buf, 12, available_capacity);
    _mav_put_float(buf, 16, read_speed);
    _mav_put_float(buf, 20, write_speed);
    _mav_put_uint8_t(buf, 24, storage_id);
    _mav_put_uint8_t(buf, 25, storage_count);
    _mav_put_uint8_t(buf, 26, status);
    _mav_put_uint8_t(buf, 27, type);
    _mav_put_char_array(buf, 28, name, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORAGE_INFORMATION, buf, MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC);
#else
    mavlink_storage_information_t *packet = (mavlink_storage_information_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->total_capacity = total_capacity;
    packet->used_capacity = used_capacity;
    packet->available_capacity = available_capacity;
    packet->read_speed = read_speed;
    packet->write_speed = write_speed;
    packet->storage_id = storage_id;
    packet->storage_count = storage_count;
    packet->status = status;
    packet->type = type;
    mav_array_memcpy(packet->name, name, sizeof(char)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STORAGE_INFORMATION, (const char *)packet, MAVLINK_MSG_ID_STORAGE_INFORMATION_MIN_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN, MAVLINK_MSG_ID_STORAGE_INFORMATION_CRC);
#endif
}
#endif

#endif

// MESSAGE STORAGE_INFORMATION UNPACKING


/**
 * @brief Get field time_boot_ms from storage_information message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_storage_information_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field storage_id from storage_information message
 *
 * @return  Storage ID (1 for first, 2 for second, etc.)
 */
static inline uint8_t mavlink_msg_storage_information_get_storage_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field storage_count from storage_information message
 *
 * @return  Number of storage devices
 */
static inline uint8_t mavlink_msg_storage_information_get_storage_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field status from storage_information message
 *
 * @return  Status of storage
 */
static inline uint8_t mavlink_msg_storage_information_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field total_capacity from storage_information message
 *
 * @return [MiB] Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 */
static inline float mavlink_msg_storage_information_get_total_capacity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field used_capacity from storage_information message
 *
 * @return [MiB] Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 */
static inline float mavlink_msg_storage_information_get_used_capacity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field available_capacity from storage_information message
 *
 * @return [MiB] Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
 */
static inline float mavlink_msg_storage_information_get_available_capacity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field read_speed from storage_information message
 *
 * @return [MiB/s] Read speed.
 */
static inline float mavlink_msg_storage_information_get_read_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field write_speed from storage_information message
 *
 * @return [MiB/s] Write speed.
 */
static inline float mavlink_msg_storage_information_get_write_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field type from storage_information message
 *
 * @return  Type of storage
 */
static inline uint8_t mavlink_msg_storage_information_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field name from storage_information message
 *
 * @return  Textual storage name to be used in UI (microSD 1, Internal Memory, etc.) This is a NULL terminated string. If it is exactly 32 characters long, add a terminating NULL. If this string is empty, the generic type is shown to the user.
 */
static inline uint16_t mavlink_msg_storage_information_get_name(const mavlink_message_t* msg, char *name)
{
    return _MAV_RETURN_char_array(msg, name, 32,  28);
}

/**
 * @brief Decode a storage_information message into a struct
 *
 * @param msg The message to decode
 * @param storage_information C-struct to decode the message contents into
 */
static inline void mavlink_msg_storage_information_decode(const mavlink_message_t* msg, mavlink_storage_information_t* storage_information)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    storage_information->time_boot_ms = mavlink_msg_storage_information_get_time_boot_ms(msg);
    storage_information->total_capacity = mavlink_msg_storage_information_get_total_capacity(msg);
    storage_information->used_capacity = mavlink_msg_storage_information_get_used_capacity(msg);
    storage_information->available_capacity = mavlink_msg_storage_information_get_available_capacity(msg);
    storage_information->read_speed = mavlink_msg_storage_information_get_read_speed(msg);
    storage_information->write_speed = mavlink_msg_storage_information_get_write_speed(msg);
    storage_information->storage_id = mavlink_msg_storage_information_get_storage_id(msg);
    storage_information->storage_count = mavlink_msg_storage_information_get_storage_count(msg);
    storage_information->status = mavlink_msg_storage_information_get_status(msg);
    storage_information->type = mavlink_msg_storage_information_get_type(msg);
    mavlink_msg_storage_information_get_name(msg, storage_information->name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN? msg->len : MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN;
        memset(storage_information, 0, MAVLINK_MSG_ID_STORAGE_INFORMATION_LEN);
    memcpy(storage_information, _MAV_PAYLOAD(msg), len);
#endif
}

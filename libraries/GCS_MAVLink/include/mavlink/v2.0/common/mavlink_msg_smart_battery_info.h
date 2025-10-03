#pragma once
// MESSAGE SMART_BATTERY_INFO PACKING

#define MAVLINK_MSG_ID_SMART_BATTERY_INFO 370

MAVPACKED(
typedef struct __mavlink_smart_battery_info_t {
 int32_t capacity_full_specification; /*< [mAh] Capacity when full according to manufacturer, -1: field not provided.*/
 int32_t capacity_full; /*< [mAh] Capacity when full (accounting for battery degradation), -1: field not provided.*/
 uint16_t cycle_count; /*<  Charge/discharge cycle count. UINT16_MAX: field not provided.*/
 uint16_t weight; /*< [g] Battery weight. 0: field not provided.*/
 uint16_t discharge_minimum_voltage; /*< [mV] Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.*/
 uint16_t charging_minimum_voltage; /*< [mV] Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.*/
 uint16_t resting_minimum_voltage; /*< [mV] Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.*/
 uint8_t id; /*<  Battery ID*/
 uint8_t battery_function; /*<  Function of the battery*/
 uint8_t type; /*<  Type (chemistry) of the battery*/
 char serial_number[16]; /*<  Serial number in ASCII characters, 0 terminated. All 0: field not provided.*/
 char device_name[50]; /*<  Static device name in ASCII characters, 0 terminated. All 0: field not provided. Encode as manufacturer name then product name separated using an underscore.*/
 uint16_t charging_maximum_voltage; /*< [mV] Maximum per-cell voltage when charged. 0: field not provided.*/
 uint8_t cells_in_series; /*<  Number of battery cells in series. 0: field not provided.*/
 uint32_t discharge_maximum_current; /*< [mA] Maximum pack discharge current. 0: field not provided.*/
 uint32_t discharge_maximum_burst_current; /*< [mA] Maximum pack discharge burst current. 0: field not provided.*/
 char manufacture_date[11]; /*<  Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. All 0: field not provided.*/
}) mavlink_smart_battery_info_t;

#define MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN 109
#define MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN 87
#define MAVLINK_MSG_ID_370_LEN 109
#define MAVLINK_MSG_ID_370_MIN_LEN 87

#define MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC 75
#define MAVLINK_MSG_ID_370_CRC 75

#define MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_SERIAL_NUMBER_LEN 16
#define MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_DEVICE_NAME_LEN 50
#define MAVLINK_MSG_SMART_BATTERY_INFO_FIELD_MANUFACTURE_DATE_LEN 11

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SMART_BATTERY_INFO { \
    370, \
    "SMART_BATTERY_INFO", \
    17, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_smart_battery_info_t, id) }, \
         { "battery_function", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_smart_battery_info_t, battery_function) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_smart_battery_info_t, type) }, \
         { "capacity_full_specification", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_smart_battery_info_t, capacity_full_specification) }, \
         { "capacity_full", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_smart_battery_info_t, capacity_full) }, \
         { "cycle_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_smart_battery_info_t, cycle_count) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 16, 21, offsetof(mavlink_smart_battery_info_t, serial_number) }, \
         { "device_name", NULL, MAVLINK_TYPE_CHAR, 50, 37, offsetof(mavlink_smart_battery_info_t, device_name) }, \
         { "weight", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_smart_battery_info_t, weight) }, \
         { "discharge_minimum_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_smart_battery_info_t, discharge_minimum_voltage) }, \
         { "charging_minimum_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_smart_battery_info_t, charging_minimum_voltage) }, \
         { "resting_minimum_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_smart_battery_info_t, resting_minimum_voltage) }, \
         { "charging_maximum_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 87, offsetof(mavlink_smart_battery_info_t, charging_maximum_voltage) }, \
         { "cells_in_series", NULL, MAVLINK_TYPE_UINT8_T, 0, 89, offsetof(mavlink_smart_battery_info_t, cells_in_series) }, \
         { "discharge_maximum_current", NULL, MAVLINK_TYPE_UINT32_T, 0, 90, offsetof(mavlink_smart_battery_info_t, discharge_maximum_current) }, \
         { "discharge_maximum_burst_current", NULL, MAVLINK_TYPE_UINT32_T, 0, 94, offsetof(mavlink_smart_battery_info_t, discharge_maximum_burst_current) }, \
         { "manufacture_date", NULL, MAVLINK_TYPE_CHAR, 11, 98, offsetof(mavlink_smart_battery_info_t, manufacture_date) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SMART_BATTERY_INFO { \
    "SMART_BATTERY_INFO", \
    17, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_smart_battery_info_t, id) }, \
         { "battery_function", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_smart_battery_info_t, battery_function) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_smart_battery_info_t, type) }, \
         { "capacity_full_specification", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_smart_battery_info_t, capacity_full_specification) }, \
         { "capacity_full", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_smart_battery_info_t, capacity_full) }, \
         { "cycle_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_smart_battery_info_t, cycle_count) }, \
         { "serial_number", NULL, MAVLINK_TYPE_CHAR, 16, 21, offsetof(mavlink_smart_battery_info_t, serial_number) }, \
         { "device_name", NULL, MAVLINK_TYPE_CHAR, 50, 37, offsetof(mavlink_smart_battery_info_t, device_name) }, \
         { "weight", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_smart_battery_info_t, weight) }, \
         { "discharge_minimum_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_smart_battery_info_t, discharge_minimum_voltage) }, \
         { "charging_minimum_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_smart_battery_info_t, charging_minimum_voltage) }, \
         { "resting_minimum_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_smart_battery_info_t, resting_minimum_voltage) }, \
         { "charging_maximum_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 87, offsetof(mavlink_smart_battery_info_t, charging_maximum_voltage) }, \
         { "cells_in_series", NULL, MAVLINK_TYPE_UINT8_T, 0, 89, offsetof(mavlink_smart_battery_info_t, cells_in_series) }, \
         { "discharge_maximum_current", NULL, MAVLINK_TYPE_UINT32_T, 0, 90, offsetof(mavlink_smart_battery_info_t, discharge_maximum_current) }, \
         { "discharge_maximum_burst_current", NULL, MAVLINK_TYPE_UINT32_T, 0, 94, offsetof(mavlink_smart_battery_info_t, discharge_maximum_burst_current) }, \
         { "manufacture_date", NULL, MAVLINK_TYPE_CHAR, 11, 98, offsetof(mavlink_smart_battery_info_t, manufacture_date) }, \
         } \
}
#endif

/**
 * @brief Pack a smart_battery_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Battery ID
 * @param battery_function  Function of the battery
 * @param type  Type (chemistry) of the battery
 * @param capacity_full_specification [mAh] Capacity when full according to manufacturer, -1: field not provided.
 * @param capacity_full [mAh] Capacity when full (accounting for battery degradation), -1: field not provided.
 * @param cycle_count  Charge/discharge cycle count. UINT16_MAX: field not provided.
 * @param serial_number  Serial number in ASCII characters, 0 terminated. All 0: field not provided.
 * @param device_name  Static device name in ASCII characters, 0 terminated. All 0: field not provided. Encode as manufacturer name then product name separated using an underscore.
 * @param weight [g] Battery weight. 0: field not provided.
 * @param discharge_minimum_voltage [mV] Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
 * @param charging_minimum_voltage [mV] Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
 * @param resting_minimum_voltage [mV] Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
 * @param charging_maximum_voltage [mV] Maximum per-cell voltage when charged. 0: field not provided.
 * @param cells_in_series  Number of battery cells in series. 0: field not provided.
 * @param discharge_maximum_current [mA] Maximum pack discharge current. 0: field not provided.
 * @param discharge_maximum_burst_current [mA] Maximum pack discharge burst current. 0: field not provided.
 * @param manufacture_date  Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. All 0: field not provided.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_smart_battery_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char *serial_number, const char *device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage, uint16_t charging_maximum_voltage, uint8_t cells_in_series, uint32_t discharge_maximum_current, uint32_t discharge_maximum_burst_current, const char *manufacture_date)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN];
    _mav_put_int32_t(buf, 0, capacity_full_specification);
    _mav_put_int32_t(buf, 4, capacity_full);
    _mav_put_uint16_t(buf, 8, cycle_count);
    _mav_put_uint16_t(buf, 10, weight);
    _mav_put_uint16_t(buf, 12, discharge_minimum_voltage);
    _mav_put_uint16_t(buf, 14, charging_minimum_voltage);
    _mav_put_uint16_t(buf, 16, resting_minimum_voltage);
    _mav_put_uint8_t(buf, 18, id);
    _mav_put_uint8_t(buf, 19, battery_function);
    _mav_put_uint8_t(buf, 20, type);
    _mav_put_uint16_t(buf, 87, charging_maximum_voltage);
    _mav_put_uint8_t(buf, 89, cells_in_series);
    _mav_put_uint32_t(buf, 90, discharge_maximum_current);
    _mav_put_uint32_t(buf, 94, discharge_maximum_burst_current);
    _mav_put_char_array(buf, 21, serial_number, 16);
    _mav_put_char_array(buf, 37, device_name, 50);
    _mav_put_char_array(buf, 98, manufacture_date, 11);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN);
#else
    mavlink_smart_battery_info_t packet;
    packet.capacity_full_specification = capacity_full_specification;
    packet.capacity_full = capacity_full;
    packet.cycle_count = cycle_count;
    packet.weight = weight;
    packet.discharge_minimum_voltage = discharge_minimum_voltage;
    packet.charging_minimum_voltage = charging_minimum_voltage;
    packet.resting_minimum_voltage = resting_minimum_voltage;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.charging_maximum_voltage = charging_maximum_voltage;
    packet.cells_in_series = cells_in_series;
    packet.discharge_maximum_current = discharge_maximum_current;
    packet.discharge_maximum_burst_current = discharge_maximum_burst_current;
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*16);
    mav_array_memcpy(packet.device_name, device_name, sizeof(char)*50);
    mav_array_memcpy(packet.manufacture_date, manufacture_date, sizeof(char)*11);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SMART_BATTERY_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC);
}

/**
 * @brief Pack a smart_battery_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Battery ID
 * @param battery_function  Function of the battery
 * @param type  Type (chemistry) of the battery
 * @param capacity_full_specification [mAh] Capacity when full according to manufacturer, -1: field not provided.
 * @param capacity_full [mAh] Capacity when full (accounting for battery degradation), -1: field not provided.
 * @param cycle_count  Charge/discharge cycle count. UINT16_MAX: field not provided.
 * @param serial_number  Serial number in ASCII characters, 0 terminated. All 0: field not provided.
 * @param device_name  Static device name in ASCII characters, 0 terminated. All 0: field not provided. Encode as manufacturer name then product name separated using an underscore.
 * @param weight [g] Battery weight. 0: field not provided.
 * @param discharge_minimum_voltage [mV] Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
 * @param charging_minimum_voltage [mV] Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
 * @param resting_minimum_voltage [mV] Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
 * @param charging_maximum_voltage [mV] Maximum per-cell voltage when charged. 0: field not provided.
 * @param cells_in_series  Number of battery cells in series. 0: field not provided.
 * @param discharge_maximum_current [mA] Maximum pack discharge current. 0: field not provided.
 * @param discharge_maximum_burst_current [mA] Maximum pack discharge burst current. 0: field not provided.
 * @param manufacture_date  Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. All 0: field not provided.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_smart_battery_info_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char *serial_number, const char *device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage, uint16_t charging_maximum_voltage, uint8_t cells_in_series, uint32_t discharge_maximum_current, uint32_t discharge_maximum_burst_current, const char *manufacture_date)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN];
    _mav_put_int32_t(buf, 0, capacity_full_specification);
    _mav_put_int32_t(buf, 4, capacity_full);
    _mav_put_uint16_t(buf, 8, cycle_count);
    _mav_put_uint16_t(buf, 10, weight);
    _mav_put_uint16_t(buf, 12, discharge_minimum_voltage);
    _mav_put_uint16_t(buf, 14, charging_minimum_voltage);
    _mav_put_uint16_t(buf, 16, resting_minimum_voltage);
    _mav_put_uint8_t(buf, 18, id);
    _mav_put_uint8_t(buf, 19, battery_function);
    _mav_put_uint8_t(buf, 20, type);
    _mav_put_uint16_t(buf, 87, charging_maximum_voltage);
    _mav_put_uint8_t(buf, 89, cells_in_series);
    _mav_put_uint32_t(buf, 90, discharge_maximum_current);
    _mav_put_uint32_t(buf, 94, discharge_maximum_burst_current);
    _mav_put_char_array(buf, 21, serial_number, 16);
    _mav_put_char_array(buf, 37, device_name, 50);
    _mav_put_char_array(buf, 98, manufacture_date, 11);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN);
#else
    mavlink_smart_battery_info_t packet;
    packet.capacity_full_specification = capacity_full_specification;
    packet.capacity_full = capacity_full;
    packet.cycle_count = cycle_count;
    packet.weight = weight;
    packet.discharge_minimum_voltage = discharge_minimum_voltage;
    packet.charging_minimum_voltage = charging_minimum_voltage;
    packet.resting_minimum_voltage = resting_minimum_voltage;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.charging_maximum_voltage = charging_maximum_voltage;
    packet.cells_in_series = cells_in_series;
    packet.discharge_maximum_current = discharge_maximum_current;
    packet.discharge_maximum_burst_current = discharge_maximum_burst_current;
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*16);
    mav_array_memcpy(packet.device_name, device_name, sizeof(char)*50);
    mav_array_memcpy(packet.manufacture_date, manufacture_date, sizeof(char)*11);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SMART_BATTERY_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN);
#endif
}

/**
 * @brief Pack a smart_battery_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  Battery ID
 * @param battery_function  Function of the battery
 * @param type  Type (chemistry) of the battery
 * @param capacity_full_specification [mAh] Capacity when full according to manufacturer, -1: field not provided.
 * @param capacity_full [mAh] Capacity when full (accounting for battery degradation), -1: field not provided.
 * @param cycle_count  Charge/discharge cycle count. UINT16_MAX: field not provided.
 * @param serial_number  Serial number in ASCII characters, 0 terminated. All 0: field not provided.
 * @param device_name  Static device name in ASCII characters, 0 terminated. All 0: field not provided. Encode as manufacturer name then product name separated using an underscore.
 * @param weight [g] Battery weight. 0: field not provided.
 * @param discharge_minimum_voltage [mV] Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
 * @param charging_minimum_voltage [mV] Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
 * @param resting_minimum_voltage [mV] Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
 * @param charging_maximum_voltage [mV] Maximum per-cell voltage when charged. 0: field not provided.
 * @param cells_in_series  Number of battery cells in series. 0: field not provided.
 * @param discharge_maximum_current [mA] Maximum pack discharge current. 0: field not provided.
 * @param discharge_maximum_burst_current [mA] Maximum pack discharge burst current. 0: field not provided.
 * @param manufacture_date  Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. All 0: field not provided.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_smart_battery_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint8_t battery_function,uint8_t type,int32_t capacity_full_specification,int32_t capacity_full,uint16_t cycle_count,const char *serial_number,const char *device_name,uint16_t weight,uint16_t discharge_minimum_voltage,uint16_t charging_minimum_voltage,uint16_t resting_minimum_voltage,uint16_t charging_maximum_voltage,uint8_t cells_in_series,uint32_t discharge_maximum_current,uint32_t discharge_maximum_burst_current,const char *manufacture_date)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN];
    _mav_put_int32_t(buf, 0, capacity_full_specification);
    _mav_put_int32_t(buf, 4, capacity_full);
    _mav_put_uint16_t(buf, 8, cycle_count);
    _mav_put_uint16_t(buf, 10, weight);
    _mav_put_uint16_t(buf, 12, discharge_minimum_voltage);
    _mav_put_uint16_t(buf, 14, charging_minimum_voltage);
    _mav_put_uint16_t(buf, 16, resting_minimum_voltage);
    _mav_put_uint8_t(buf, 18, id);
    _mav_put_uint8_t(buf, 19, battery_function);
    _mav_put_uint8_t(buf, 20, type);
    _mav_put_uint16_t(buf, 87, charging_maximum_voltage);
    _mav_put_uint8_t(buf, 89, cells_in_series);
    _mav_put_uint32_t(buf, 90, discharge_maximum_current);
    _mav_put_uint32_t(buf, 94, discharge_maximum_burst_current);
    _mav_put_char_array(buf, 21, serial_number, 16);
    _mav_put_char_array(buf, 37, device_name, 50);
    _mav_put_char_array(buf, 98, manufacture_date, 11);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN);
#else
    mavlink_smart_battery_info_t packet;
    packet.capacity_full_specification = capacity_full_specification;
    packet.capacity_full = capacity_full;
    packet.cycle_count = cycle_count;
    packet.weight = weight;
    packet.discharge_minimum_voltage = discharge_minimum_voltage;
    packet.charging_minimum_voltage = charging_minimum_voltage;
    packet.resting_minimum_voltage = resting_minimum_voltage;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.charging_maximum_voltage = charging_maximum_voltage;
    packet.cells_in_series = cells_in_series;
    packet.discharge_maximum_current = discharge_maximum_current;
    packet.discharge_maximum_burst_current = discharge_maximum_burst_current;
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*16);
    mav_array_memcpy(packet.device_name, device_name, sizeof(char)*50);
    mav_array_memcpy(packet.manufacture_date, manufacture_date, sizeof(char)*11);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SMART_BATTERY_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC);
}

/**
 * @brief Encode a smart_battery_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param smart_battery_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_smart_battery_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_smart_battery_info_t* smart_battery_info)
{
    return mavlink_msg_smart_battery_info_pack(system_id, component_id, msg, smart_battery_info->id, smart_battery_info->battery_function, smart_battery_info->type, smart_battery_info->capacity_full_specification, smart_battery_info->capacity_full, smart_battery_info->cycle_count, smart_battery_info->serial_number, smart_battery_info->device_name, smart_battery_info->weight, smart_battery_info->discharge_minimum_voltage, smart_battery_info->charging_minimum_voltage, smart_battery_info->resting_minimum_voltage, smart_battery_info->charging_maximum_voltage, smart_battery_info->cells_in_series, smart_battery_info->discharge_maximum_current, smart_battery_info->discharge_maximum_burst_current, smart_battery_info->manufacture_date);
}

/**
 * @brief Encode a smart_battery_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param smart_battery_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_smart_battery_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_smart_battery_info_t* smart_battery_info)
{
    return mavlink_msg_smart_battery_info_pack_chan(system_id, component_id, chan, msg, smart_battery_info->id, smart_battery_info->battery_function, smart_battery_info->type, smart_battery_info->capacity_full_specification, smart_battery_info->capacity_full, smart_battery_info->cycle_count, smart_battery_info->serial_number, smart_battery_info->device_name, smart_battery_info->weight, smart_battery_info->discharge_minimum_voltage, smart_battery_info->charging_minimum_voltage, smart_battery_info->resting_minimum_voltage, smart_battery_info->charging_maximum_voltage, smart_battery_info->cells_in_series, smart_battery_info->discharge_maximum_current, smart_battery_info->discharge_maximum_burst_current, smart_battery_info->manufacture_date);
}

/**
 * @brief Encode a smart_battery_info struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param smart_battery_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_smart_battery_info_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_smart_battery_info_t* smart_battery_info)
{
    return mavlink_msg_smart_battery_info_pack_status(system_id, component_id, _status, msg,  smart_battery_info->id, smart_battery_info->battery_function, smart_battery_info->type, smart_battery_info->capacity_full_specification, smart_battery_info->capacity_full, smart_battery_info->cycle_count, smart_battery_info->serial_number, smart_battery_info->device_name, smart_battery_info->weight, smart_battery_info->discharge_minimum_voltage, smart_battery_info->charging_minimum_voltage, smart_battery_info->resting_minimum_voltage, smart_battery_info->charging_maximum_voltage, smart_battery_info->cells_in_series, smart_battery_info->discharge_maximum_current, smart_battery_info->discharge_maximum_burst_current, smart_battery_info->manufacture_date);
}

/**
 * @brief Send a smart_battery_info message
 * @param chan MAVLink channel to send the message
 *
 * @param id  Battery ID
 * @param battery_function  Function of the battery
 * @param type  Type (chemistry) of the battery
 * @param capacity_full_specification [mAh] Capacity when full according to manufacturer, -1: field not provided.
 * @param capacity_full [mAh] Capacity when full (accounting for battery degradation), -1: field not provided.
 * @param cycle_count  Charge/discharge cycle count. UINT16_MAX: field not provided.
 * @param serial_number  Serial number in ASCII characters, 0 terminated. All 0: field not provided.
 * @param device_name  Static device name in ASCII characters, 0 terminated. All 0: field not provided. Encode as manufacturer name then product name separated using an underscore.
 * @param weight [g] Battery weight. 0: field not provided.
 * @param discharge_minimum_voltage [mV] Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
 * @param charging_minimum_voltage [mV] Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
 * @param resting_minimum_voltage [mV] Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
 * @param charging_maximum_voltage [mV] Maximum per-cell voltage when charged. 0: field not provided.
 * @param cells_in_series  Number of battery cells in series. 0: field not provided.
 * @param discharge_maximum_current [mA] Maximum pack discharge current. 0: field not provided.
 * @param discharge_maximum_burst_current [mA] Maximum pack discharge burst current. 0: field not provided.
 * @param manufacture_date  Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. All 0: field not provided.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_smart_battery_info_send(mavlink_channel_t chan, uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char *serial_number, const char *device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage, uint16_t charging_maximum_voltage, uint8_t cells_in_series, uint32_t discharge_maximum_current, uint32_t discharge_maximum_burst_current, const char *manufacture_date)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN];
    _mav_put_int32_t(buf, 0, capacity_full_specification);
    _mav_put_int32_t(buf, 4, capacity_full);
    _mav_put_uint16_t(buf, 8, cycle_count);
    _mav_put_uint16_t(buf, 10, weight);
    _mav_put_uint16_t(buf, 12, discharge_minimum_voltage);
    _mav_put_uint16_t(buf, 14, charging_minimum_voltage);
    _mav_put_uint16_t(buf, 16, resting_minimum_voltage);
    _mav_put_uint8_t(buf, 18, id);
    _mav_put_uint8_t(buf, 19, battery_function);
    _mav_put_uint8_t(buf, 20, type);
    _mav_put_uint16_t(buf, 87, charging_maximum_voltage);
    _mav_put_uint8_t(buf, 89, cells_in_series);
    _mav_put_uint32_t(buf, 90, discharge_maximum_current);
    _mav_put_uint32_t(buf, 94, discharge_maximum_burst_current);
    _mav_put_char_array(buf, 21, serial_number, 16);
    _mav_put_char_array(buf, 37, device_name, 50);
    _mav_put_char_array(buf, 98, manufacture_date, 11);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMART_BATTERY_INFO, buf, MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC);
#else
    mavlink_smart_battery_info_t packet;
    packet.capacity_full_specification = capacity_full_specification;
    packet.capacity_full = capacity_full;
    packet.cycle_count = cycle_count;
    packet.weight = weight;
    packet.discharge_minimum_voltage = discharge_minimum_voltage;
    packet.charging_minimum_voltage = charging_minimum_voltage;
    packet.resting_minimum_voltage = resting_minimum_voltage;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.charging_maximum_voltage = charging_maximum_voltage;
    packet.cells_in_series = cells_in_series;
    packet.discharge_maximum_current = discharge_maximum_current;
    packet.discharge_maximum_burst_current = discharge_maximum_burst_current;
    mav_array_memcpy(packet.serial_number, serial_number, sizeof(char)*16);
    mav_array_memcpy(packet.device_name, device_name, sizeof(char)*50);
    mav_array_memcpy(packet.manufacture_date, manufacture_date, sizeof(char)*11);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMART_BATTERY_INFO, (const char *)&packet, MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC);
#endif
}

/**
 * @brief Send a smart_battery_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_smart_battery_info_send_struct(mavlink_channel_t chan, const mavlink_smart_battery_info_t* smart_battery_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_smart_battery_info_send(chan, smart_battery_info->id, smart_battery_info->battery_function, smart_battery_info->type, smart_battery_info->capacity_full_specification, smart_battery_info->capacity_full, smart_battery_info->cycle_count, smart_battery_info->serial_number, smart_battery_info->device_name, smart_battery_info->weight, smart_battery_info->discharge_minimum_voltage, smart_battery_info->charging_minimum_voltage, smart_battery_info->resting_minimum_voltage, smart_battery_info->charging_maximum_voltage, smart_battery_info->cells_in_series, smart_battery_info->discharge_maximum_current, smart_battery_info->discharge_maximum_burst_current, smart_battery_info->manufacture_date);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMART_BATTERY_INFO, (const char *)smart_battery_info, MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_smart_battery_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint8_t battery_function, uint8_t type, int32_t capacity_full_specification, int32_t capacity_full, uint16_t cycle_count, const char *serial_number, const char *device_name, uint16_t weight, uint16_t discharge_minimum_voltage, uint16_t charging_minimum_voltage, uint16_t resting_minimum_voltage, uint16_t charging_maximum_voltage, uint8_t cells_in_series, uint32_t discharge_maximum_current, uint32_t discharge_maximum_burst_current, const char *manufacture_date)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, capacity_full_specification);
    _mav_put_int32_t(buf, 4, capacity_full);
    _mav_put_uint16_t(buf, 8, cycle_count);
    _mav_put_uint16_t(buf, 10, weight);
    _mav_put_uint16_t(buf, 12, discharge_minimum_voltage);
    _mav_put_uint16_t(buf, 14, charging_minimum_voltage);
    _mav_put_uint16_t(buf, 16, resting_minimum_voltage);
    _mav_put_uint8_t(buf, 18, id);
    _mav_put_uint8_t(buf, 19, battery_function);
    _mav_put_uint8_t(buf, 20, type);
    _mav_put_uint16_t(buf, 87, charging_maximum_voltage);
    _mav_put_uint8_t(buf, 89, cells_in_series);
    _mav_put_uint32_t(buf, 90, discharge_maximum_current);
    _mav_put_uint32_t(buf, 94, discharge_maximum_burst_current);
    _mav_put_char_array(buf, 21, serial_number, 16);
    _mav_put_char_array(buf, 37, device_name, 50);
    _mav_put_char_array(buf, 98, manufacture_date, 11);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMART_BATTERY_INFO, buf, MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC);
#else
    mavlink_smart_battery_info_t *packet = (mavlink_smart_battery_info_t *)msgbuf;
    packet->capacity_full_specification = capacity_full_specification;
    packet->capacity_full = capacity_full;
    packet->cycle_count = cycle_count;
    packet->weight = weight;
    packet->discharge_minimum_voltage = discharge_minimum_voltage;
    packet->charging_minimum_voltage = charging_minimum_voltage;
    packet->resting_minimum_voltage = resting_minimum_voltage;
    packet->id = id;
    packet->battery_function = battery_function;
    packet->type = type;
    packet->charging_maximum_voltage = charging_maximum_voltage;
    packet->cells_in_series = cells_in_series;
    packet->discharge_maximum_current = discharge_maximum_current;
    packet->discharge_maximum_burst_current = discharge_maximum_burst_current;
    mav_array_memcpy(packet->serial_number, serial_number, sizeof(char)*16);
    mav_array_memcpy(packet->device_name, device_name, sizeof(char)*50);
    mav_array_memcpy(packet->manufacture_date, manufacture_date, sizeof(char)*11);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SMART_BATTERY_INFO, (const char *)packet, MAVLINK_MSG_ID_SMART_BATTERY_INFO_MIN_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN, MAVLINK_MSG_ID_SMART_BATTERY_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE SMART_BATTERY_INFO UNPACKING


/**
 * @brief Get field id from smart_battery_info message
 *
 * @return  Battery ID
 */
static inline uint8_t mavlink_msg_smart_battery_info_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field battery_function from smart_battery_info message
 *
 * @return  Function of the battery
 */
static inline uint8_t mavlink_msg_smart_battery_info_get_battery_function(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field type from smart_battery_info message
 *
 * @return  Type (chemistry) of the battery
 */
static inline uint8_t mavlink_msg_smart_battery_info_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field capacity_full_specification from smart_battery_info message
 *
 * @return [mAh] Capacity when full according to manufacturer, -1: field not provided.
 */
static inline int32_t mavlink_msg_smart_battery_info_get_capacity_full_specification(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field capacity_full from smart_battery_info message
 *
 * @return [mAh] Capacity when full (accounting for battery degradation), -1: field not provided.
 */
static inline int32_t mavlink_msg_smart_battery_info_get_capacity_full(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field cycle_count from smart_battery_info message
 *
 * @return  Charge/discharge cycle count. UINT16_MAX: field not provided.
 */
static inline uint16_t mavlink_msg_smart_battery_info_get_cycle_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field serial_number from smart_battery_info message
 *
 * @return  Serial number in ASCII characters, 0 terminated. All 0: field not provided.
 */
static inline uint16_t mavlink_msg_smart_battery_info_get_serial_number(const mavlink_message_t* msg, char *serial_number)
{
    return _MAV_RETURN_char_array(msg, serial_number, 16,  21);
}

/**
 * @brief Get field device_name from smart_battery_info message
 *
 * @return  Static device name in ASCII characters, 0 terminated. All 0: field not provided. Encode as manufacturer name then product name separated using an underscore.
 */
static inline uint16_t mavlink_msg_smart_battery_info_get_device_name(const mavlink_message_t* msg, char *device_name)
{
    return _MAV_RETURN_char_array(msg, device_name, 50,  37);
}

/**
 * @brief Get field weight from smart_battery_info message
 *
 * @return [g] Battery weight. 0: field not provided.
 */
static inline uint16_t mavlink_msg_smart_battery_info_get_weight(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field discharge_minimum_voltage from smart_battery_info message
 *
 * @return [mV] Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
 */
static inline uint16_t mavlink_msg_smart_battery_info_get_discharge_minimum_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field charging_minimum_voltage from smart_battery_info message
 *
 * @return [mV] Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
 */
static inline uint16_t mavlink_msg_smart_battery_info_get_charging_minimum_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field resting_minimum_voltage from smart_battery_info message
 *
 * @return [mV] Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
 */
static inline uint16_t mavlink_msg_smart_battery_info_get_resting_minimum_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field charging_maximum_voltage from smart_battery_info message
 *
 * @return [mV] Maximum per-cell voltage when charged. 0: field not provided.
 */
static inline uint16_t mavlink_msg_smart_battery_info_get_charging_maximum_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  87);
}

/**
 * @brief Get field cells_in_series from smart_battery_info message
 *
 * @return  Number of battery cells in series. 0: field not provided.
 */
static inline uint8_t mavlink_msg_smart_battery_info_get_cells_in_series(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  89);
}

/**
 * @brief Get field discharge_maximum_current from smart_battery_info message
 *
 * @return [mA] Maximum pack discharge current. 0: field not provided.
 */
static inline uint32_t mavlink_msg_smart_battery_info_get_discharge_maximum_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  90);
}

/**
 * @brief Get field discharge_maximum_burst_current from smart_battery_info message
 *
 * @return [mA] Maximum pack discharge burst current. 0: field not provided.
 */
static inline uint32_t mavlink_msg_smart_battery_info_get_discharge_maximum_burst_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  94);
}

/**
 * @brief Get field manufacture_date from smart_battery_info message
 *
 * @return  Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. All 0: field not provided.
 */
static inline uint16_t mavlink_msg_smart_battery_info_get_manufacture_date(const mavlink_message_t* msg, char *manufacture_date)
{
    return _MAV_RETURN_char_array(msg, manufacture_date, 11,  98);
}

/**
 * @brief Decode a smart_battery_info message into a struct
 *
 * @param msg The message to decode
 * @param smart_battery_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_smart_battery_info_decode(const mavlink_message_t* msg, mavlink_smart_battery_info_t* smart_battery_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    smart_battery_info->capacity_full_specification = mavlink_msg_smart_battery_info_get_capacity_full_specification(msg);
    smart_battery_info->capacity_full = mavlink_msg_smart_battery_info_get_capacity_full(msg);
    smart_battery_info->cycle_count = mavlink_msg_smart_battery_info_get_cycle_count(msg);
    smart_battery_info->weight = mavlink_msg_smart_battery_info_get_weight(msg);
    smart_battery_info->discharge_minimum_voltage = mavlink_msg_smart_battery_info_get_discharge_minimum_voltage(msg);
    smart_battery_info->charging_minimum_voltage = mavlink_msg_smart_battery_info_get_charging_minimum_voltage(msg);
    smart_battery_info->resting_minimum_voltage = mavlink_msg_smart_battery_info_get_resting_minimum_voltage(msg);
    smart_battery_info->id = mavlink_msg_smart_battery_info_get_id(msg);
    smart_battery_info->battery_function = mavlink_msg_smart_battery_info_get_battery_function(msg);
    smart_battery_info->type = mavlink_msg_smart_battery_info_get_type(msg);
    mavlink_msg_smart_battery_info_get_serial_number(msg, smart_battery_info->serial_number);
    mavlink_msg_smart_battery_info_get_device_name(msg, smart_battery_info->device_name);
    smart_battery_info->charging_maximum_voltage = mavlink_msg_smart_battery_info_get_charging_maximum_voltage(msg);
    smart_battery_info->cells_in_series = mavlink_msg_smart_battery_info_get_cells_in_series(msg);
    smart_battery_info->discharge_maximum_current = mavlink_msg_smart_battery_info_get_discharge_maximum_current(msg);
    smart_battery_info->discharge_maximum_burst_current = mavlink_msg_smart_battery_info_get_discharge_maximum_burst_current(msg);
    mavlink_msg_smart_battery_info_get_manufacture_date(msg, smart_battery_info->manufacture_date);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN? msg->len : MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN;
        memset(smart_battery_info, 0, MAVLINK_MSG_ID_SMART_BATTERY_INFO_LEN);
    memcpy(smart_battery_info, _MAV_PAYLOAD(msg), len);
#endif
}

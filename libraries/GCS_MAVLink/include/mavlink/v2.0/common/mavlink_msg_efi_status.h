#pragma once
// MESSAGE EFI_STATUS PACKING

#define MAVLINK_MSG_ID_EFI_STATUS 225

MAVPACKED(
typedef struct __mavlink_efi_status_t {
 float ecu_index; /*<  ECU index*/
 float rpm; /*<  RPM*/
 float fuel_consumed; /*< [cm^3] Fuel consumed*/
 float fuel_flow; /*< [cm^3/min] Fuel flow rate*/
 float engine_load; /*< [%] Engine load*/
 float throttle_position; /*< [%] Throttle position*/
 float spark_dwell_time; /*< [ms] Spark dwell time*/
 float barometric_pressure; /*< [kPa] Barometric pressure*/
 float intake_manifold_pressure; /*< [kPa] Intake manifold pressure(*/
 float intake_manifold_temperature; /*< [degC] Intake manifold temperature*/
 float cylinder_head_temperature; /*< [degC] Cylinder head temperature*/
 float ignition_timing; /*< [deg] Ignition timing (Crank angle degrees)*/
 float injection_time; /*< [ms] Injection time*/
 float exhaust_gas_temperature; /*< [degC] Exhaust gas temperature*/
 float throttle_out; /*< [%] Output throttle*/
 float pt_compensation; /*<  Pressure/temperature compensation*/
 uint8_t health; /*<  EFI health status*/
 float ignition_voltage; /*< [V] Supply voltage to EFI sparking system.  Zero in this value means "unknown", so if the supply voltage really is zero volts use 0.0001 instead.*/
 float fuel_pressure; /*< [kPa] Fuel pressure. Zero in this value means "unknown", so if the fuel pressure really is zero kPa use 0.0001 instead.*/
}) mavlink_efi_status_t;

#define MAVLINK_MSG_ID_EFI_STATUS_LEN 73
#define MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN 65
#define MAVLINK_MSG_ID_225_LEN 73
#define MAVLINK_MSG_ID_225_MIN_LEN 65

#define MAVLINK_MSG_ID_EFI_STATUS_CRC 208
#define MAVLINK_MSG_ID_225_CRC 208



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_EFI_STATUS { \
    225, \
    "EFI_STATUS", \
    19, \
    {  { "health", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_efi_status_t, health) }, \
         { "ecu_index", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_efi_status_t, ecu_index) }, \
         { "rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_efi_status_t, rpm) }, \
         { "fuel_consumed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_efi_status_t, fuel_consumed) }, \
         { "fuel_flow", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_efi_status_t, fuel_flow) }, \
         { "engine_load", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_efi_status_t, engine_load) }, \
         { "throttle_position", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_efi_status_t, throttle_position) }, \
         { "spark_dwell_time", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_efi_status_t, spark_dwell_time) }, \
         { "barometric_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_efi_status_t, barometric_pressure) }, \
         { "intake_manifold_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_efi_status_t, intake_manifold_pressure) }, \
         { "intake_manifold_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_efi_status_t, intake_manifold_temperature) }, \
         { "cylinder_head_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_efi_status_t, cylinder_head_temperature) }, \
         { "ignition_timing", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_efi_status_t, ignition_timing) }, \
         { "injection_time", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_efi_status_t, injection_time) }, \
         { "exhaust_gas_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_efi_status_t, exhaust_gas_temperature) }, \
         { "throttle_out", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_efi_status_t, throttle_out) }, \
         { "pt_compensation", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_efi_status_t, pt_compensation) }, \
         { "ignition_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 65, offsetof(mavlink_efi_status_t, ignition_voltage) }, \
         { "fuel_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 69, offsetof(mavlink_efi_status_t, fuel_pressure) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_EFI_STATUS { \
    "EFI_STATUS", \
    19, \
    {  { "health", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_efi_status_t, health) }, \
         { "ecu_index", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_efi_status_t, ecu_index) }, \
         { "rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_efi_status_t, rpm) }, \
         { "fuel_consumed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_efi_status_t, fuel_consumed) }, \
         { "fuel_flow", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_efi_status_t, fuel_flow) }, \
         { "engine_load", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_efi_status_t, engine_load) }, \
         { "throttle_position", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_efi_status_t, throttle_position) }, \
         { "spark_dwell_time", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_efi_status_t, spark_dwell_time) }, \
         { "barometric_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_efi_status_t, barometric_pressure) }, \
         { "intake_manifold_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_efi_status_t, intake_manifold_pressure) }, \
         { "intake_manifold_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_efi_status_t, intake_manifold_temperature) }, \
         { "cylinder_head_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_efi_status_t, cylinder_head_temperature) }, \
         { "ignition_timing", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_efi_status_t, ignition_timing) }, \
         { "injection_time", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_efi_status_t, injection_time) }, \
         { "exhaust_gas_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_efi_status_t, exhaust_gas_temperature) }, \
         { "throttle_out", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_efi_status_t, throttle_out) }, \
         { "pt_compensation", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_efi_status_t, pt_compensation) }, \
         { "ignition_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 65, offsetof(mavlink_efi_status_t, ignition_voltage) }, \
         { "fuel_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 69, offsetof(mavlink_efi_status_t, fuel_pressure) }, \
         } \
}
#endif

/**
 * @brief Pack a efi_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param health  EFI health status
 * @param ecu_index  ECU index
 * @param rpm  RPM
 * @param fuel_consumed [cm^3] Fuel consumed
 * @param fuel_flow [cm^3/min] Fuel flow rate
 * @param engine_load [%] Engine load
 * @param throttle_position [%] Throttle position
 * @param spark_dwell_time [ms] Spark dwell time
 * @param barometric_pressure [kPa] Barometric pressure
 * @param intake_manifold_pressure [kPa] Intake manifold pressure(
 * @param intake_manifold_temperature [degC] Intake manifold temperature
 * @param cylinder_head_temperature [degC] Cylinder head temperature
 * @param ignition_timing [deg] Ignition timing (Crank angle degrees)
 * @param injection_time [ms] Injection time
 * @param exhaust_gas_temperature [degC] Exhaust gas temperature
 * @param throttle_out [%] Output throttle
 * @param pt_compensation  Pressure/temperature compensation
 * @param ignition_voltage [V] Supply voltage to EFI sparking system.  Zero in this value means "unknown", so if the supply voltage really is zero volts use 0.0001 instead.
 * @param fuel_pressure [kPa] Fuel pressure. Zero in this value means "unknown", so if the fuel pressure really is zero kPa use 0.0001 instead.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_efi_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t health, float ecu_index, float rpm, float fuel_consumed, float fuel_flow, float engine_load, float throttle_position, float spark_dwell_time, float barometric_pressure, float intake_manifold_pressure, float intake_manifold_temperature, float cylinder_head_temperature, float ignition_timing, float injection_time, float exhaust_gas_temperature, float throttle_out, float pt_compensation, float ignition_voltage, float fuel_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFI_STATUS_LEN];
    _mav_put_float(buf, 0, ecu_index);
    _mav_put_float(buf, 4, rpm);
    _mav_put_float(buf, 8, fuel_consumed);
    _mav_put_float(buf, 12, fuel_flow);
    _mav_put_float(buf, 16, engine_load);
    _mav_put_float(buf, 20, throttle_position);
    _mav_put_float(buf, 24, spark_dwell_time);
    _mav_put_float(buf, 28, barometric_pressure);
    _mav_put_float(buf, 32, intake_manifold_pressure);
    _mav_put_float(buf, 36, intake_manifold_temperature);
    _mav_put_float(buf, 40, cylinder_head_temperature);
    _mav_put_float(buf, 44, ignition_timing);
    _mav_put_float(buf, 48, injection_time);
    _mav_put_float(buf, 52, exhaust_gas_temperature);
    _mav_put_float(buf, 56, throttle_out);
    _mav_put_float(buf, 60, pt_compensation);
    _mav_put_uint8_t(buf, 64, health);
    _mav_put_float(buf, 65, ignition_voltage);
    _mav_put_float(buf, 69, fuel_pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EFI_STATUS_LEN);
#else
    mavlink_efi_status_t packet;
    packet.ecu_index = ecu_index;
    packet.rpm = rpm;
    packet.fuel_consumed = fuel_consumed;
    packet.fuel_flow = fuel_flow;
    packet.engine_load = engine_load;
    packet.throttle_position = throttle_position;
    packet.spark_dwell_time = spark_dwell_time;
    packet.barometric_pressure = barometric_pressure;
    packet.intake_manifold_pressure = intake_manifold_pressure;
    packet.intake_manifold_temperature = intake_manifold_temperature;
    packet.cylinder_head_temperature = cylinder_head_temperature;
    packet.ignition_timing = ignition_timing;
    packet.injection_time = injection_time;
    packet.exhaust_gas_temperature = exhaust_gas_temperature;
    packet.throttle_out = throttle_out;
    packet.pt_compensation = pt_compensation;
    packet.health = health;
    packet.ignition_voltage = ignition_voltage;
    packet.fuel_pressure = fuel_pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EFI_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EFI_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFI_STATUS_LEN, MAVLINK_MSG_ID_EFI_STATUS_CRC);
}

/**
 * @brief Pack a efi_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param health  EFI health status
 * @param ecu_index  ECU index
 * @param rpm  RPM
 * @param fuel_consumed [cm^3] Fuel consumed
 * @param fuel_flow [cm^3/min] Fuel flow rate
 * @param engine_load [%] Engine load
 * @param throttle_position [%] Throttle position
 * @param spark_dwell_time [ms] Spark dwell time
 * @param barometric_pressure [kPa] Barometric pressure
 * @param intake_manifold_pressure [kPa] Intake manifold pressure(
 * @param intake_manifold_temperature [degC] Intake manifold temperature
 * @param cylinder_head_temperature [degC] Cylinder head temperature
 * @param ignition_timing [deg] Ignition timing (Crank angle degrees)
 * @param injection_time [ms] Injection time
 * @param exhaust_gas_temperature [degC] Exhaust gas temperature
 * @param throttle_out [%] Output throttle
 * @param pt_compensation  Pressure/temperature compensation
 * @param ignition_voltage [V] Supply voltage to EFI sparking system.  Zero in this value means "unknown", so if the supply voltage really is zero volts use 0.0001 instead.
 * @param fuel_pressure [kPa] Fuel pressure. Zero in this value means "unknown", so if the fuel pressure really is zero kPa use 0.0001 instead.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_efi_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t health, float ecu_index, float rpm, float fuel_consumed, float fuel_flow, float engine_load, float throttle_position, float spark_dwell_time, float barometric_pressure, float intake_manifold_pressure, float intake_manifold_temperature, float cylinder_head_temperature, float ignition_timing, float injection_time, float exhaust_gas_temperature, float throttle_out, float pt_compensation, float ignition_voltage, float fuel_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFI_STATUS_LEN];
    _mav_put_float(buf, 0, ecu_index);
    _mav_put_float(buf, 4, rpm);
    _mav_put_float(buf, 8, fuel_consumed);
    _mav_put_float(buf, 12, fuel_flow);
    _mav_put_float(buf, 16, engine_load);
    _mav_put_float(buf, 20, throttle_position);
    _mav_put_float(buf, 24, spark_dwell_time);
    _mav_put_float(buf, 28, barometric_pressure);
    _mav_put_float(buf, 32, intake_manifold_pressure);
    _mav_put_float(buf, 36, intake_manifold_temperature);
    _mav_put_float(buf, 40, cylinder_head_temperature);
    _mav_put_float(buf, 44, ignition_timing);
    _mav_put_float(buf, 48, injection_time);
    _mav_put_float(buf, 52, exhaust_gas_temperature);
    _mav_put_float(buf, 56, throttle_out);
    _mav_put_float(buf, 60, pt_compensation);
    _mav_put_uint8_t(buf, 64, health);
    _mav_put_float(buf, 65, ignition_voltage);
    _mav_put_float(buf, 69, fuel_pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EFI_STATUS_LEN);
#else
    mavlink_efi_status_t packet;
    packet.ecu_index = ecu_index;
    packet.rpm = rpm;
    packet.fuel_consumed = fuel_consumed;
    packet.fuel_flow = fuel_flow;
    packet.engine_load = engine_load;
    packet.throttle_position = throttle_position;
    packet.spark_dwell_time = spark_dwell_time;
    packet.barometric_pressure = barometric_pressure;
    packet.intake_manifold_pressure = intake_manifold_pressure;
    packet.intake_manifold_temperature = intake_manifold_temperature;
    packet.cylinder_head_temperature = cylinder_head_temperature;
    packet.ignition_timing = ignition_timing;
    packet.injection_time = injection_time;
    packet.exhaust_gas_temperature = exhaust_gas_temperature;
    packet.throttle_out = throttle_out;
    packet.pt_compensation = pt_compensation;
    packet.health = health;
    packet.ignition_voltage = ignition_voltage;
    packet.fuel_pressure = fuel_pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EFI_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EFI_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFI_STATUS_LEN, MAVLINK_MSG_ID_EFI_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFI_STATUS_LEN);
#endif
}

/**
 * @brief Pack a efi_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param health  EFI health status
 * @param ecu_index  ECU index
 * @param rpm  RPM
 * @param fuel_consumed [cm^3] Fuel consumed
 * @param fuel_flow [cm^3/min] Fuel flow rate
 * @param engine_load [%] Engine load
 * @param throttle_position [%] Throttle position
 * @param spark_dwell_time [ms] Spark dwell time
 * @param barometric_pressure [kPa] Barometric pressure
 * @param intake_manifold_pressure [kPa] Intake manifold pressure(
 * @param intake_manifold_temperature [degC] Intake manifold temperature
 * @param cylinder_head_temperature [degC] Cylinder head temperature
 * @param ignition_timing [deg] Ignition timing (Crank angle degrees)
 * @param injection_time [ms] Injection time
 * @param exhaust_gas_temperature [degC] Exhaust gas temperature
 * @param throttle_out [%] Output throttle
 * @param pt_compensation  Pressure/temperature compensation
 * @param ignition_voltage [V] Supply voltage to EFI sparking system.  Zero in this value means "unknown", so if the supply voltage really is zero volts use 0.0001 instead.
 * @param fuel_pressure [kPa] Fuel pressure. Zero in this value means "unknown", so if the fuel pressure really is zero kPa use 0.0001 instead.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_efi_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t health,float ecu_index,float rpm,float fuel_consumed,float fuel_flow,float engine_load,float throttle_position,float spark_dwell_time,float barometric_pressure,float intake_manifold_pressure,float intake_manifold_temperature,float cylinder_head_temperature,float ignition_timing,float injection_time,float exhaust_gas_temperature,float throttle_out,float pt_compensation,float ignition_voltage,float fuel_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFI_STATUS_LEN];
    _mav_put_float(buf, 0, ecu_index);
    _mav_put_float(buf, 4, rpm);
    _mav_put_float(buf, 8, fuel_consumed);
    _mav_put_float(buf, 12, fuel_flow);
    _mav_put_float(buf, 16, engine_load);
    _mav_put_float(buf, 20, throttle_position);
    _mav_put_float(buf, 24, spark_dwell_time);
    _mav_put_float(buf, 28, barometric_pressure);
    _mav_put_float(buf, 32, intake_manifold_pressure);
    _mav_put_float(buf, 36, intake_manifold_temperature);
    _mav_put_float(buf, 40, cylinder_head_temperature);
    _mav_put_float(buf, 44, ignition_timing);
    _mav_put_float(buf, 48, injection_time);
    _mav_put_float(buf, 52, exhaust_gas_temperature);
    _mav_put_float(buf, 56, throttle_out);
    _mav_put_float(buf, 60, pt_compensation);
    _mav_put_uint8_t(buf, 64, health);
    _mav_put_float(buf, 65, ignition_voltage);
    _mav_put_float(buf, 69, fuel_pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EFI_STATUS_LEN);
#else
    mavlink_efi_status_t packet;
    packet.ecu_index = ecu_index;
    packet.rpm = rpm;
    packet.fuel_consumed = fuel_consumed;
    packet.fuel_flow = fuel_flow;
    packet.engine_load = engine_load;
    packet.throttle_position = throttle_position;
    packet.spark_dwell_time = spark_dwell_time;
    packet.barometric_pressure = barometric_pressure;
    packet.intake_manifold_pressure = intake_manifold_pressure;
    packet.intake_manifold_temperature = intake_manifold_temperature;
    packet.cylinder_head_temperature = cylinder_head_temperature;
    packet.ignition_timing = ignition_timing;
    packet.injection_time = injection_time;
    packet.exhaust_gas_temperature = exhaust_gas_temperature;
    packet.throttle_out = throttle_out;
    packet.pt_compensation = pt_compensation;
    packet.health = health;
    packet.ignition_voltage = ignition_voltage;
    packet.fuel_pressure = fuel_pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EFI_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_EFI_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFI_STATUS_LEN, MAVLINK_MSG_ID_EFI_STATUS_CRC);
}

/**
 * @brief Encode a efi_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param efi_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_efi_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_efi_status_t* efi_status)
{
    return mavlink_msg_efi_status_pack(system_id, component_id, msg, efi_status->health, efi_status->ecu_index, efi_status->rpm, efi_status->fuel_consumed, efi_status->fuel_flow, efi_status->engine_load, efi_status->throttle_position, efi_status->spark_dwell_time, efi_status->barometric_pressure, efi_status->intake_manifold_pressure, efi_status->intake_manifold_temperature, efi_status->cylinder_head_temperature, efi_status->ignition_timing, efi_status->injection_time, efi_status->exhaust_gas_temperature, efi_status->throttle_out, efi_status->pt_compensation, efi_status->ignition_voltage, efi_status->fuel_pressure);
}

/**
 * @brief Encode a efi_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param efi_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_efi_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_efi_status_t* efi_status)
{
    return mavlink_msg_efi_status_pack_chan(system_id, component_id, chan, msg, efi_status->health, efi_status->ecu_index, efi_status->rpm, efi_status->fuel_consumed, efi_status->fuel_flow, efi_status->engine_load, efi_status->throttle_position, efi_status->spark_dwell_time, efi_status->barometric_pressure, efi_status->intake_manifold_pressure, efi_status->intake_manifold_temperature, efi_status->cylinder_head_temperature, efi_status->ignition_timing, efi_status->injection_time, efi_status->exhaust_gas_temperature, efi_status->throttle_out, efi_status->pt_compensation, efi_status->ignition_voltage, efi_status->fuel_pressure);
}

/**
 * @brief Encode a efi_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param efi_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_efi_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_efi_status_t* efi_status)
{
    return mavlink_msg_efi_status_pack_status(system_id, component_id, _status, msg,  efi_status->health, efi_status->ecu_index, efi_status->rpm, efi_status->fuel_consumed, efi_status->fuel_flow, efi_status->engine_load, efi_status->throttle_position, efi_status->spark_dwell_time, efi_status->barometric_pressure, efi_status->intake_manifold_pressure, efi_status->intake_manifold_temperature, efi_status->cylinder_head_temperature, efi_status->ignition_timing, efi_status->injection_time, efi_status->exhaust_gas_temperature, efi_status->throttle_out, efi_status->pt_compensation, efi_status->ignition_voltage, efi_status->fuel_pressure);
}

/**
 * @brief Send a efi_status message
 * @param chan MAVLink channel to send the message
 *
 * @param health  EFI health status
 * @param ecu_index  ECU index
 * @param rpm  RPM
 * @param fuel_consumed [cm^3] Fuel consumed
 * @param fuel_flow [cm^3/min] Fuel flow rate
 * @param engine_load [%] Engine load
 * @param throttle_position [%] Throttle position
 * @param spark_dwell_time [ms] Spark dwell time
 * @param barometric_pressure [kPa] Barometric pressure
 * @param intake_manifold_pressure [kPa] Intake manifold pressure(
 * @param intake_manifold_temperature [degC] Intake manifold temperature
 * @param cylinder_head_temperature [degC] Cylinder head temperature
 * @param ignition_timing [deg] Ignition timing (Crank angle degrees)
 * @param injection_time [ms] Injection time
 * @param exhaust_gas_temperature [degC] Exhaust gas temperature
 * @param throttle_out [%] Output throttle
 * @param pt_compensation  Pressure/temperature compensation
 * @param ignition_voltage [V] Supply voltage to EFI sparking system.  Zero in this value means "unknown", so if the supply voltage really is zero volts use 0.0001 instead.
 * @param fuel_pressure [kPa] Fuel pressure. Zero in this value means "unknown", so if the fuel pressure really is zero kPa use 0.0001 instead.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_efi_status_send(mavlink_channel_t chan, uint8_t health, float ecu_index, float rpm, float fuel_consumed, float fuel_flow, float engine_load, float throttle_position, float spark_dwell_time, float barometric_pressure, float intake_manifold_pressure, float intake_manifold_temperature, float cylinder_head_temperature, float ignition_timing, float injection_time, float exhaust_gas_temperature, float throttle_out, float pt_compensation, float ignition_voltage, float fuel_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_EFI_STATUS_LEN];
    _mav_put_float(buf, 0, ecu_index);
    _mav_put_float(buf, 4, rpm);
    _mav_put_float(buf, 8, fuel_consumed);
    _mav_put_float(buf, 12, fuel_flow);
    _mav_put_float(buf, 16, engine_load);
    _mav_put_float(buf, 20, throttle_position);
    _mav_put_float(buf, 24, spark_dwell_time);
    _mav_put_float(buf, 28, barometric_pressure);
    _mav_put_float(buf, 32, intake_manifold_pressure);
    _mav_put_float(buf, 36, intake_manifold_temperature);
    _mav_put_float(buf, 40, cylinder_head_temperature);
    _mav_put_float(buf, 44, ignition_timing);
    _mav_put_float(buf, 48, injection_time);
    _mav_put_float(buf, 52, exhaust_gas_temperature);
    _mav_put_float(buf, 56, throttle_out);
    _mav_put_float(buf, 60, pt_compensation);
    _mav_put_uint8_t(buf, 64, health);
    _mav_put_float(buf, 65, ignition_voltage);
    _mav_put_float(buf, 69, fuel_pressure);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFI_STATUS, buf, MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFI_STATUS_LEN, MAVLINK_MSG_ID_EFI_STATUS_CRC);
#else
    mavlink_efi_status_t packet;
    packet.ecu_index = ecu_index;
    packet.rpm = rpm;
    packet.fuel_consumed = fuel_consumed;
    packet.fuel_flow = fuel_flow;
    packet.engine_load = engine_load;
    packet.throttle_position = throttle_position;
    packet.spark_dwell_time = spark_dwell_time;
    packet.barometric_pressure = barometric_pressure;
    packet.intake_manifold_pressure = intake_manifold_pressure;
    packet.intake_manifold_temperature = intake_manifold_temperature;
    packet.cylinder_head_temperature = cylinder_head_temperature;
    packet.ignition_timing = ignition_timing;
    packet.injection_time = injection_time;
    packet.exhaust_gas_temperature = exhaust_gas_temperature;
    packet.throttle_out = throttle_out;
    packet.pt_compensation = pt_compensation;
    packet.health = health;
    packet.ignition_voltage = ignition_voltage;
    packet.fuel_pressure = fuel_pressure;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFI_STATUS, (const char *)&packet, MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFI_STATUS_LEN, MAVLINK_MSG_ID_EFI_STATUS_CRC);
#endif
}

/**
 * @brief Send a efi_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_efi_status_send_struct(mavlink_channel_t chan, const mavlink_efi_status_t* efi_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_efi_status_send(chan, efi_status->health, efi_status->ecu_index, efi_status->rpm, efi_status->fuel_consumed, efi_status->fuel_flow, efi_status->engine_load, efi_status->throttle_position, efi_status->spark_dwell_time, efi_status->barometric_pressure, efi_status->intake_manifold_pressure, efi_status->intake_manifold_temperature, efi_status->cylinder_head_temperature, efi_status->ignition_timing, efi_status->injection_time, efi_status->exhaust_gas_temperature, efi_status->throttle_out, efi_status->pt_compensation, efi_status->ignition_voltage, efi_status->fuel_pressure);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFI_STATUS, (const char *)efi_status, MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFI_STATUS_LEN, MAVLINK_MSG_ID_EFI_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_EFI_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_efi_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t health, float ecu_index, float rpm, float fuel_consumed, float fuel_flow, float engine_load, float throttle_position, float spark_dwell_time, float barometric_pressure, float intake_manifold_pressure, float intake_manifold_temperature, float cylinder_head_temperature, float ignition_timing, float injection_time, float exhaust_gas_temperature, float throttle_out, float pt_compensation, float ignition_voltage, float fuel_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, ecu_index);
    _mav_put_float(buf, 4, rpm);
    _mav_put_float(buf, 8, fuel_consumed);
    _mav_put_float(buf, 12, fuel_flow);
    _mav_put_float(buf, 16, engine_load);
    _mav_put_float(buf, 20, throttle_position);
    _mav_put_float(buf, 24, spark_dwell_time);
    _mav_put_float(buf, 28, barometric_pressure);
    _mav_put_float(buf, 32, intake_manifold_pressure);
    _mav_put_float(buf, 36, intake_manifold_temperature);
    _mav_put_float(buf, 40, cylinder_head_temperature);
    _mav_put_float(buf, 44, ignition_timing);
    _mav_put_float(buf, 48, injection_time);
    _mav_put_float(buf, 52, exhaust_gas_temperature);
    _mav_put_float(buf, 56, throttle_out);
    _mav_put_float(buf, 60, pt_compensation);
    _mav_put_uint8_t(buf, 64, health);
    _mav_put_float(buf, 65, ignition_voltage);
    _mav_put_float(buf, 69, fuel_pressure);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFI_STATUS, buf, MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFI_STATUS_LEN, MAVLINK_MSG_ID_EFI_STATUS_CRC);
#else
    mavlink_efi_status_t *packet = (mavlink_efi_status_t *)msgbuf;
    packet->ecu_index = ecu_index;
    packet->rpm = rpm;
    packet->fuel_consumed = fuel_consumed;
    packet->fuel_flow = fuel_flow;
    packet->engine_load = engine_load;
    packet->throttle_position = throttle_position;
    packet->spark_dwell_time = spark_dwell_time;
    packet->barometric_pressure = barometric_pressure;
    packet->intake_manifold_pressure = intake_manifold_pressure;
    packet->intake_manifold_temperature = intake_manifold_temperature;
    packet->cylinder_head_temperature = cylinder_head_temperature;
    packet->ignition_timing = ignition_timing;
    packet->injection_time = injection_time;
    packet->exhaust_gas_temperature = exhaust_gas_temperature;
    packet->throttle_out = throttle_out;
    packet->pt_compensation = pt_compensation;
    packet->health = health;
    packet->ignition_voltage = ignition_voltage;
    packet->fuel_pressure = fuel_pressure;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EFI_STATUS, (const char *)packet, MAVLINK_MSG_ID_EFI_STATUS_MIN_LEN, MAVLINK_MSG_ID_EFI_STATUS_LEN, MAVLINK_MSG_ID_EFI_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE EFI_STATUS UNPACKING


/**
 * @brief Get field health from efi_status message
 *
 * @return  EFI health status
 */
static inline uint8_t mavlink_msg_efi_status_get_health(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  64);
}

/**
 * @brief Get field ecu_index from efi_status message
 *
 * @return  ECU index
 */
static inline float mavlink_msg_efi_status_get_ecu_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field rpm from efi_status message
 *
 * @return  RPM
 */
static inline float mavlink_msg_efi_status_get_rpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field fuel_consumed from efi_status message
 *
 * @return [cm^3] Fuel consumed
 */
static inline float mavlink_msg_efi_status_get_fuel_consumed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field fuel_flow from efi_status message
 *
 * @return [cm^3/min] Fuel flow rate
 */
static inline float mavlink_msg_efi_status_get_fuel_flow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field engine_load from efi_status message
 *
 * @return [%] Engine load
 */
static inline float mavlink_msg_efi_status_get_engine_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field throttle_position from efi_status message
 *
 * @return [%] Throttle position
 */
static inline float mavlink_msg_efi_status_get_throttle_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field spark_dwell_time from efi_status message
 *
 * @return [ms] Spark dwell time
 */
static inline float mavlink_msg_efi_status_get_spark_dwell_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field barometric_pressure from efi_status message
 *
 * @return [kPa] Barometric pressure
 */
static inline float mavlink_msg_efi_status_get_barometric_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field intake_manifold_pressure from efi_status message
 *
 * @return [kPa] Intake manifold pressure(
 */
static inline float mavlink_msg_efi_status_get_intake_manifold_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field intake_manifold_temperature from efi_status message
 *
 * @return [degC] Intake manifold temperature
 */
static inline float mavlink_msg_efi_status_get_intake_manifold_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field cylinder_head_temperature from efi_status message
 *
 * @return [degC] Cylinder head temperature
 */
static inline float mavlink_msg_efi_status_get_cylinder_head_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field ignition_timing from efi_status message
 *
 * @return [deg] Ignition timing (Crank angle degrees)
 */
static inline float mavlink_msg_efi_status_get_ignition_timing(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field injection_time from efi_status message
 *
 * @return [ms] Injection time
 */
static inline float mavlink_msg_efi_status_get_injection_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field exhaust_gas_temperature from efi_status message
 *
 * @return [degC] Exhaust gas temperature
 */
static inline float mavlink_msg_efi_status_get_exhaust_gas_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field throttle_out from efi_status message
 *
 * @return [%] Output throttle
 */
static inline float mavlink_msg_efi_status_get_throttle_out(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field pt_compensation from efi_status message
 *
 * @return  Pressure/temperature compensation
 */
static inline float mavlink_msg_efi_status_get_pt_compensation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field ignition_voltage from efi_status message
 *
 * @return [V] Supply voltage to EFI sparking system.  Zero in this value means "unknown", so if the supply voltage really is zero volts use 0.0001 instead.
 */
static inline float mavlink_msg_efi_status_get_ignition_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  65);
}

/**
 * @brief Get field fuel_pressure from efi_status message
 *
 * @return [kPa] Fuel pressure. Zero in this value means "unknown", so if the fuel pressure really is zero kPa use 0.0001 instead.
 */
static inline float mavlink_msg_efi_status_get_fuel_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  69);
}

/**
 * @brief Decode a efi_status message into a struct
 *
 * @param msg The message to decode
 * @param efi_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_efi_status_decode(const mavlink_message_t* msg, mavlink_efi_status_t* efi_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    efi_status->ecu_index = mavlink_msg_efi_status_get_ecu_index(msg);
    efi_status->rpm = mavlink_msg_efi_status_get_rpm(msg);
    efi_status->fuel_consumed = mavlink_msg_efi_status_get_fuel_consumed(msg);
    efi_status->fuel_flow = mavlink_msg_efi_status_get_fuel_flow(msg);
    efi_status->engine_load = mavlink_msg_efi_status_get_engine_load(msg);
    efi_status->throttle_position = mavlink_msg_efi_status_get_throttle_position(msg);
    efi_status->spark_dwell_time = mavlink_msg_efi_status_get_spark_dwell_time(msg);
    efi_status->barometric_pressure = mavlink_msg_efi_status_get_barometric_pressure(msg);
    efi_status->intake_manifold_pressure = mavlink_msg_efi_status_get_intake_manifold_pressure(msg);
    efi_status->intake_manifold_temperature = mavlink_msg_efi_status_get_intake_manifold_temperature(msg);
    efi_status->cylinder_head_temperature = mavlink_msg_efi_status_get_cylinder_head_temperature(msg);
    efi_status->ignition_timing = mavlink_msg_efi_status_get_ignition_timing(msg);
    efi_status->injection_time = mavlink_msg_efi_status_get_injection_time(msg);
    efi_status->exhaust_gas_temperature = mavlink_msg_efi_status_get_exhaust_gas_temperature(msg);
    efi_status->throttle_out = mavlink_msg_efi_status_get_throttle_out(msg);
    efi_status->pt_compensation = mavlink_msg_efi_status_get_pt_compensation(msg);
    efi_status->health = mavlink_msg_efi_status_get_health(msg);
    efi_status->ignition_voltage = mavlink_msg_efi_status_get_ignition_voltage(msg);
    efi_status->fuel_pressure = mavlink_msg_efi_status_get_fuel_pressure(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_EFI_STATUS_LEN? msg->len : MAVLINK_MSG_ID_EFI_STATUS_LEN;
        memset(efi_status, 0, MAVLINK_MSG_ID_EFI_STATUS_LEN);
    memcpy(efi_status, _MAV_PAYLOAD(msg), len);
#endif
}

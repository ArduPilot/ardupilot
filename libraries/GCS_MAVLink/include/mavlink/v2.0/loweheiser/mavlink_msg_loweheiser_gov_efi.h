#pragma once
// MESSAGE LOWEHEISER_GOV_EFI PACKING

#define MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI 10151


typedef struct __mavlink_loweheiser_gov_efi_t {
 float volt_batt; /*< [V] Generator Battery voltage.*/
 float curr_batt; /*< [A] Generator Battery current.*/
 float curr_gen; /*< [A] Current being produced by generator.*/
 float curr_rot; /*< [A] Load current being consumed by the UAV (sum of curr_gen and curr_batt)*/
 float fuel_level; /*< [l] Generator fuel remaining in litres.*/
 float throttle; /*< [%] Throttle Output.*/
 uint32_t runtime; /*< [s] Seconds this generator has run since it was rebooted.*/
 int32_t until_maintenance; /*< [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past due.*/
 float rectifier_temp; /*< [degC] The Temperature of the rectifier.*/
 float generator_temp; /*< [degC] The temperature of the mechanical motor, fuel cell core or generator.*/
 float efi_batt; /*< [V]  EFI Supply Voltage.*/
 float efi_rpm; /*< [rpm] Motor RPM.*/
 float efi_pw; /*< [ms] Injector pulse-width in miliseconds.*/
 float efi_fuel_flow; /*<  Fuel flow rate in litres/hour.*/
 float efi_fuel_consumed; /*< [l] Fuel consumed.*/
 float efi_baro; /*< [kPa] Atmospheric pressure.*/
 float efi_mat; /*< [degC] Manifold Air Temperature.*/
 float efi_clt; /*< [degC] Cylinder Head Temperature.*/
 float efi_tps; /*< [%] Throttle Position.*/
 float efi_exhaust_gas_temperature; /*< [degC] Exhaust gas temperature.*/
 uint16_t generator_status; /*<  Generator status.*/
 uint16_t efi_status; /*<  EFI status.*/
 uint8_t efi_index; /*<  EFI index.*/
} mavlink_loweheiser_gov_efi_t;

#define MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN 85
#define MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN 85
#define MAVLINK_MSG_ID_10151_LEN 85
#define MAVLINK_MSG_ID_10151_MIN_LEN 85

#define MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_CRC 195
#define MAVLINK_MSG_ID_10151_CRC 195



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LOWEHEISER_GOV_EFI { \
    10151, \
    "LOWEHEISER_GOV_EFI", \
    23, \
    {  { "volt_batt", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_loweheiser_gov_efi_t, volt_batt) }, \
         { "curr_batt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_loweheiser_gov_efi_t, curr_batt) }, \
         { "curr_gen", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_loweheiser_gov_efi_t, curr_gen) }, \
         { "curr_rot", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_loweheiser_gov_efi_t, curr_rot) }, \
         { "fuel_level", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_loweheiser_gov_efi_t, fuel_level) }, \
         { "throttle", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_loweheiser_gov_efi_t, throttle) }, \
         { "runtime", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_loweheiser_gov_efi_t, runtime) }, \
         { "until_maintenance", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_loweheiser_gov_efi_t, until_maintenance) }, \
         { "rectifier_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_loweheiser_gov_efi_t, rectifier_temp) }, \
         { "generator_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_loweheiser_gov_efi_t, generator_temp) }, \
         { "efi_batt", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_loweheiser_gov_efi_t, efi_batt) }, \
         { "efi_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_loweheiser_gov_efi_t, efi_rpm) }, \
         { "efi_pw", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_loweheiser_gov_efi_t, efi_pw) }, \
         { "efi_fuel_flow", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_loweheiser_gov_efi_t, efi_fuel_flow) }, \
         { "efi_fuel_consumed", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_loweheiser_gov_efi_t, efi_fuel_consumed) }, \
         { "efi_baro", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_loweheiser_gov_efi_t, efi_baro) }, \
         { "efi_mat", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_loweheiser_gov_efi_t, efi_mat) }, \
         { "efi_clt", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_loweheiser_gov_efi_t, efi_clt) }, \
         { "efi_tps", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_loweheiser_gov_efi_t, efi_tps) }, \
         { "efi_exhaust_gas_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_loweheiser_gov_efi_t, efi_exhaust_gas_temperature) }, \
         { "efi_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 84, offsetof(mavlink_loweheiser_gov_efi_t, efi_index) }, \
         { "generator_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 80, offsetof(mavlink_loweheiser_gov_efi_t, generator_status) }, \
         { "efi_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 82, offsetof(mavlink_loweheiser_gov_efi_t, efi_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LOWEHEISER_GOV_EFI { \
    "LOWEHEISER_GOV_EFI", \
    23, \
    {  { "volt_batt", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_loweheiser_gov_efi_t, volt_batt) }, \
         { "curr_batt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_loweheiser_gov_efi_t, curr_batt) }, \
         { "curr_gen", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_loweheiser_gov_efi_t, curr_gen) }, \
         { "curr_rot", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_loweheiser_gov_efi_t, curr_rot) }, \
         { "fuel_level", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_loweheiser_gov_efi_t, fuel_level) }, \
         { "throttle", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_loweheiser_gov_efi_t, throttle) }, \
         { "runtime", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_loweheiser_gov_efi_t, runtime) }, \
         { "until_maintenance", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_loweheiser_gov_efi_t, until_maintenance) }, \
         { "rectifier_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_loweheiser_gov_efi_t, rectifier_temp) }, \
         { "generator_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_loweheiser_gov_efi_t, generator_temp) }, \
         { "efi_batt", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_loweheiser_gov_efi_t, efi_batt) }, \
         { "efi_rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_loweheiser_gov_efi_t, efi_rpm) }, \
         { "efi_pw", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_loweheiser_gov_efi_t, efi_pw) }, \
         { "efi_fuel_flow", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_loweheiser_gov_efi_t, efi_fuel_flow) }, \
         { "efi_fuel_consumed", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_loweheiser_gov_efi_t, efi_fuel_consumed) }, \
         { "efi_baro", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_loweheiser_gov_efi_t, efi_baro) }, \
         { "efi_mat", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_loweheiser_gov_efi_t, efi_mat) }, \
         { "efi_clt", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_loweheiser_gov_efi_t, efi_clt) }, \
         { "efi_tps", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_loweheiser_gov_efi_t, efi_tps) }, \
         { "efi_exhaust_gas_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_loweheiser_gov_efi_t, efi_exhaust_gas_temperature) }, \
         { "efi_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 84, offsetof(mavlink_loweheiser_gov_efi_t, efi_index) }, \
         { "generator_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 80, offsetof(mavlink_loweheiser_gov_efi_t, generator_status) }, \
         { "efi_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 82, offsetof(mavlink_loweheiser_gov_efi_t, efi_status) }, \
         } \
}
#endif

/**
 * @brief Pack a loweheiser_gov_efi message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param volt_batt [V] Generator Battery voltage.
 * @param curr_batt [A] Generator Battery current.
 * @param curr_gen [A] Current being produced by generator.
 * @param curr_rot [A] Load current being consumed by the UAV (sum of curr_gen and curr_batt)
 * @param fuel_level [l] Generator fuel remaining in litres.
 * @param throttle [%] Throttle Output.
 * @param runtime [s] Seconds this generator has run since it was rebooted.
 * @param until_maintenance [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past due.
 * @param rectifier_temp [degC] The Temperature of the rectifier.
 * @param generator_temp [degC] The temperature of the mechanical motor, fuel cell core or generator.
 * @param efi_batt [V]  EFI Supply Voltage.
 * @param efi_rpm [rpm] Motor RPM.
 * @param efi_pw [ms] Injector pulse-width in miliseconds.
 * @param efi_fuel_flow  Fuel flow rate in litres/hour.
 * @param efi_fuel_consumed [l] Fuel consumed.
 * @param efi_baro [kPa] Atmospheric pressure.
 * @param efi_mat [degC] Manifold Air Temperature.
 * @param efi_clt [degC] Cylinder Head Temperature.
 * @param efi_tps [%] Throttle Position.
 * @param efi_exhaust_gas_temperature [degC] Exhaust gas temperature.
 * @param efi_index  EFI index.
 * @param generator_status  Generator status.
 * @param efi_status  EFI status.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_loweheiser_gov_efi_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float volt_batt, float curr_batt, float curr_gen, float curr_rot, float fuel_level, float throttle, uint32_t runtime, int32_t until_maintenance, float rectifier_temp, float generator_temp, float efi_batt, float efi_rpm, float efi_pw, float efi_fuel_flow, float efi_fuel_consumed, float efi_baro, float efi_mat, float efi_clt, float efi_tps, float efi_exhaust_gas_temperature, uint8_t efi_index, uint16_t generator_status, uint16_t efi_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN];
    _mav_put_float(buf, 0, volt_batt);
    _mav_put_float(buf, 4, curr_batt);
    _mav_put_float(buf, 8, curr_gen);
    _mav_put_float(buf, 12, curr_rot);
    _mav_put_float(buf, 16, fuel_level);
    _mav_put_float(buf, 20, throttle);
    _mav_put_uint32_t(buf, 24, runtime);
    _mav_put_int32_t(buf, 28, until_maintenance);
    _mav_put_float(buf, 32, rectifier_temp);
    _mav_put_float(buf, 36, generator_temp);
    _mav_put_float(buf, 40, efi_batt);
    _mav_put_float(buf, 44, efi_rpm);
    _mav_put_float(buf, 48, efi_pw);
    _mav_put_float(buf, 52, efi_fuel_flow);
    _mav_put_float(buf, 56, efi_fuel_consumed);
    _mav_put_float(buf, 60, efi_baro);
    _mav_put_float(buf, 64, efi_mat);
    _mav_put_float(buf, 68, efi_clt);
    _mav_put_float(buf, 72, efi_tps);
    _mav_put_float(buf, 76, efi_exhaust_gas_temperature);
    _mav_put_uint16_t(buf, 80, generator_status);
    _mav_put_uint16_t(buf, 82, efi_status);
    _mav_put_uint8_t(buf, 84, efi_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN);
#else
    mavlink_loweheiser_gov_efi_t packet;
    packet.volt_batt = volt_batt;
    packet.curr_batt = curr_batt;
    packet.curr_gen = curr_gen;
    packet.curr_rot = curr_rot;
    packet.fuel_level = fuel_level;
    packet.throttle = throttle;
    packet.runtime = runtime;
    packet.until_maintenance = until_maintenance;
    packet.rectifier_temp = rectifier_temp;
    packet.generator_temp = generator_temp;
    packet.efi_batt = efi_batt;
    packet.efi_rpm = efi_rpm;
    packet.efi_pw = efi_pw;
    packet.efi_fuel_flow = efi_fuel_flow;
    packet.efi_fuel_consumed = efi_fuel_consumed;
    packet.efi_baro = efi_baro;
    packet.efi_mat = efi_mat;
    packet.efi_clt = efi_clt;
    packet.efi_tps = efi_tps;
    packet.efi_exhaust_gas_temperature = efi_exhaust_gas_temperature;
    packet.generator_status = generator_status;
    packet.efi_status = efi_status;
    packet.efi_index = efi_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_CRC);
}

/**
 * @brief Pack a loweheiser_gov_efi message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param volt_batt [V] Generator Battery voltage.
 * @param curr_batt [A] Generator Battery current.
 * @param curr_gen [A] Current being produced by generator.
 * @param curr_rot [A] Load current being consumed by the UAV (sum of curr_gen and curr_batt)
 * @param fuel_level [l] Generator fuel remaining in litres.
 * @param throttle [%] Throttle Output.
 * @param runtime [s] Seconds this generator has run since it was rebooted.
 * @param until_maintenance [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past due.
 * @param rectifier_temp [degC] The Temperature of the rectifier.
 * @param generator_temp [degC] The temperature of the mechanical motor, fuel cell core or generator.
 * @param efi_batt [V]  EFI Supply Voltage.
 * @param efi_rpm [rpm] Motor RPM.
 * @param efi_pw [ms] Injector pulse-width in miliseconds.
 * @param efi_fuel_flow  Fuel flow rate in litres/hour.
 * @param efi_fuel_consumed [l] Fuel consumed.
 * @param efi_baro [kPa] Atmospheric pressure.
 * @param efi_mat [degC] Manifold Air Temperature.
 * @param efi_clt [degC] Cylinder Head Temperature.
 * @param efi_tps [%] Throttle Position.
 * @param efi_exhaust_gas_temperature [degC] Exhaust gas temperature.
 * @param efi_index  EFI index.
 * @param generator_status  Generator status.
 * @param efi_status  EFI status.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_loweheiser_gov_efi_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               float volt_batt, float curr_batt, float curr_gen, float curr_rot, float fuel_level, float throttle, uint32_t runtime, int32_t until_maintenance, float rectifier_temp, float generator_temp, float efi_batt, float efi_rpm, float efi_pw, float efi_fuel_flow, float efi_fuel_consumed, float efi_baro, float efi_mat, float efi_clt, float efi_tps, float efi_exhaust_gas_temperature, uint8_t efi_index, uint16_t generator_status, uint16_t efi_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN];
    _mav_put_float(buf, 0, volt_batt);
    _mav_put_float(buf, 4, curr_batt);
    _mav_put_float(buf, 8, curr_gen);
    _mav_put_float(buf, 12, curr_rot);
    _mav_put_float(buf, 16, fuel_level);
    _mav_put_float(buf, 20, throttle);
    _mav_put_uint32_t(buf, 24, runtime);
    _mav_put_int32_t(buf, 28, until_maintenance);
    _mav_put_float(buf, 32, rectifier_temp);
    _mav_put_float(buf, 36, generator_temp);
    _mav_put_float(buf, 40, efi_batt);
    _mav_put_float(buf, 44, efi_rpm);
    _mav_put_float(buf, 48, efi_pw);
    _mav_put_float(buf, 52, efi_fuel_flow);
    _mav_put_float(buf, 56, efi_fuel_consumed);
    _mav_put_float(buf, 60, efi_baro);
    _mav_put_float(buf, 64, efi_mat);
    _mav_put_float(buf, 68, efi_clt);
    _mav_put_float(buf, 72, efi_tps);
    _mav_put_float(buf, 76, efi_exhaust_gas_temperature);
    _mav_put_uint16_t(buf, 80, generator_status);
    _mav_put_uint16_t(buf, 82, efi_status);
    _mav_put_uint8_t(buf, 84, efi_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN);
#else
    mavlink_loweheiser_gov_efi_t packet;
    packet.volt_batt = volt_batt;
    packet.curr_batt = curr_batt;
    packet.curr_gen = curr_gen;
    packet.curr_rot = curr_rot;
    packet.fuel_level = fuel_level;
    packet.throttle = throttle;
    packet.runtime = runtime;
    packet.until_maintenance = until_maintenance;
    packet.rectifier_temp = rectifier_temp;
    packet.generator_temp = generator_temp;
    packet.efi_batt = efi_batt;
    packet.efi_rpm = efi_rpm;
    packet.efi_pw = efi_pw;
    packet.efi_fuel_flow = efi_fuel_flow;
    packet.efi_fuel_consumed = efi_fuel_consumed;
    packet.efi_baro = efi_baro;
    packet.efi_mat = efi_mat;
    packet.efi_clt = efi_clt;
    packet.efi_tps = efi_tps;
    packet.efi_exhaust_gas_temperature = efi_exhaust_gas_temperature;
    packet.generator_status = generator_status;
    packet.efi_status = efi_status;
    packet.efi_index = efi_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN);
#endif
}

/**
 * @brief Pack a loweheiser_gov_efi message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param volt_batt [V] Generator Battery voltage.
 * @param curr_batt [A] Generator Battery current.
 * @param curr_gen [A] Current being produced by generator.
 * @param curr_rot [A] Load current being consumed by the UAV (sum of curr_gen and curr_batt)
 * @param fuel_level [l] Generator fuel remaining in litres.
 * @param throttle [%] Throttle Output.
 * @param runtime [s] Seconds this generator has run since it was rebooted.
 * @param until_maintenance [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past due.
 * @param rectifier_temp [degC] The Temperature of the rectifier.
 * @param generator_temp [degC] The temperature of the mechanical motor, fuel cell core or generator.
 * @param efi_batt [V]  EFI Supply Voltage.
 * @param efi_rpm [rpm] Motor RPM.
 * @param efi_pw [ms] Injector pulse-width in miliseconds.
 * @param efi_fuel_flow  Fuel flow rate in litres/hour.
 * @param efi_fuel_consumed [l] Fuel consumed.
 * @param efi_baro [kPa] Atmospheric pressure.
 * @param efi_mat [degC] Manifold Air Temperature.
 * @param efi_clt [degC] Cylinder Head Temperature.
 * @param efi_tps [%] Throttle Position.
 * @param efi_exhaust_gas_temperature [degC] Exhaust gas temperature.
 * @param efi_index  EFI index.
 * @param generator_status  Generator status.
 * @param efi_status  EFI status.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_loweheiser_gov_efi_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float volt_batt,float curr_batt,float curr_gen,float curr_rot,float fuel_level,float throttle,uint32_t runtime,int32_t until_maintenance,float rectifier_temp,float generator_temp,float efi_batt,float efi_rpm,float efi_pw,float efi_fuel_flow,float efi_fuel_consumed,float efi_baro,float efi_mat,float efi_clt,float efi_tps,float efi_exhaust_gas_temperature,uint8_t efi_index,uint16_t generator_status,uint16_t efi_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN];
    _mav_put_float(buf, 0, volt_batt);
    _mav_put_float(buf, 4, curr_batt);
    _mav_put_float(buf, 8, curr_gen);
    _mav_put_float(buf, 12, curr_rot);
    _mav_put_float(buf, 16, fuel_level);
    _mav_put_float(buf, 20, throttle);
    _mav_put_uint32_t(buf, 24, runtime);
    _mav_put_int32_t(buf, 28, until_maintenance);
    _mav_put_float(buf, 32, rectifier_temp);
    _mav_put_float(buf, 36, generator_temp);
    _mav_put_float(buf, 40, efi_batt);
    _mav_put_float(buf, 44, efi_rpm);
    _mav_put_float(buf, 48, efi_pw);
    _mav_put_float(buf, 52, efi_fuel_flow);
    _mav_put_float(buf, 56, efi_fuel_consumed);
    _mav_put_float(buf, 60, efi_baro);
    _mav_put_float(buf, 64, efi_mat);
    _mav_put_float(buf, 68, efi_clt);
    _mav_put_float(buf, 72, efi_tps);
    _mav_put_float(buf, 76, efi_exhaust_gas_temperature);
    _mav_put_uint16_t(buf, 80, generator_status);
    _mav_put_uint16_t(buf, 82, efi_status);
    _mav_put_uint8_t(buf, 84, efi_index);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN);
#else
    mavlink_loweheiser_gov_efi_t packet;
    packet.volt_batt = volt_batt;
    packet.curr_batt = curr_batt;
    packet.curr_gen = curr_gen;
    packet.curr_rot = curr_rot;
    packet.fuel_level = fuel_level;
    packet.throttle = throttle;
    packet.runtime = runtime;
    packet.until_maintenance = until_maintenance;
    packet.rectifier_temp = rectifier_temp;
    packet.generator_temp = generator_temp;
    packet.efi_batt = efi_batt;
    packet.efi_rpm = efi_rpm;
    packet.efi_pw = efi_pw;
    packet.efi_fuel_flow = efi_fuel_flow;
    packet.efi_fuel_consumed = efi_fuel_consumed;
    packet.efi_baro = efi_baro;
    packet.efi_mat = efi_mat;
    packet.efi_clt = efi_clt;
    packet.efi_tps = efi_tps;
    packet.efi_exhaust_gas_temperature = efi_exhaust_gas_temperature;
    packet.generator_status = generator_status;
    packet.efi_status = efi_status;
    packet.efi_index = efi_index;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_CRC);
}

/**
 * @brief Encode a loweheiser_gov_efi struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param loweheiser_gov_efi C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_loweheiser_gov_efi_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_loweheiser_gov_efi_t* loweheiser_gov_efi)
{
    return mavlink_msg_loweheiser_gov_efi_pack(system_id, component_id, msg, loweheiser_gov_efi->volt_batt, loweheiser_gov_efi->curr_batt, loweheiser_gov_efi->curr_gen, loweheiser_gov_efi->curr_rot, loweheiser_gov_efi->fuel_level, loweheiser_gov_efi->throttle, loweheiser_gov_efi->runtime, loweheiser_gov_efi->until_maintenance, loweheiser_gov_efi->rectifier_temp, loweheiser_gov_efi->generator_temp, loweheiser_gov_efi->efi_batt, loweheiser_gov_efi->efi_rpm, loweheiser_gov_efi->efi_pw, loweheiser_gov_efi->efi_fuel_flow, loweheiser_gov_efi->efi_fuel_consumed, loweheiser_gov_efi->efi_baro, loweheiser_gov_efi->efi_mat, loweheiser_gov_efi->efi_clt, loweheiser_gov_efi->efi_tps, loweheiser_gov_efi->efi_exhaust_gas_temperature, loweheiser_gov_efi->efi_index, loweheiser_gov_efi->generator_status, loweheiser_gov_efi->efi_status);
}

/**
 * @brief Encode a loweheiser_gov_efi struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param loweheiser_gov_efi C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_loweheiser_gov_efi_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_loweheiser_gov_efi_t* loweheiser_gov_efi)
{
    return mavlink_msg_loweheiser_gov_efi_pack_chan(system_id, component_id, chan, msg, loweheiser_gov_efi->volt_batt, loweheiser_gov_efi->curr_batt, loweheiser_gov_efi->curr_gen, loweheiser_gov_efi->curr_rot, loweheiser_gov_efi->fuel_level, loweheiser_gov_efi->throttle, loweheiser_gov_efi->runtime, loweheiser_gov_efi->until_maintenance, loweheiser_gov_efi->rectifier_temp, loweheiser_gov_efi->generator_temp, loweheiser_gov_efi->efi_batt, loweheiser_gov_efi->efi_rpm, loweheiser_gov_efi->efi_pw, loweheiser_gov_efi->efi_fuel_flow, loweheiser_gov_efi->efi_fuel_consumed, loweheiser_gov_efi->efi_baro, loweheiser_gov_efi->efi_mat, loweheiser_gov_efi->efi_clt, loweheiser_gov_efi->efi_tps, loweheiser_gov_efi->efi_exhaust_gas_temperature, loweheiser_gov_efi->efi_index, loweheiser_gov_efi->generator_status, loweheiser_gov_efi->efi_status);
}

/**
 * @brief Encode a loweheiser_gov_efi struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param loweheiser_gov_efi C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_loweheiser_gov_efi_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_loweheiser_gov_efi_t* loweheiser_gov_efi)
{
    return mavlink_msg_loweheiser_gov_efi_pack_status(system_id, component_id, _status, msg,  loweheiser_gov_efi->volt_batt, loweheiser_gov_efi->curr_batt, loweheiser_gov_efi->curr_gen, loweheiser_gov_efi->curr_rot, loweheiser_gov_efi->fuel_level, loweheiser_gov_efi->throttle, loweheiser_gov_efi->runtime, loweheiser_gov_efi->until_maintenance, loweheiser_gov_efi->rectifier_temp, loweheiser_gov_efi->generator_temp, loweheiser_gov_efi->efi_batt, loweheiser_gov_efi->efi_rpm, loweheiser_gov_efi->efi_pw, loweheiser_gov_efi->efi_fuel_flow, loweheiser_gov_efi->efi_fuel_consumed, loweheiser_gov_efi->efi_baro, loweheiser_gov_efi->efi_mat, loweheiser_gov_efi->efi_clt, loweheiser_gov_efi->efi_tps, loweheiser_gov_efi->efi_exhaust_gas_temperature, loweheiser_gov_efi->efi_index, loweheiser_gov_efi->generator_status, loweheiser_gov_efi->efi_status);
}

/**
 * @brief Send a loweheiser_gov_efi message
 * @param chan MAVLink channel to send the message
 *
 * @param volt_batt [V] Generator Battery voltage.
 * @param curr_batt [A] Generator Battery current.
 * @param curr_gen [A] Current being produced by generator.
 * @param curr_rot [A] Load current being consumed by the UAV (sum of curr_gen and curr_batt)
 * @param fuel_level [l] Generator fuel remaining in litres.
 * @param throttle [%] Throttle Output.
 * @param runtime [s] Seconds this generator has run since it was rebooted.
 * @param until_maintenance [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past due.
 * @param rectifier_temp [degC] The Temperature of the rectifier.
 * @param generator_temp [degC] The temperature of the mechanical motor, fuel cell core or generator.
 * @param efi_batt [V]  EFI Supply Voltage.
 * @param efi_rpm [rpm] Motor RPM.
 * @param efi_pw [ms] Injector pulse-width in miliseconds.
 * @param efi_fuel_flow  Fuel flow rate in litres/hour.
 * @param efi_fuel_consumed [l] Fuel consumed.
 * @param efi_baro [kPa] Atmospheric pressure.
 * @param efi_mat [degC] Manifold Air Temperature.
 * @param efi_clt [degC] Cylinder Head Temperature.
 * @param efi_tps [%] Throttle Position.
 * @param efi_exhaust_gas_temperature [degC] Exhaust gas temperature.
 * @param efi_index  EFI index.
 * @param generator_status  Generator status.
 * @param efi_status  EFI status.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_loweheiser_gov_efi_send(mavlink_channel_t chan, float volt_batt, float curr_batt, float curr_gen, float curr_rot, float fuel_level, float throttle, uint32_t runtime, int32_t until_maintenance, float rectifier_temp, float generator_temp, float efi_batt, float efi_rpm, float efi_pw, float efi_fuel_flow, float efi_fuel_consumed, float efi_baro, float efi_mat, float efi_clt, float efi_tps, float efi_exhaust_gas_temperature, uint8_t efi_index, uint16_t generator_status, uint16_t efi_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN];
    _mav_put_float(buf, 0, volt_batt);
    _mav_put_float(buf, 4, curr_batt);
    _mav_put_float(buf, 8, curr_gen);
    _mav_put_float(buf, 12, curr_rot);
    _mav_put_float(buf, 16, fuel_level);
    _mav_put_float(buf, 20, throttle);
    _mav_put_uint32_t(buf, 24, runtime);
    _mav_put_int32_t(buf, 28, until_maintenance);
    _mav_put_float(buf, 32, rectifier_temp);
    _mav_put_float(buf, 36, generator_temp);
    _mav_put_float(buf, 40, efi_batt);
    _mav_put_float(buf, 44, efi_rpm);
    _mav_put_float(buf, 48, efi_pw);
    _mav_put_float(buf, 52, efi_fuel_flow);
    _mav_put_float(buf, 56, efi_fuel_consumed);
    _mav_put_float(buf, 60, efi_baro);
    _mav_put_float(buf, 64, efi_mat);
    _mav_put_float(buf, 68, efi_clt);
    _mav_put_float(buf, 72, efi_tps);
    _mav_put_float(buf, 76, efi_exhaust_gas_temperature);
    _mav_put_uint16_t(buf, 80, generator_status);
    _mav_put_uint16_t(buf, 82, efi_status);
    _mav_put_uint8_t(buf, 84, efi_index);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI, buf, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_CRC);
#else
    mavlink_loweheiser_gov_efi_t packet;
    packet.volt_batt = volt_batt;
    packet.curr_batt = curr_batt;
    packet.curr_gen = curr_gen;
    packet.curr_rot = curr_rot;
    packet.fuel_level = fuel_level;
    packet.throttle = throttle;
    packet.runtime = runtime;
    packet.until_maintenance = until_maintenance;
    packet.rectifier_temp = rectifier_temp;
    packet.generator_temp = generator_temp;
    packet.efi_batt = efi_batt;
    packet.efi_rpm = efi_rpm;
    packet.efi_pw = efi_pw;
    packet.efi_fuel_flow = efi_fuel_flow;
    packet.efi_fuel_consumed = efi_fuel_consumed;
    packet.efi_baro = efi_baro;
    packet.efi_mat = efi_mat;
    packet.efi_clt = efi_clt;
    packet.efi_tps = efi_tps;
    packet.efi_exhaust_gas_temperature = efi_exhaust_gas_temperature;
    packet.generator_status = generator_status;
    packet.efi_status = efi_status;
    packet.efi_index = efi_index;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI, (const char *)&packet, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_CRC);
#endif
}

/**
 * @brief Send a loweheiser_gov_efi message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_loweheiser_gov_efi_send_struct(mavlink_channel_t chan, const mavlink_loweheiser_gov_efi_t* loweheiser_gov_efi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_loweheiser_gov_efi_send(chan, loweheiser_gov_efi->volt_batt, loweheiser_gov_efi->curr_batt, loweheiser_gov_efi->curr_gen, loweheiser_gov_efi->curr_rot, loweheiser_gov_efi->fuel_level, loweheiser_gov_efi->throttle, loweheiser_gov_efi->runtime, loweheiser_gov_efi->until_maintenance, loweheiser_gov_efi->rectifier_temp, loweheiser_gov_efi->generator_temp, loweheiser_gov_efi->efi_batt, loweheiser_gov_efi->efi_rpm, loweheiser_gov_efi->efi_pw, loweheiser_gov_efi->efi_fuel_flow, loweheiser_gov_efi->efi_fuel_consumed, loweheiser_gov_efi->efi_baro, loweheiser_gov_efi->efi_mat, loweheiser_gov_efi->efi_clt, loweheiser_gov_efi->efi_tps, loweheiser_gov_efi->efi_exhaust_gas_temperature, loweheiser_gov_efi->efi_index, loweheiser_gov_efi->generator_status, loweheiser_gov_efi->efi_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI, (const char *)loweheiser_gov_efi, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_CRC);
#endif
}

#if MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_loweheiser_gov_efi_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float volt_batt, float curr_batt, float curr_gen, float curr_rot, float fuel_level, float throttle, uint32_t runtime, int32_t until_maintenance, float rectifier_temp, float generator_temp, float efi_batt, float efi_rpm, float efi_pw, float efi_fuel_flow, float efi_fuel_consumed, float efi_baro, float efi_mat, float efi_clt, float efi_tps, float efi_exhaust_gas_temperature, uint8_t efi_index, uint16_t generator_status, uint16_t efi_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, volt_batt);
    _mav_put_float(buf, 4, curr_batt);
    _mav_put_float(buf, 8, curr_gen);
    _mav_put_float(buf, 12, curr_rot);
    _mav_put_float(buf, 16, fuel_level);
    _mav_put_float(buf, 20, throttle);
    _mav_put_uint32_t(buf, 24, runtime);
    _mav_put_int32_t(buf, 28, until_maintenance);
    _mav_put_float(buf, 32, rectifier_temp);
    _mav_put_float(buf, 36, generator_temp);
    _mav_put_float(buf, 40, efi_batt);
    _mav_put_float(buf, 44, efi_rpm);
    _mav_put_float(buf, 48, efi_pw);
    _mav_put_float(buf, 52, efi_fuel_flow);
    _mav_put_float(buf, 56, efi_fuel_consumed);
    _mav_put_float(buf, 60, efi_baro);
    _mav_put_float(buf, 64, efi_mat);
    _mav_put_float(buf, 68, efi_clt);
    _mav_put_float(buf, 72, efi_tps);
    _mav_put_float(buf, 76, efi_exhaust_gas_temperature);
    _mav_put_uint16_t(buf, 80, generator_status);
    _mav_put_uint16_t(buf, 82, efi_status);
    _mav_put_uint8_t(buf, 84, efi_index);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI, buf, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_CRC);
#else
    mavlink_loweheiser_gov_efi_t *packet = (mavlink_loweheiser_gov_efi_t *)msgbuf;
    packet->volt_batt = volt_batt;
    packet->curr_batt = curr_batt;
    packet->curr_gen = curr_gen;
    packet->curr_rot = curr_rot;
    packet->fuel_level = fuel_level;
    packet->throttle = throttle;
    packet->runtime = runtime;
    packet->until_maintenance = until_maintenance;
    packet->rectifier_temp = rectifier_temp;
    packet->generator_temp = generator_temp;
    packet->efi_batt = efi_batt;
    packet->efi_rpm = efi_rpm;
    packet->efi_pw = efi_pw;
    packet->efi_fuel_flow = efi_fuel_flow;
    packet->efi_fuel_consumed = efi_fuel_consumed;
    packet->efi_baro = efi_baro;
    packet->efi_mat = efi_mat;
    packet->efi_clt = efi_clt;
    packet->efi_tps = efi_tps;
    packet->efi_exhaust_gas_temperature = efi_exhaust_gas_temperature;
    packet->generator_status = generator_status;
    packet->efi_status = efi_status;
    packet->efi_index = efi_index;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI, (const char *)packet, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_MIN_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_CRC);
#endif
}
#endif

#endif

// MESSAGE LOWEHEISER_GOV_EFI UNPACKING


/**
 * @brief Get field volt_batt from loweheiser_gov_efi message
 *
 * @return [V] Generator Battery voltage.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_volt_batt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field curr_batt from loweheiser_gov_efi message
 *
 * @return [A] Generator Battery current.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_curr_batt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field curr_gen from loweheiser_gov_efi message
 *
 * @return [A] Current being produced by generator.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_curr_gen(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field curr_rot from loweheiser_gov_efi message
 *
 * @return [A] Load current being consumed by the UAV (sum of curr_gen and curr_batt)
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_curr_rot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field fuel_level from loweheiser_gov_efi message
 *
 * @return [l] Generator fuel remaining in litres.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_fuel_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field throttle from loweheiser_gov_efi message
 *
 * @return [%] Throttle Output.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field runtime from loweheiser_gov_efi message
 *
 * @return [s] Seconds this generator has run since it was rebooted.
 */
static inline uint32_t mavlink_msg_loweheiser_gov_efi_get_runtime(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field until_maintenance from loweheiser_gov_efi message
 *
 * @return [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past due.
 */
static inline int32_t mavlink_msg_loweheiser_gov_efi_get_until_maintenance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field rectifier_temp from loweheiser_gov_efi message
 *
 * @return [degC] The Temperature of the rectifier.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_rectifier_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field generator_temp from loweheiser_gov_efi message
 *
 * @return [degC] The temperature of the mechanical motor, fuel cell core or generator.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_generator_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field efi_batt from loweheiser_gov_efi message
 *
 * @return [V]  EFI Supply Voltage.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_efi_batt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field efi_rpm from loweheiser_gov_efi message
 *
 * @return [rpm] Motor RPM.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_efi_rpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field efi_pw from loweheiser_gov_efi message
 *
 * @return [ms] Injector pulse-width in miliseconds.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_efi_pw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field efi_fuel_flow from loweheiser_gov_efi message
 *
 * @return  Fuel flow rate in litres/hour.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_efi_fuel_flow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field efi_fuel_consumed from loweheiser_gov_efi message
 *
 * @return [l] Fuel consumed.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_efi_fuel_consumed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field efi_baro from loweheiser_gov_efi message
 *
 * @return [kPa] Atmospheric pressure.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_efi_baro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field efi_mat from loweheiser_gov_efi message
 *
 * @return [degC] Manifold Air Temperature.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_efi_mat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field efi_clt from loweheiser_gov_efi message
 *
 * @return [degC] Cylinder Head Temperature.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_efi_clt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field efi_tps from loweheiser_gov_efi message
 *
 * @return [%] Throttle Position.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_efi_tps(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field efi_exhaust_gas_temperature from loweheiser_gov_efi message
 *
 * @return [degC] Exhaust gas temperature.
 */
static inline float mavlink_msg_loweheiser_gov_efi_get_efi_exhaust_gas_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field efi_index from loweheiser_gov_efi message
 *
 * @return  EFI index.
 */
static inline uint8_t mavlink_msg_loweheiser_gov_efi_get_efi_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  84);
}

/**
 * @brief Get field generator_status from loweheiser_gov_efi message
 *
 * @return  Generator status.
 */
static inline uint16_t mavlink_msg_loweheiser_gov_efi_get_generator_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  80);
}

/**
 * @brief Get field efi_status from loweheiser_gov_efi message
 *
 * @return  EFI status.
 */
static inline uint16_t mavlink_msg_loweheiser_gov_efi_get_efi_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  82);
}

/**
 * @brief Decode a loweheiser_gov_efi message into a struct
 *
 * @param msg The message to decode
 * @param loweheiser_gov_efi C-struct to decode the message contents into
 */
static inline void mavlink_msg_loweheiser_gov_efi_decode(const mavlink_message_t* msg, mavlink_loweheiser_gov_efi_t* loweheiser_gov_efi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    loweheiser_gov_efi->volt_batt = mavlink_msg_loweheiser_gov_efi_get_volt_batt(msg);
    loweheiser_gov_efi->curr_batt = mavlink_msg_loweheiser_gov_efi_get_curr_batt(msg);
    loweheiser_gov_efi->curr_gen = mavlink_msg_loweheiser_gov_efi_get_curr_gen(msg);
    loweheiser_gov_efi->curr_rot = mavlink_msg_loweheiser_gov_efi_get_curr_rot(msg);
    loweheiser_gov_efi->fuel_level = mavlink_msg_loweheiser_gov_efi_get_fuel_level(msg);
    loweheiser_gov_efi->throttle = mavlink_msg_loweheiser_gov_efi_get_throttle(msg);
    loweheiser_gov_efi->runtime = mavlink_msg_loweheiser_gov_efi_get_runtime(msg);
    loweheiser_gov_efi->until_maintenance = mavlink_msg_loweheiser_gov_efi_get_until_maintenance(msg);
    loweheiser_gov_efi->rectifier_temp = mavlink_msg_loweheiser_gov_efi_get_rectifier_temp(msg);
    loweheiser_gov_efi->generator_temp = mavlink_msg_loweheiser_gov_efi_get_generator_temp(msg);
    loweheiser_gov_efi->efi_batt = mavlink_msg_loweheiser_gov_efi_get_efi_batt(msg);
    loweheiser_gov_efi->efi_rpm = mavlink_msg_loweheiser_gov_efi_get_efi_rpm(msg);
    loweheiser_gov_efi->efi_pw = mavlink_msg_loweheiser_gov_efi_get_efi_pw(msg);
    loweheiser_gov_efi->efi_fuel_flow = mavlink_msg_loweheiser_gov_efi_get_efi_fuel_flow(msg);
    loweheiser_gov_efi->efi_fuel_consumed = mavlink_msg_loweheiser_gov_efi_get_efi_fuel_consumed(msg);
    loweheiser_gov_efi->efi_baro = mavlink_msg_loweheiser_gov_efi_get_efi_baro(msg);
    loweheiser_gov_efi->efi_mat = mavlink_msg_loweheiser_gov_efi_get_efi_mat(msg);
    loweheiser_gov_efi->efi_clt = mavlink_msg_loweheiser_gov_efi_get_efi_clt(msg);
    loweheiser_gov_efi->efi_tps = mavlink_msg_loweheiser_gov_efi_get_efi_tps(msg);
    loweheiser_gov_efi->efi_exhaust_gas_temperature = mavlink_msg_loweheiser_gov_efi_get_efi_exhaust_gas_temperature(msg);
    loweheiser_gov_efi->generator_status = mavlink_msg_loweheiser_gov_efi_get_generator_status(msg);
    loweheiser_gov_efi->efi_status = mavlink_msg_loweheiser_gov_efi_get_efi_status(msg);
    loweheiser_gov_efi->efi_index = mavlink_msg_loweheiser_gov_efi_get_efi_index(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN? msg->len : MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN;
        memset(loweheiser_gov_efi, 0, MAVLINK_MSG_ID_LOWEHEISER_GOV_EFI_LEN);
    memcpy(loweheiser_gov_efi, _MAV_PAYLOAD(msg), len);
#endif
}

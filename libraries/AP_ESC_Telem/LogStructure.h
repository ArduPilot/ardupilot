#pragma once

#include <AP_Logger/LogStructure.h>

#if HAL_WANTS_EDTV2
#define LOG_IDS_FROM_ESC_TELEM                  \
    LOG_ESC_MSG,                                \
    LOG_EDT2_MSG,                               \
    LOG_EDTS_MSG
#else
#define LOG_IDS_FROM_ESC_TELEM                  \
    LOG_ESC_MSG
#endif

// @LoggerMessage: ESC
// @Description: Feedback received from ESCs
// @Field: TimeUS: microseconds since system startup
// @Field: Instance: ESC instance number
// @Field: RPM: reported motor rotation rate
// @Field: RawRPM: reported motor rotation rate without slew adjustment
// @Field: Volt: Perceived input voltage for the ESC
// @Field: Curr: Perceived current through the ESC
// @Field: Temp: ESC temperature in centi-degrees C
// @Field: CTot: current consumed total mAh
// @Field: MotTemp: measured motor temperature in centi-degrees C
// @Field: Err: error rate
struct PACKED log_Esc {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float rpm;
    float raw_rpm;
    float voltage;
    float current;
    int16_t esc_temp;
    float current_tot;
    int16_t motor_temp;
    float error_rate;
};

#if HAL_WANTS_EDTV2
// @LoggerMessage: EDT2
// @Description: Status received from ESCs via Extended DShot telemetry v2
// @Field: TimeUS: microseconds since system startup
// @Field: Instance: ESC instance number
// @Field: Alert: the "alert" status bit
// @Field: Warning: the "warning" status bit
// @Field: Error: the "error" status bit
// @Field: MaxStress: the maximum stress level observed for this ESC, scaled to [0..15]
struct PACKED log_Edt2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint8_t alert;
    uint8_t warning;
    uint8_t error;
    uint8_t max_stress;
};

// @LoggerMessage: EDTS
// @Description: Stress values received from ESCs via Extended DShot telemetry v2
// @Field: TimeUS: microseconds since system startup
// @Field: Instance: ESC instance number
// @Field: Stress: the stress value observed for this ESC, scaled to [0..255]
struct PACKED log_Edts {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint8_t stress;
};
#endif // HAL_WANTS_EDTV2

#if HAL_WANTS_EDTV2
#define LOG_STRUCTURE_FROM_ESC_TELEM \
    { LOG_ESC_MSG, sizeof(log_Esc), \
      "ESC",  "QBffffcfcf", "TimeUS,Instance,RPM,RawRPM,Volt,Curr,Temp,CTot,MotTemp,Err", "s#qqvAOaO%", "F-00--BCB-" , true }, \
    { LOG_EDT2_MSG, sizeof(log_Edt2), \
      "EDT2",  "QBBBBB", "TimeUS,Instance,Alert,Warning,Error,MaxStress", "s#----", "F-----" , true }, \
    { LOG_EDTS_MSG, sizeof(log_Edts), \
      "EDTS",  "QBB", "TimeUS,Instance,Stress", "s#-", "F--", true },
#else
#define LOG_STRUCTURE_FROM_ESC_TELEM \
    { LOG_ESC_MSG, sizeof(log_Esc), \
      "ESC",  "QBffffcfcf", "TimeUS,Instance,RPM,RawRPM,Volt,Curr,Temp,CTot,MotTemp,Err", "s#qqvAOaO%", "F-00--BCB-" , true },
#endif

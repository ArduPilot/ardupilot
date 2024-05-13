#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_ESC_TELEM                  \
    LOG_ESC_MSG,                                \
    LOG_EDT2_MSG

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

enum class log_Edt2_Status : uint8_t {
    HAS_STRESS_DATA = 1U<<0, // true if the message contains up-to-date stress data
    HAS_STATUS_DATA = 1U<<1, // true if the message contains up-to-date status data
    ALERT_BIT   = 1U<<2, // true if the last status had the "alert" bit (e.g. demag)
    WARNING_BIT = 1U<<3, // true if the last status had the "warning" bit (e.g. desync)
    ERROR_BIT   = 1U<<4, // true if the last status had the "error" bit (e.g. stall)
};


// @LoggerMessage: EDT2
// @Description: Status received from ESCs via Extended DShot telemetry v2
// @Field: TimeUS: microseconds since system startup
// @Field: Instance: ESC instance number
// @Field: Stress: current stress level (commutation effort), scaled to [0..255]
// @Field: MaxStress: maximum stress level (commutation effort) since arming, scaled to [0..15]
// @Field: Status: status bits
// @FieldBitmaskEnum: Status: log_Edt2_Status
struct PACKED log_Edt2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    uint8_t stress;
    uint8_t max_stress;
    uint8_t status;
};

#define LOG_STRUCTURE_FROM_ESC_TELEM  \
    { LOG_ESC_MSG, sizeof(log_Esc), \
      "ESC",  "QBffffcfcf", "TimeUS,Instance,RPM,RawRPM,Volt,Curr,Temp,CTot,MotTemp,Err", "s#qqvAOaO%", "F-00--BCB-" , true }, \
    { LOG_EDT2_MSG, sizeof(log_Edt2), \
      "EDT2",  "QBBBB", "TimeUS,Instance,Stress,MaxStress,Status", "s#---", "F----" , true },


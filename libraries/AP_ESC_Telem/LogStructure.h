#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_ESC_TELEM                  \
    LOG_ESC_MSG

// @LoggerMessage: ESC
// @Description: Feedback received from ESCs
// @Field: TimeUS: microseconds since system startup
// @Field: Instance: ESC instance number
// @Field: RPM: reported motor rotation rate
// @Field: Volt: Perceived input voltage for the ESC
// @Field: Curr: Perceived current through the ESC
// @Field: Temp: ESC temperature in centi-degrees C
// @Field: CTot: current consumed total mAh
// @Field: MotTemp: measured motor temperature in centi-degrees C
// @Field: Err: error rate
// @Field: RXErr: RX error rate
struct PACKED log_Esc {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    int32_t rpm;
    float voltage;
    float current;
    int16_t esc_temp;
    float current_tot;
    int16_t motor_temp;
    float error_rate;
    float rx_err_rate;
};

#define LOG_STRUCTURE_FROM_ESC_TELEM \
    { LOG_ESC_MSG, sizeof(log_Esc), \
      "ESC",  "QBeffcfcff", "TimeUS,Instance,RPM,Volt,Curr,Temp,CTot,MotTemp,Err,RXErr", "s#qvAOaO%%", "F-B--BCB--" },

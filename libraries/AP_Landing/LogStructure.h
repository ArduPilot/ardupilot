#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_Landing_config.h"

#define LOG_IDS_FROM_LANDING \
    LOG_DSTL_MSG

#if HAL_LANDING_DEEPSTALL_ENABLED

// @LoggerMessage: DSTL
// @Description: Deepstall Landing data
// @Field: TimeUS: Time since system startup
// @Field: Stg: Deepstall landing stage
// @Field: THdg: Target heading
// @Field: Lat: Landing point latitude
// @Field: Lng: Landing point longitude
// @Field: Alt: Landing point altitude
// @Field: XT: Crosstrack error
// @Field: Travel: Expected travel distance vehicle will travel from this point
// @Field: L1I: L1 controller crosstrack integrator value
// @Field: Loiter: wind estimate loiter angle flown
// @Field: Des: Deepstall steering PID desired value
// @Field: P: Deepstall steering PID Proportional response component
// @Field: I: Deepstall steering PID Integral response component
// @Field: D: Deepstall steering PID Derivative response component

struct PACKED log_DSTL {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t stage;
    float target_heading;
    int32_t target_lat;
    int32_t target_lng;
    int32_t target_alt;
    int16_t crosstrack_error;
    int16_t travel_distance;
    float l1_i;
    int32_t loiter_sum_cd;
    float desired;
    float P;
    float I;
    float D;
};

#define LOG_STRUCTURE_FROM_LANDING        \
    { LOG_DSTL_MSG, sizeof(log_DSTL), \
        "DSTL", "QBfLLeccfeffff", "TimeUS,Stg,THdg,Lat,Lng,Alt,XT,Travel,L1I,Loiter,Des,P,I,D", "s??DUm--------", "F??000--------" , true },
#else
#define LOG_STRUCTURE_FROM_LANDING
#endif

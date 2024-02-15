#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_AVOIDANCE \
    LOG_OA_BENDYRULER_MSG, \
    LOG_OA_DIJKSTRA_MSG, \
    LOG_SIMPLE_AVOID_MSG, \
    LOG_OD_VISGRAPH_MSG

// @LoggerMessage: OABR
// @Description: Object avoidance (Bendy Ruler) diagnostics
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of BendyRuler currently active
// @Field: Act: True if Bendy Ruler avoidance is being used
// @Field: DYaw: Best yaw chosen to avoid obstacle
// @Field: Yaw: Current vehicle yaw
// @Field: DP: Desired pitch chosen to avoid obstacle
// @Field: RChg: True if BendyRuler resisted changing bearing and continued in last calculated bearing
// @Field: Mar: Margin from path to obstacle on best yaw chosen
// @Field: DLt: Destination latitude
// @Field: DLg: Destination longitude
// @Field: DAlt: Desired alt above EKF Origin
// @Field: OLt: Intermediate location chosen for avoidance
// @Field: OLg: Intermediate location chosen for avoidance
// @Field: OAlt: Intermediate alt chosen for avoidance above EKF origin
struct PACKED log_OABendyRuler {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    uint8_t active;
    uint16_t target_yaw;
    uint16_t yaw;
    uint16_t target_pitch;
    bool resist_chg;
    float margin;
    int32_t final_lat;
    int32_t final_lng;
    int32_t final_alt;
    int32_t oa_lat;
    int32_t oa_lng;
    int32_t oa_alt;
};

// @LoggerMessage: OADJ
// @Description: Object avoidance (Dijkstra) diagnostics
// @Field: TimeUS: Time since system startup
// @Field: State: Dijkstra avoidance library state
// @Field: Err: Dijkstra library error condition
// @Field: CurrPoint: Destination point in calculated path to destination
// @Field: TotPoints: Number of points in path to destination
// @Field: DLat: Destination latitude
// @Field: DLng: Destination longitude
// @Field: OALat: Object Avoidance chosen destination point latitude
// @Field: OALng: Object Avoidance chosen destination point longitude
struct PACKED log_OADijkstra {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t state;
    uint8_t error_id;
    uint8_t curr_point;
    uint8_t tot_points;
    int32_t final_lat;
    int32_t final_lng;
    int32_t oa_lat;
    int32_t oa_lng;
};

// @LoggerMessage: SA
// @Description: Simple Avoidance messages
// @Field: TimeUS: Time since system startup
// @Field: State: True if Simple Avoidance is active
// @Field: DVelX: Desired velocity, X-Axis (Velocity before Avoidance)
// @Field: DVelY: Desired velocity, Y-Axis (Velocity before Avoidance)
// @Field: DVelZ: Desired velocity, Z-Axis (Velocity before Avoidance)
// @Field: MVelX: Modified velocity, X-Axis (Velocity after Avoidance)
// @Field: MVelY: Modified velocity, Y-Axis (Velocity after Avoidance)
// @Field: MVelZ: Modified velocity, Z-Axis (Velocity after Avoidance)
// @Field: Back: True if vehicle is backing away
struct PACKED log_SimpleAvoid {
  LOG_PACKET_HEADER;
  uint64_t time_us;
  uint8_t state;
  float desired_vel_x;
  float desired_vel_y;
  float desired_vel_z;
  float modified_vel_x;
  float modified_vel_y;
  float modified_vel_z;
  uint8_t backing_up;
};

// @LoggerMessage: OAVG
// @Description: Object avoidance path planning visgraph points
// @Field: TimeUS: Time since system startup
// @Field: version: Visgraph version, increments each time the visgraph is re-generated
// @Field: point_num: point number in visgraph
// @Field: Lat: Latitude
// @Field: Lon: longitude
struct PACKED log_OD_Visgraph {
  LOG_PACKET_HEADER;
  uint64_t time_us;
  uint8_t version;
  uint8_t point_num;
  int32_t Lat;
  int32_t Lon;
};

#define LOG_STRUCTURE_FROM_AVOIDANCE \
    { LOG_OA_BENDYRULER_MSG, sizeof(log_OABendyRuler), \
      "OABR","QBBHHHBfLLiLLi","TimeUS,Type,Act,DYaw,Yaw,DP,RChg,Mar,DLt,DLg,DAlt,OLt,OLg,OAlt", "s--ddd-mDUmDUm", "F-------GGBGGB" , true }, \
    { LOG_OA_DIJKSTRA_MSG, sizeof(log_OADijkstra), \
      "OADJ","QBBBBLLLL","TimeUS,State,Err,CurrPoint,TotPoints,DLat,DLng,OALat,OALng", "s----DUDU", "F----GGGG" , true }, \
    { LOG_SIMPLE_AVOID_MSG, sizeof(log_SimpleAvoid), \
      "SA",  "QBffffffB","TimeUS,State,DVelX,DVelY,DVelZ,MVelX,MVelY,MVelZ,Back", "s-nnnnnn-", "F--------", true }, \
     { LOG_OD_VISGRAPH_MSG, sizeof(log_OD_Visgraph), \
      "OAVG", "QBBLL", "TimeUS,version,point_num,Lat,Lon", "s--DU", "F--GG", true},

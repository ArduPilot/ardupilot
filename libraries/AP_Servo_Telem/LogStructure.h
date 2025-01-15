#pragma once

#include "AP_Servo_Telem_config.h"
#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_SERVO_TELEM \
    LOG_CSRV_MSG

// @LoggerMessage: CSRV
// @Description: Servo feedback data
// @Field: TimeUS: Time since system startup
// @Field: Id: Servo number this data relates to
// @Field: Pos: Current servo position
// @Field: Force: Force being applied
// @Field: Speed: Current servo movement speed
// @Field: Pow: Amount of rated power being applied
// @Field: PosCmd: commanded servo position
// @Field: V: Voltage
// @Field: A: Current
// @Field: MotT: motor temperature
// @Field: PCBT: PCB temperature
// @Field: Err: error flags

struct PACKED log_CSRV {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    float position;
    float force;
    float speed;
    uint8_t power_pct;
    float pos_cmd;
    float voltage;
    float current;
    float mot_temp;
    float pcb_temp;
    uint8_t error;
};


#if AP_SERVO_TELEM_ENABLED
#define LOG_STRUCTURE_FROM_SERVO_TELEM \
    { LOG_CSRV_MSG, sizeof(log_CSRV), \
      "CSRV","QBfffBfffffB","TimeUS,Id,Pos,Force,Speed,Pow,PosCmd,V,A,MotT,PCBT,Err", "s#dtk%dvAOO-", "F-000000000-", true },
#else
#define LOG_STRUCTURE_FROM_SERVO_TELEM
#endif

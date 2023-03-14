#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_PICCOLOCAN                 \
    LOG_PICCOLOCAN_ESC_STATUS_A,                \
    LOG_PICCOLOCAN_ESC_STATUS_B,                \
    LOG_PICCOLOCAN_ESC_STATUS_C,                \
    LOG_PICCOLOCAN_ESC_STATUS_D,                \
    LOG_PICCOLOCAN_ESC_WARNING_ERROR,           \
    LOG_PICCOLOCAN_ESC_MOTORSTATUSFLAGS

// @LoggerMessage: PCEA
// @Description: PiccoloCAN Telemetry - StatusA
// @Field: TimeUS: Time since system startup
// @Field: I: ESC instance number
// @Field: Status0: Byte 0 of status message
// @Field: Status1: Byte 1 of status message
// @Field: Status2: Byte 2 of status message
// @Field: Command: 
// @Field: RPM: 
struct PACKED log_PCEA {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint8_t  status1;
    uint8_t  status2;
    uint8_t  status3;
    uint16_t command;
    uint16_t rpm;
};

// @LoggerMessage: PCEB
// @Description: PiccoloCAN Telemetry - StatusB
// @Field: TimeUS: Time since system startup
// @Field: I: ESC instance number
// @Field: Voltage:
// @Field: Current:
// @Field: DutyCycle:
// @Field: ESCTemperature: 
// @Field: MotorTemperature: 
struct PACKED log_PCEB {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint16_t voltage;
    int16_t  current;
    uint16_t dutyCycle;
    int8_t   escTemperature;
    uint8_t  motorTemperature;
};

// @LoggerMessage: PCEC
// @Description: PiccoloCAN Telemetry - StatusC
// @Field: TimeUS: Time since system startup
// @Field: I: ESC instance number
// @Field: Reserved:
// @Field: FetTemperature:
// @Field: PWMFrequency:
// @Field: TimingAdvance: 
struct PACKED log_PCEC {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    int16_t  reserved;
    float    fetTemperature;
    uint16_t pwmFrequency;
    uint16_t timingAdvance;
};

// @LoggerMessage: PCED
// @Description: PiccoloCAN Telemetry - StatusD
// @Field: TimeUS: Time since system startup
// @Field: I: ESC instance number
// @Field: Reserved:
// @Field: DemagAngle:
// @Field: ReservedBytes0:
// @Field: ReservedBytes1:
// @Field: ReservedBytes2:
// @Field: ReservedBytes3:
// @Field: ReservedBytes4:
// @Field: ReservedBytes5:
// @Field: ReservedBytes6:
// @Field: ReservedBytes7:
struct PACKED log_PCED {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    int16_t  reserved;
    uint8_t  demagAngle;
    uint8_t  reservedBytes0;
    uint8_t  reservedBytes1;
    uint8_t  reservedBytes2;
    uint8_t  reservedBytes3;
    uint8_t  reservedBytes4;
    uint8_t  reservedBytes5;
    uint8_t  reservedBytes6;
    uint8_t  reservedBytes7;
};

// @LoggerMessage: PCEW
// @Description: PiccoloCAN Telemetry - WarningError
// @Field: TimeUS: Time since system startup
// @Field: I: ESC instance number
// @Field: Reserved:
// @Field: DemagAngle:
// @Field: ReservedBytes0:
// @Field: ReservedBytes1:
// @Field: ReservedBytes2:
// @Field: ReservedBytes3:
// @Field: ReservedBytes4:
// @Field: ReservedBytes5:
// @Field: ReservedBytes6:
// @Field: ReservedBytes7:
struct PACKED log_PCEW {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    uint8_t  warnings0;
    uint8_t  warnings1;
    uint8_t  errors2;
    uint8_t  errors3;
};

struct PACKED log_PCEM {
  LOG_PACKET_HEADER;
  uint64_t time_us;
  uint8_t  instance;
  uint16_t standbyCause;
  uint16_t disableCause;
  uint16_t offCause;
  uint16_t failedStartCause;
};

#define LOG_STRUCTURE_FROM_PICCOLOCAN \
    { LOG_PICCOLOCAN_ESC_STATUS_A, sizeof(log_PCEA), \
      "PCEA", "QBBBBHH", "TimeUS,I,S1,S2,S3,cmd,rpm", "s#-----", "F------", true}, \
    { LOG_PICCOLOCAN_ESC_STATUS_B, sizeof(log_PCEB), \
      "PCEB", "QBHhHbB", "TimeUS,I,volt,curr,duty,escTemp,motorTemp", "s#-A%OO", "F-AAA--", true}, \
    { LOG_PICCOLOCAN_ESC_STATUS_C, sizeof(log_PCEC), \
      "PCEC", "QBhfHH", "TimeUS,I,res,fetTemp,pwmFreq,timingAdv", "s#-Oz-", "F----A", true},   \
    { LOG_PICCOLOCAN_ESC_STATUS_D, sizeof(log_PCED), \
      "PCED", "QBhBBBBBBBB", "TimeUS,I,res,demagAng,r0,r1,r2,r3,r4,r5,r6,r7", "s#----------", "F-----------", true}, \
    { LOG_PICCOLOCAN_ESC_WARNING_ERROR, sizeof(log_PCEW), \
      "PCEW", "QBBBBB", "TimeUS,I,W0,W1,E2,E3", "s#----", "F-----", true}, \
    { LOG_PICCOLOCAN_ESC_MOTORSTATUSFLAGS, sizeof(log_PCEM), \
      "PCEM", "QBHHHH", "TimeUS,I,standbyFlag,disableFlag,offFlag,failedStartFlag", "s#----", "F-----", true}
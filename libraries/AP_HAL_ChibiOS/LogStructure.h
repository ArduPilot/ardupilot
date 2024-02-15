#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_HAL_CHIBIOS \
    LOG_MON_MSG,                 \
    LOG_WDOG_MSG

// @LoggerMessage: MON
// @Description: Main loop performance monitoring message.
// @Field: TimeUS: Time since system startup
// @Field: Dly: Loop delay so far
// @Field: Tsk: Current task
// @Field: IErr: Internal error mask
// @Field: IErrCnt: Count of internal error occurances
// @Field: IErrLn: Internal Error line
// @Field: MM: MAVLink message currently being processed
// @Field: MC: MAVLink command currently being processed
// @Field: SmLn: If semaphore taken, line of semaphore take call
// @Field: SPICnt: count of SPI transactions
// @Field: I2CCnt: count of i2c transactions
struct PACKED log_MON {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t loop_delay;
    int8_t current_task;
    uint32_t internal_error_mask;
    uint16_t internal_error_count;
    uint16_t internal_error_line;
    uint16_t mavmsg;
    uint16_t mavcmd;
    uint16_t semline;
    uint32_t spicnt;
    uint32_t i2ccnt;
};

// @LoggerMessage: WDOG
// @Description: Watchdog diagnostics
// @Field: TimeUS: Time since system startup
// @Field: Tsk: current task number
// @Field: IE: internal error mast
// @Field: IEC: internal error count
// @Field: IEL: line internal error was raised on
// @Field: MvMsg: mavlink message being acted on
// @Field: MvCmd: mavlink command being acted on
// @Field: SmLn: line semaphore was taken on
// @Field: FL: fault_line
// @Field: FT: fault_type
// @Field: FA: fault address
// @Field: FP: fault thread priority
// @Field: ICSR: ICS regiuster
// @Field: LR: long return address
// @Field: TN: Thread name
struct PACKED log_WDOG {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int8_t scheduler_task;
    uint32_t internal_errors;
    uint16_t internal_error_count;
    uint16_t internal_error_last_line;
    uint16_t last_mavlink_msgid;
    uint16_t last_mavlink_cmd;
    uint16_t semaphore_line;
    uint16_t fault_line;
    uint16_t fault_type;
    uint32_t fault_addr;
    uint8_t fault_thd_prio;
    uint32_t fault_icsr;
    uint32_t fault_lr;
    char thread_name4[4];
};

#define LOG_STRUCTURE_FROM_HAL_CHIBIOS                                  \
    { LOG_MON_MSG, sizeof(log_MON),                                     \
      "MON","QIbIHHHHHII","TimeUS,Dly,Tsk,IErr,IErrCnt,IErrLn,MM,MC,SmLn,SPICnt,I2CCnt", "s----------", "F----------", false }, \
    { LOG_WDOG_MSG, sizeof(log_WDOG),                                   \
     "WDOG","QbIHHHHHHHIBIIn","TimeUS,Tsk,IE,IEC,IEL,MvMsg,MvCmd,SmLn,FL,FT,FA,FP,ICSR,LR,TN", "s--------------", "F--------------", false },

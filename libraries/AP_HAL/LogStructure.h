#pragma once

#include <AP_Logger/LogStructure.h>
#include "UARTDriver.h"

#define LOG_IDS_FROM_HAL \
    LOG_UART_MSG, \
    LOG_PM2_MSG

// @LoggerMessage: UART
// @Description: UART stats
// @Field: TimeUS: Time since system startup
// @Field: I: instance
// @Field: Tx: transmitted data rate bytes per second
// @Field: Rx: received data rate bytes per second, this is all incoming data, it may not all be processed by the driver using this port.
// @Field: RxDp: Data rate of dropped received bytes, ideally should be 0. This is the difference between the received data rate and the processed data rate.
struct PACKED log_UART {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float tx_rate;
    float rx_rate;
    float rx_drop_rate;
};

// @LoggerMessage: PM2
// @Description: True CPU load (idle-hook based; requires BRD_IDLE_STATS=Log)
// @Field: TimeUS: Time since system startup
// @Field: Avg: Average true CPU load since the previous PM2 message
// @Field: Peak: Peak true CPU load since the previous PM2 message
struct PACKED log_PM2 {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float avg_load;
    float peak_load;
};

#if HAL_UART_STATS_ENABLED
#define LOG_STRUCTURE_FROM_HAL_UART                     \
    { LOG_UART_MSG, sizeof(log_UART),                   \
      "UART","QBfff","TimeUS,I,Tx,Rx,RxDp", "s#BBB", "F----" },
#else
#define LOG_STRUCTURE_FROM_HAL_UART
#endif

#if AP_CPU_IDLE_STATS_ENABLED
#define LOG_STRUCTURE_FROM_HAL_CPU_LOAD                 \
    { LOG_PM2_MSG, sizeof(log_PM2),                     \
      "PM2","Qff","TimeUS,Avg,Peak", "s%%", "F00" },
#else
#define LOG_STRUCTURE_FROM_HAL_CPU_LOAD
#endif

#define LOG_STRUCTURE_FROM_HAL                          \
    LOG_STRUCTURE_FROM_HAL_UART                         \
    LOG_STRUCTURE_FROM_HAL_CPU_LOAD

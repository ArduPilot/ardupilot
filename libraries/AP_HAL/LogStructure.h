#pragma once

#include <AP_Logger/LogStructure.h>
#include "UARTDriver.h"

#define LOG_IDS_FROM_HAL \
    LOG_UART_MSG

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

#if HAL_UART_STATS_ENABLED
#define LOG_STRUCTURE_FROM_HAL                          \
    { LOG_UART_MSG, sizeof(log_UART),                   \
      "UART","QBfff","TimeUS,I,Tx,Rx,RxDp", "s#BBB", "F----" },
#else
#define LOG_STRUCTURE_FROM_HAL
#endif

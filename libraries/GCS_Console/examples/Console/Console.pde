// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

//
// Example code for the AP_HAL AVRUARTDriver, based on FastSerial
//
// This code is placed into the public domain.
//

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_Param.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>

#include "simplegcs.h"

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

void flush_console_to_statustext() {
    uint8_t data[50];
    int n = hal.console->backend_read(data, 50);
    if (n > 0) {
        try_send_statustext(MAVLINK_COMM_0, (char*) data, n);
    }
}

void setup(void) {
    /* Allocate large enough buffers on uart0 to support mavlink */
    hal.uart0->begin(115200, 128, 256);

    /* Setup GCS_Mavlink library's comm 0 port. */
    mavlink_comm_0_port = hal.uart0;
    
    char hello[] = "Hello statustext\r\n";
    try_send_statustext(MAVLINK_COMM_0, hello, strlen(hello));

    hal.console->backend_open();
    hal.console->printf_P(PSTR("Hello hal.console\r\n"));
}

int i = 0;
void loop(void) {
    try_send_message(MAVLINK_COMM_0, MAVLINK_MSG_ID_HEARTBEAT);
    simplegcs_update(MAVLINK_COMM_0);
    flush_console_to_statustext();
    hal.scheduler->delay(500);
}

AP_HAL_MAIN();

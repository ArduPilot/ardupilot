// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

//
// Example code for the AP_HAL AVRUARTDriver, based on FastSerial
//
// This code is placed into the public domain.
//

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_Param.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <GCS_Console.h>

#include "simplegcs.h"

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void flush_console_to_statustext() {
    uint8_t data[50];
    int n = hal.console->backend_read(data, 50);
    if (n > 0) {
        try_send_statustext(MAVLINK_COMM_0, (char*) data, n);
    }
}

void console_loopback() {
    int a = hal.console->available();
    if (a > 0) {
        hal.console->print("Console loopback:");
        int r = hal.console->read();
        while (r > 0) {
            hal.console->write( (uint8_t) r );
            r = hal.console->read();
        }
        hal.console->println();
    }   
}

void setup(void) {
    /* Allocate large enough buffers on uartA to support mavlink */
    hal.uartA->begin(115200, 128, 256);

    /* Setup GCS_Mavlink library's comm 0 port. */
    mavlink_comm_0_port = hal.uartA;
    
    char hello[] = "Hello statustext\r\n";
    try_send_statustext(MAVLINK_COMM_0, hello, strlen(hello));

    hal.console->backend_open();
    hal.console->printf_P(PSTR("Hello hal.console\r\n"));
}

int i = 0;
void loop(void) {
    try_send_message(MAVLINK_COMM_0, MAVLINK_MSG_ID_HEARTBEAT);
    simplegcs_update(MAVLINK_COMM_0);
    // flush_console_to_statustext();
    gcs_console_send(MAVLINK_COMM_0);

    console_loopback();
    hal.scheduler->delay(100);
}

AP_HAL_MAIN();

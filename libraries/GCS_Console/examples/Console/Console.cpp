// This code is placed into the public domain.

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <GCS_Console/GCS_Console.h>

#include "simplegcs.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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
        hal.console->printf("Console loopback:");
        int r = hal.console->read();
        while (r > 0) {
            hal.console->write( (uint8_t) r );
            r = hal.console->read();
        }
        hal.console->printf("\n");
    }   
}

void setup(void) {
    /* Allocate large enough buffers on uartA to support mavlink */
    hal.uartA->begin(115200, 128, 256);

    /* Setup GCS_Mavlink library's comm 0 port. */
    mavlink_comm_port[0] = hal.uartA;
    
    char hello[] = "Hello statustext\r\n";
    try_send_statustext(MAVLINK_COMM_0, hello, strlen(hello));

    hal.console->backend_open();
    hal.console->printf("Hello hal.console\r\n");
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

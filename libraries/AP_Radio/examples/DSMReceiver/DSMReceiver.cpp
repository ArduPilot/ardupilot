/*
 *  DSM receiver example code for Cypress radio
 *
 * With thanks to the SuperBitRF project and their excellent cyrf6936 dongle:
 *  https://1bitsquared.com/products/superbit-usbrf-dongle
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Radio/AP_Radio.h>
#include <stdio.h>

#define debug(fmt, args...)   printf(fmt, ##args)

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Radio radio;

static bool do_bind = false;

void setup()
{
    hal.console->begin(115200);
    debug("RADIO init\n");
    hal.scheduler->delay(1000);
    radio.init();
    if (do_bind) {
        radio.start_recv_bind();
    }
}

void loop()
{
    static AP_Radio::stats stats;
    hal.scheduler->delay(1000);

    AP_Radio::stats new_stats = radio.get_stats();
    debug("recv:%3u bad:%3u to:%3u re:%u N:%2u 1:%4u 1:%4u 3:%4u 4:%4u 5:%4u 6:%4u 7:%4u 8:%4u 14:%u\n",
          new_stats.recv_packets - stats.recv_packets,
          new_stats.bad_packets - stats.bad_packets,
          new_stats.timeouts - stats.timeouts,
          new_stats.recv_errors - stats.recv_errors,
          radio.num_channels(),
          radio.read(0), radio.read(1), radio.read(2), radio.read(3),
          radio.read(4), radio.read(5), radio.read(6), radio.read(7),
          radio.read(13));
    stats = new_stats;
}

AP_HAL_MAIN();

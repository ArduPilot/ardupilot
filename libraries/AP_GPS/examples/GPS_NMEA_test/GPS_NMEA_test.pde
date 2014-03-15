// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Test for AP_GPS_NMEA
//

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>

#include <AP_HAL.h>
#include <AP_GPS.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_Notify.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_GPS_NMEA NMEA_gps;
GPS *gps = &NMEA_gps;

#define T6 1000000
#define T7 10000000

const uint8_t sirf_to_nmea[] = { 0xa0, 0xa2, // preamble
                                 0x00, 0x18, // message length
                                 0x81, 0x02, // switch to NMEA
                                 0x01, 0x01, // GGA on with checksum
                                 0x00, 0x01, // GLL off
                                 0x00, 0x01, // GSA off
                                 0x00, 0x01, // GSV off
                                 0x01, 0x01, // RMC on with checksum
                                 0x01, 0x01, // VTG on with checksum
                                 0x00, 0x01, // MSS off
                                 0x00, 0x01, // EPE off
                                 0x00, 0x01, // ZPA off
                                 0x00, 0x00, // pad
                                 0x96, 0x00, // 38400
                                 0x01, 0x25, // checksum TBD
                                 0xb0, 0xb3}; // postamble

void setup()
{
    hal.console->println_P(PSTR("GPS_NMEA library test"));
    hal.uartB->begin(38400);

    // try to coerce a SiRF unit that's been traumatized by
    // AP_GPS_AUTO back into NMEA mode so that we can test
    // it.
    for (uint8_t i = 0; i < sizeof(sirf_to_nmea); i++)
        hal.uartB->write(sirf_to_nmea[i]);

    gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_4G);
}

void loop()
{
    gps->update();
    if (gps->new_data) {
        if (gps->fix) {
            hal.console->printf_P(
                PSTR("Lat: %.7f Lon: %.7f Alt: %.2fm GSP: %.2fm/s "
                    "CoG: %d SAT: %d TIM: %u/%lu\r\n"),
                (float)gps->latitude / T7,
                (float)gps->longitude / T7,
                (float)gps->altitude_cm / 100.0,
                (float)gps->ground_speed_cm / 100.0,
                (int)gps->ground_course_cd / 100,
                gps->num_sats,
                gps->time_week,
                gps->time_week_ms);
        } else {
            hal.console->println_P(PSTR("No fix"));
        }
        gps->new_data = false;
    }
}

AP_HAL_MAIN();

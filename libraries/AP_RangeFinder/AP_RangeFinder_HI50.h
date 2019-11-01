#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#include <stdint.h>

/*

DEVICE=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
./Tools/autotest/sim_vehicle.py -v APMrover2 -A "--uartF=uart:$DEVICE" --gdb --debug

 param set RNGFND1_TYPE 28  # HI50
 param set SERIAL5_PROTOCOL 9.000000 # rangefinder
 param set SERIAL5_BAUD 19200
 reboot
*/
// problems:

//  - random glitching to ranom numbers, would need to keep several
// readings to discard them
//  - slow update rate
//  - no attempt to recover if a problem occurs, 'though we are structured for it

class AP_RangeFinder_HI50 : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    // get_reading isn't actually called as we override update()
    bool get_reading(uint16_t &reading_cm) override;

    // line-buffer related stuff:
    uint8_t linebuf[32];
    uint8_t linebuf_len;
    void zero_linebuf() {
        linebuf_len = 0;
        memset(linebuf, '\0', ARRAY_SIZE(linebuf));
    }
    bool fill_linebuf();
    bool get_line_into_linebuf();

    // state-related stuff:
    enum class HI50State {
        ERROR = 30,
        CLOSED = 46,
        OPENING_SEND_OPEN = 48,
        OPENING_WAIT_OK = 49,
        GET_VERSION_START = 50,
        WAITING_FOR_VERSION = 51,
        WORK = 52,
    };
    HI50State hi50_state = HI50State::CLOSED;
    void set_state(HI50State newstate) {
        hi50_state = newstate;
    }
    void handle_state_opening_wait_ok();
    void handle_state_waiting_for_version();
    void handle_state_work();

    enum class WorkState {
        START = 59,
        REQUEST = 60,
        REQUEST_FAST = 61,
        REQUEST_SLOW = 62,
        REQUEST_AUTO = 63,
        EXPECT_READING = 64,
    };
    WorkState workstate;
    void set_workstate(WorkState newstate) {
        workstate = newstate;
    }
    void handle_workstate_expect_reading();

};

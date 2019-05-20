/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

// allow buttons for up to 4 pins
#define AP_BUTTON_NUM_PINS 4

// how often we send reports
#define AP_BUTTON_REPORT_PERIOD_MS 1000

class AP_Button {
public:
    // constructor
    AP_Button(void);

    static const struct AP_Param::GroupInfo var_info[];

    // update button state and send messages, called periodically by main loop
    void update(void);
    
private:
    AP_Int8 enable;
    AP_Int8 pin[AP_BUTTON_NUM_PINS];

    // number of seconds to send change notifications
    AP_Int16 report_send_time;

    // last button press mask
    uint8_t last_mask;

    // when the mask last changed
    uint64_t last_change_time_ms;

    // time of last report
    uint32_t last_report_ms;

    // has the timer been installed?
    bool initialised:1;
    
    // called by timer thread
    void timer_update(void);

    // get current mask
    uint8_t get_mask(void);

    // send a BUTTON_CHANGE report
    void send_report(void);

    // setup pins as pullup input
    void setup_pins();
};


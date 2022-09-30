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

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_BUTTON_ENABLED
#define HAL_BUTTON_ENABLED 1
#endif

#if HAL_BUTTON_ENABLED

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

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Button);

    static const struct AP_Param::GroupInfo var_info[];

    // update button state and send messages, called periodically by main loop
    void update(void);

    static AP_Button *get_singleton(void) {
        return _singleton;
    }

    // get state of a button
    // used by scripting
    bool get_button_state(uint8_t number);

    // check settings are valid
    bool arming_checks(size_t buflen, char *buffer) const;
    
private:

    static AP_Button *_singleton;

    AP_Int8 enable;
    AP_Int8 pin[AP_BUTTON_NUM_PINS];
    AP_Int8 options[AP_BUTTON_NUM_PINS];  // if a pin's bit is set then it uses PWM assertion

    bool is_pwm_input(uint8_t n) const {
        return ((uint8_t)options[n].get() & (1U<<0)) != 0;
    }
    bool is_input_inverted(uint8_t n) const {
        return ((uint8_t)options[n].get() & (1U<<1)) != 0;
    }

    AP_Int16 pin_func[AP_BUTTON_NUM_PINS];  // from the RC_Channel functions

    // number of seconds to send change notifications
    AP_Int16 report_send_time;

    // last button press mask
    uint8_t last_mask;

    // debounced button press mask
    uint8_t debounce_mask;

    // current state of PWM pins:
    uint8_t pwm_state;
    uint8_t tentative_pwm_state;  // for debouncing
    uint64_t pwm_start_debounce_ms;

    // mask indicating which action was most recent taken for pins
    uint8_t state_actioned_mask;

    // pwm sources are used when using PWM on the input
    AP_HAL::PWMSource pwm_pin_source[AP_BUTTON_NUM_PINS];

    // state from the timer interrupt:
    HAL_Semaphore last_debounced_change_ms_sem;
    // last time GPIO pins were debounced:
    uint64_t last_debounced_change_ms;

    // time at least last debounce changes across both PWM and GPIO
    // pins was detected:
    uint64_t last_debounce_ms;

    // when the mask last changed
    uint64_t last_change_time_ms;

    // when button change events were last actioned
    uint64_t last_action_time_ms;

    // true if allocated aux functions have been initialised
    bool aux_functions_initialised;
    // call init_aux_function for all used functions
    void run_aux_functions(bool force);

    // time of last report
    uint32_t last_report_ms;

    // has the timer been installed?
    bool initialised:1;
    
    // called by timer thread
    void timer_update(void);

    // get current mask
    uint8_t get_mask(void);

    // send a BUTTON_CHANGE report
    void send_report(void) const;

    // setup pins as pullup input
    void setup_pins();
};

namespace AP {
    AP_Button &button();
};

#endif

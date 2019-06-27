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


#include "AP_Button.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Button::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable button reporting
    // @Description: This enables the button checking module. When this is disabled the parameters for setting button inputs are not visible
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Button, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PIN1
    // @DisplayName: First button Pin
    // @Description: Digital pin number for first button input. 
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    AP_GROUPINFO("PIN1",  1, AP_Button, pin[0], -1),

    // @Param: PIN2
    // @DisplayName: Second button Pin
    // @Description: Digital pin number for second button input. 
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    AP_GROUPINFO("PIN2",  2, AP_Button, pin[1], -1),

    // @Param: PIN3
    // @DisplayName: Third button Pin
    // @Description: Digital pin number for third button input. 
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    AP_GROUPINFO("PIN3",  3, AP_Button, pin[2], -1),

    // @Param: PIN4
    // @DisplayName: Fourth button Pin
    // @Description: Digital pin number for fourth button input. 
    // @User: Standard
    // @Values: -1:Disabled,50:AUXOUT1,51:AUXOUT2,52:AUXOUT3,53:AUXOUT4,54:AUXOUT5,55:AUXOUT6
    AP_GROUPINFO("PIN4",  4, AP_Button, pin[3], -1),

    // @Param: REPORT_SEND
    // @DisplayName: Report send time
    // @Description: The duration in seconds that a BUTTON_CHANGE report is repeatedly sent to the GCS regarding a button changing state. Note that the BUTTON_CHANGE message is MAVLink2 only.
    // @User: Standard
    // @Range: 0 3600
    AP_GROUPINFO("REPORT_SEND", 5, AP_Button, report_send_time, 10),

    AP_GROUPEND    
};


// constructor
AP_Button::AP_Button(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  update and report, called from main loop
 */
void AP_Button::update(void)
{
    if (!enable) {
        return;
    }

    // call setup pins at update rate (5Hz) to allow for runtime parameter change of pins
    setup_pins();

    if (!initialised) {
        initialised = true;

        // get initial mask
        last_mask = get_mask();

        // register 1kHz timer callback
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Button::timer_update, void));        
    }

    if (last_change_time_ms != 0 &&
        (AP_HAL::millis() - last_report_ms) > AP_BUTTON_REPORT_PERIOD_MS &&
        (AP_HAL::millis64() - last_change_time_ms) < report_send_time*1000ULL) {
        // send a change report
        last_report_ms = AP_HAL::millis();

        // send a report to GCS
        send_report();
    }
}

/*
  get current mask
 */
uint8_t AP_Button::get_mask(void)
{
    uint8_t mask = 0;
    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        if (pin[i] == -1) {
            continue;
        }
        mask |= hal.gpio->read(pin[i]) << i;
    }
    return mask;
}

/*
  called at 1kHz to check for button state change
 */
void AP_Button::timer_update(void)
{
    if (!enable) {
        return;
    }
    uint8_t mask = get_mask();
    if (mask != last_mask) {
        last_mask = mask;
        last_change_time_ms = AP_HAL::millis64();
    }
}

/*
  send a BUTTON_CHANGE report to the GCS
 */
void AP_Button::send_report(void)
{
    const mavlink_button_change_t packet{
            time_boot_ms: AP_HAL::millis(),
            last_change_ms: uint32_t(last_change_time_ms),
            state: last_mask
    };
    gcs().send_to_active_channels(MAVLINK_MSG_ID_BUTTON_CHANGE,
                                  (const char *)&packet);
}

/*
  setup the pins as input with pullup. We need pullup to give reliable
  input with a pulldown button
 */
void AP_Button::setup_pins(void)
{
    for (uint8_t i=0; i<AP_BUTTON_NUM_PINS; i++) {
        if (pin[i] == -1) {
            continue;
        }
        hal.gpio->pinMode(pin[i], HAL_GPIO_INPUT);
        // setup pullup
        hal.gpio->write(pin[i], 1);
    }
}

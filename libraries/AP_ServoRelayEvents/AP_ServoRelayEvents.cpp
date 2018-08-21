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
/*
 *   AP_ServoRelayEvents - handle servo and relay MAVLink events
 */


#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_ServoRelayEvents.h"
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

bool AP_ServoRelayEvents::do_set_servo(uint8_t _channel, uint16_t pwm)
{
    SRV_Channel *c = SRV_Channels::srv_channel(_channel-1);
    if (c == nullptr) {
        return false;
    }
    if (c->get_function() != SRV_Channel::k_none) {
        gcs().send_text(MAV_SEVERITY_INFO, "ServoRelayEvent: Channel %d is already in use", _channel);
        return false;
    }
    if (type == EVENT_TYPE_SERVO && 
        channel == _channel) {
        // cancel previous repeat
        repeat = 0;
    }
    c->set_output_pwm(pwm);
    return true;
}

bool AP_ServoRelayEvents::do_set_relay(uint8_t relay_num, uint8_t state)
{
    if (!relay.enabled(relay_num)) {
        return false;
    }
    if (type == EVENT_TYPE_RELAY && 
        channel == relay_num) {
        // cancel previous repeat
        repeat = 0;
    }
    if (state == 1) {
        relay.on(relay_num);
    } else if (state == 0) {
        relay.off(relay_num);
    } else {
        relay.toggle(relay_num);
    }
    return true;
}

bool AP_ServoRelayEvents::do_repeat_servo(uint8_t _channel, uint16_t _servo_value, 
                                          int16_t _repeat, uint16_t _delay_ms)
{
    SRV_Channel *c = SRV_Channels::srv_channel(_channel-1);
    if (c == nullptr) {
        return false;
    }
    if (c->get_function() != SRV_Channel::k_none) {
        gcs().send_text(MAV_SEVERITY_INFO, "ServoRelayEvent: Channel %d is already in use", _channel);
        return false;
    }
    channel = _channel;
    type = EVENT_TYPE_SERVO;

    start_time_ms  = 0;
    delay_ms    = _delay_ms / 2;
    repeat      = _repeat * 2;
    servo_value = _servo_value;
    update_events();
    return true;
}

bool AP_ServoRelayEvents::do_repeat_relay(uint8_t relay_num, int16_t _repeat, uint32_t _delay_ms)
{
    if (!relay.enabled(relay_num)) {
        return false;
    }
    type = EVENT_TYPE_RELAY;
    channel = relay_num;
    start_time_ms  = 0;
    delay_ms        = _delay_ms/2; // half cycle time
    repeat          = _repeat*2;  // number of full cycles
    update_events();
    return true;
}


/*
  update state for MAV_CMD_DO_REPEAT_SERVO and MAV_CMD_DO_REPEAT_RELAY
*/
void AP_ServoRelayEvents::update_events(void)
{
    if (repeat == 0 || (AP_HAL::millis() - start_time_ms) < delay_ms) {
        return;
    }

    start_time_ms = AP_HAL::millis();

    switch (type) {
    case EVENT_TYPE_SERVO: {
        SRV_Channel *c = SRV_Channels::srv_channel(channel-1);
        if (c != nullptr) {
            if (repeat & 1) {
                c->set_output_pwm(c->get_trim());
            } else {
                c->set_output_pwm(servo_value);
            }
        }
        break;
    }
        
    case EVENT_TYPE_RELAY:
        relay.toggle(channel);
        break;
    }
    
    if (repeat > 0) {
        repeat--;
    } else {
        // toggle bottom bit so servos flip in value
        repeat ^= 1;
    }
}

// singleton instance
AP_ServoRelayEvents *AP_ServoRelayEvents::_singleton;

namespace AP {

AP_ServoRelayEvents *servorelayevents()
{
    return AP_ServoRelayEvents::get_singleton();
}

}

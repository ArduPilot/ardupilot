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
	APM_OBC.cpp

	Outback Challenge Failsafe module

*/
#include <AP_HAL.h>
#include <APM_OBC.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo APM_OBC::var_info[] PROGMEM = {
    // @Param: MAN_PIN
    // @DisplayName: Manual Pin
    // @Description: This sets a digital output pin to set high when in manual mode
    // @User: Advanced
    AP_GROUPINFO("MAN_PIN",     0, APM_OBC, _manual_pin,    -1),

    // @Param: HB_PIN
    // @DisplayName: Heartbeat Pin
    // @Description: This sets a digital output pin which is cycled at 10Hz when termination is not activated
    // @User: Advanced
    AP_GROUPINFO("HB_PIN",      1, APM_OBC, _heartbeat_pin, -1),

    // @Param: WP_COMMS
    // @DisplayName: Comms Waypoint
    // @Description: Waypoint number to navigate to on comms loss
    // @User: Advanced
    AP_GROUPINFO("WP_COMMS",    2, APM_OBC, _wp_comms_hold, 0),

    // @Param: GPS_LOSS
    // @DisplayName: GPS Loss Waypoint
    // @Description: Waypoint number to navigate to on GPS lock loss
    // @User: Advanced
    AP_GROUPINFO("WP_GPS_LOSS", 4, APM_OBC, _wp_gps_loss, 0),

    // @Param: TERMINATE
    // @DisplayName: Force Terminate
    // @Description: Can be set in flight to force termination of the heartbeat signal
    // @User: Advanced
    AP_GROUPINFO("TERMINATE",   5, APM_OBC, _terminate, 0),

    // @Param: TERM_ACTION
    // @DisplayName: Terminate action
    // @Description: This can be used to force an action on flight termination. Normally this is handled by an external failsafe board, but you can setup APM to handle it here. If set to 0 (which is the default) then no extra action is taken. If set to the magic value 42 then the plane will deliberately crash itself by setting maximum throws on all surfaces, and zero throttle
    // @User: Advanced
    AP_GROUPINFO("TERM_ACTION", 6, APM_OBC, _terminate_action, 0),

    // @Param: TERM_PIN
    // @DisplayName: Terminate Pin
    // @Description: This sets a digital output pin to set high on flight termination
    // @User: Advanced
    AP_GROUPINFO("TERM_PIN",    7, APM_OBC, _terminate_pin,    -1),

    AP_GROUPEND
};

// access to geofence state
extern bool geofence_breached(void);

// function to change waypoint
extern void change_command(uint8_t cmd_index);

// check for Failsafe conditions. This is called at 10Hz by the main
// ArduPlane code
void
APM_OBC::check(APM_OBC::control_mode mode,
	       uint32_t last_heartbeat_ms,
	       uint32_t last_gps_fix_ms)
{	
	// we always check for fence breach
	if (geofence_breached()) {
		if (!_terminate) {
			hal.console->println_P(PSTR("Fence TERMINATE"));
			_terminate.set(1);
		}
	}
	
	// tell the failsafe board if we are in manual control
	// mode. This tells it to pass through controls from the
	// receiver
	if (_manual_pin != -1) {
		if (_manual_pin != _last_manual_pin) {
			hal.gpio->pinMode(_manual_pin, GPIO_OUTPUT);
			_last_manual_pin = _manual_pin;
		}
		hal.gpio->write(_manual_pin, mode==OBC_MANUAL);
	}

	uint32_t now = hal.scheduler->millis();
	bool gcs_link_ok = ((now - last_heartbeat_ms) < 10000);
	bool gps_lock_ok = ((now - last_gps_fix_ms) < 3000);

	switch (_state) {
	case STATE_PREFLIGHT:
		// we startup in preflight mode. This mode ends when
		// we first enter auto.
		if (mode == OBC_AUTO) {
			hal.console->println_P(PSTR("Starting OBC_AUTO"));
			_state = STATE_AUTO;
		}
		break;

	case STATE_AUTO:
		// this is the normal mode. 
		if (!gcs_link_ok) {
			hal.console->println_P(PSTR("State DATA_LINK_LOSS"));
			_state = STATE_DATA_LINK_LOSS;
			if (_wp_comms_hold) {
				if (_command_index != NULL) {
					_saved_wp = _command_index->get();
				}
				change_command(_wp_comms_hold);
			}
			break;
		}
		if (!gps_lock_ok) {
			hal.console->println_P(PSTR("State GPS_LOSS"));
			_state = STATE_GPS_LOSS;
			if (_wp_gps_loss) {
				if (_command_index != NULL) {
					_saved_wp = _command_index->get();
				}
				change_command(_wp_gps_loss);
			}
			break;
		}
		break;

	case STATE_DATA_LINK_LOSS:
		if (!gps_lock_ok) {
			// losing GPS lock when data link is lost
			// leads to termination
			hal.console->println_P(PSTR("Dual loss TERMINATE"));
			_terminate.set(1);
		} else if (gcs_link_ok) {
			_state = STATE_AUTO;
			hal.console->println_P(PSTR("GCS OK"));
			if (_saved_wp != 0) {
				change_command(_saved_wp);			
				_saved_wp = 0;
			}
		}
		break;

	case STATE_GPS_LOSS:
		if (!gcs_link_ok) {
			// losing GCS link when GPS lock lost
			// leads to termination
			hal.console->println_P(PSTR("Dual loss TERMINATE"));
			_terminate.set(1);
		} else if (gps_lock_ok) {
			hal.console->println_P(PSTR("GPS OK"));
			_state = STATE_AUTO;
			if (_saved_wp != 0) {
				change_command(_saved_wp);			
				_saved_wp = 0;
			}
		}
		break;
	}

	// if we are not terminating or if there is a separate terminate
	// pin configured then toggle the heartbeat pin at 10Hz
	if (_heartbeat_pin != -1 && (_terminate_pin != -1 || !_terminate)) {
		if (_heartbeat_pin != _last_heartbeat_pin) {
			hal.gpio->pinMode(_heartbeat_pin, GPIO_OUTPUT);
			_last_heartbeat_pin = _heartbeat_pin;
		}
		_heartbeat_pin_value = !_heartbeat_pin_value;
		hal.gpio->write(_heartbeat_pin, _heartbeat_pin_value);
	}	

	// set the terminate pin
	if (_terminate_pin != -1) {
		if (_terminate_pin != _last_terminate_pin) {
			hal.gpio->pinMode(_terminate_pin, GPIO_OUTPUT);
			_last_terminate_pin = _terminate_pin;
		}
		hal.gpio->write(_terminate_pin, _terminate?1:0);
	}	
}

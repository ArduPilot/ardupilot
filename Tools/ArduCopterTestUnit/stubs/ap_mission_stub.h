/*
 * ap_mission_stub.h
 *
 *  Created on: 28 mai 2014
 *      Author: valentin
 */

#ifndef AP_MISSION_STUB_H_
#define AP_MISSION_STUB_H_

class AP_Mission
{
public:
	struct Jump_Command {
	        uint16_t target;        // target command id
	        int16_t num_times;      // num times to repeat.  -1 = repeat forever
	    };
	struct Conditional_Delay_Command {
	        float seconds;          // period of delay in seconds
	    };
	struct Conditional_Distance_Command {
	        float meters;           // distance from next waypoint in meters
	    };
	struct Yaw_Command {
	        float angle_deg;        // target angle in degrees (0=north, 90=east)
	        float turn_rate_dps;    // turn rate in degrees / second (0=use default)
	        int8_t direction;       // -1 = ccw, +1 = cw
	        uint8_t relative_angle; // 0 = absolute angle, 1 = relative angle
	    };
	struct Change_Speed_Command {
	        uint8_t speed_type;     // 0=airspeed, 1=ground speed
	        float target_ms;        // target speed in m/s, -1 means no change
	        float throttle_pct;     // throttle as a percentage (i.e. 0 ~ 100), -1 means no change
	    };
	struct Set_Relay_Command {
	        uint8_t num;            // relay number from 1 to 4
	        uint8_t state;          // on = 3.3V or 5V (depending upon board), off = 0V.  only used for do-set-relay, not for do-repeat-relay
	    };
	struct Repeat_Relay_Command {
	        uint8_t num;            // relay number from 1 to 4
	        int16_t repeat_count;   // number of times to trigger the relay
	        float cycle_time;       // cycle time in seconds (the time between peaks or the time the relay is on and off for each cycle?)
	    };
	struct Set_Servo_Command {
	        uint8_t channel;        // servo channel
	        uint16_t pwm;           // pwm value for servo
	    };
	struct Repeat_Servo_Command {
	        uint8_t channel;        // servo channel
	        uint16_t pwm;           // pwm value for servo
	        int16_t repeat_count;   // number of times to move the servo (returns to trim in between)
	        float cycle_time;       // cycle time in seconds (the time between peaks or the time the servo is at the specified pwm value for each cycle?)
	    };
	struct Cam_Trigg_Distance {
	        float meters;           // distance
	    };

	union Content {
	        Jump_Command jump;
	        Conditional_Delay_Command delay;
	        Conditional_Distance_Command distance;
	        Yaw_Command yaw;
	        Change_Speed_Command speed;
	        Set_Relay_Command relay;
	        Repeat_Relay_Command repeat_relay;
	        Set_Servo_Command servo;
	        Repeat_Servo_Command repeat_servo;
	        Cam_Trigg_Distance cam_trigg_dist;
	        Location location;
	        uint8_t bytes[12];
	    };
	typedef struct MISSION_COMMAND {        // this commands position in the command list
		Content content;
	    } Mission_Command;
	uint16_t get_current_nav_index() const {return 1;}
};//AP_Mission

#endif /* AP_MISSION_STUB_H_ */

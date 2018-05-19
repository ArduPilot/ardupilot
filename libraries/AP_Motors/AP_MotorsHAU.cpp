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
 *       AP_MotorsHAU.cpp - ArduSub motors library
 */

#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsHAU.h"

extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_MotorsHAU::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),
    // @Param: 1_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("1_DIRECTION", 1, AP_MotorsHAU, _motor_reverse[0], 1),

    // @Param: 2_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("2_DIRECTION", 2, AP_MotorsHAU, _motor_reverse[1], 1),

    // @Param: 3_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("3_DIRECTION", 3, AP_MotorsHAU, _motor_reverse[2], 1),

    // @Param: 4_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("4_DIRECTION", 4, AP_MotorsHAU, _motor_reverse[3], 1),

    // @Param: 5_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("5_DIRECTION", 5, AP_MotorsHAU, _motor_reverse[4], 1),

    // @Param: 6_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("6_DIRECTION", 6, AP_MotorsHAU, _motor_reverse[5], 1),

    // @Param: 7_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("7_DIRECTION", 7, AP_MotorsHAU, _motor_reverse[6], 1),

    // @Param: 8_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("8_DIRECTION", 8, AP_MotorsHAU, _motor_reverse[7], 1),

    // @Param: FV_CPLNG_K
    // @DisplayName: Forward/vertical to pitch decoupling factor
    // @Description: Used to decouple pitch from forward/vertical motion. 0 to disable, 1.2 normal
    // @Range: 0.0 1.5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FV_CPLNG_K", 9, AP_MotorsHAU, _forwardVerticalCouplingFactor, 1.0),

    // @Param: 9_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("9_DIRECTION", 10, AP_MotorsHAU, _motor_reverse[8], 1),

    // @Param: 10_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("10_DIRECTION", 11, AP_MotorsHAU, _motor_reverse[9], 1),

    // @Param: 11_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("11_DIRECTION", 12, AP_MotorsHAU, _motor_reverse[10], 1),

    // @Param: 12_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("12_DIRECTION", 13, AP_MotorsHAU, _motor_reverse[11], 1),

    AP_GROUPEND
};

void AP_MotorsHAU::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    // hard coded config for supported frames
    //                 ```````Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor       Testing Order
        add_motor_raw_HAU(AP_MOTORS_MOT_1,     1.0f,              1.0f,              	1.0f,           1.0f,                  	 1.0f,           1);
        add_motor_raw_HAU(AP_MOTORS_MOT_2,     1.0f,              1.0f,              	1.0f,          	1.0f,                  	-1.0f,           2);
        add_motor_raw_HAU(AP_MOTORS_MOT_3,     1.0f,              1.0f,                -1.0f,          	1.0f,                  	 1.0f,           3);
        add_motor_raw_HAU(AP_MOTORS_MOT_4,     1.0f,              1.0f,                -1.0f,           1.0f,                  	-1.0f,           4);
	//                 ```Servo #              Roll Factor     	  Pitch Factor    		Yaw Factor      Throttle Factor     Forward Factor    Testing Order   Reverse
		add_servo_HAU	 (1,				   1.0f,			  1.0f,				   	1.0f, 			1.0f,					 1.0f,		 	 5,			false);
		add_servo_HAU	 (2,				  -1.0f,			 -1.0f,				   	1.0f, 			1.0f,					-1.0f,		 	 6,			false);
		add_servo_HAU	 (3,				  -1.0f,			  1.0f,				   -1.0f, 			1.0f,					 1.0f,		 	 7,			false);
		add_servo_HAU	 (4,				   1.0f,			 -1.0f,				   -1.0f, 			1.0f,					-1.0f,		     8,			false);
		

			//servo tilt calculation
	//float s1_angle = 90.0 +  angle_compensation*atan((mtHAU.tRoll + mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown)/(1.1+mtHAU.tYaw + (mtHAU.tForward) + mtHAU.tRoll + mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown))*180/3.142;
    //float s2_angle = 90.0 +  angle_compensation*atan((-mtHAU.tRoll - mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown)/(1.1+mtHAU.tYaw + (mtHAU.tBackward) - mtHAU.tRoll - mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown))*180/3.142;
    //float s3_angle = 90.0 +  angle_compensation*atan((-mtHAU.tRoll + mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown)/(1.1-mtHAU.tYaw + (mtHAU.tForward) - mtHAU.tRoll + mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown))*180/3.142;
    //float s4_angle = 90.0 +  angle_compensation*atan((mtHAU.tRoll - mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown)/(1.1-mtHAU.tYaw + (mtHAU.tBackward) + mtHAU.tRoll - mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown))*180/3.142; 
	
}

// add_servo - quick solution to use servo from motor output library
void AP_MotorsHAU::add_servo(int8_t servo_num, bool reverse)
{
	_servo_reverse[servo_num] = reverse;
	
	//push servo number to after 4 motor as it will be use by HAU
	servo_num += 3;
	
    // ensure valid motor number is provided
    if( servo_num >= 0 && servo_num < AP_MOTORS_MAX_NUM_MOTORS ) {

        // increment number of motors if this motor is being newly motor_enabled
        if( !motor_enabled[servo_num] ) {
            motor_enabled[servo_num] = true;
        }

        // call parent class method
        add_motor_num(servo_num);
    }
}

void AP_MotorsHAU::add_servo_HAU(int8_t servo_num, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, uint8_t testing_order, bool reverse)
{
	servo_num += 3;
    //Parent takes care of enabling output and setting up masks
    add_motor_raw(servo_num, roll_fac, pitch_fac, yaw_fac, testing_order);

    //These are override parameters for an HAU
	_throttle_sfactor[servo_num] = throttle_fac;
    _forward_sfactor[servo_num] = forward_fac;
    _roll_sfactor[servo_num] = roll_fac;
	_pitch_sfactor[servo_num] = pitch_fac;
	_yaw_sfactor[servo_num] = yaw_fac;
}


void AP_MotorsHAU::add_motor_raw_HAU(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, uint8_t testing_order)
{
    //Parent takes care of enabling output and setting up masks
    add_motor_raw(motor_num, roll_fac, pitch_fac, yaw_fac, testing_order);

    //These are additional parameters for an ROV
    _throttle_factor[motor_num] = throttle_fac;
    _forward_factor[motor_num] = forward_fac;
}

// output_min - sends minimum values out to the motors
void AP_MotorsHAU::output_min()
{
    int8_t i;

    // set limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // fill the motor_out[] array for HIL use and send minimum value to each motor
    // ToDo find a field to store the minimum pwm instead of hard coding 1500
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, min_pwm_motor);
        }
    }
}

int16_t AP_MotorsHAU::calc_thrust_to_pwm(float thrust_in) const
{
    return constrain_int16(min_pwm_motor + thrust_in * 400, _throttle_radio_min, _throttle_radio_max);
}

int16_t AP_MotorsHAU::calc_angle_to_pwm(float angle_in) const
{
	int16_t min_angle_servo_pwm = 1100;
	int16_t max_angle_servo_pwm = 1900;
	return linear_interpolate	(min_angle_servo_pwm, max_angle_servo_pwm,
								angle_in,
								0, 180);
}
void AP_MotorsHAU::output_to_motors()
{
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor
	int16_t servo_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the servo
	
    switch (_spool_mode) {
    case SHUT_DOWN:
        // sends minimum values out to the motors
        // set motor output based on thrust requests
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = min_pwm_motor;
				if(i>3) servo_out[i-4] = calc_angle_to_pwm(90);
            }
        }
        break;
    case SPIN_WHEN_ARMED:
        // sends output to motors when armed but not flying
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = min_pwm_motor;
				if(i>3) servo_out[i-4] = calc_angle_to_pwm(90);
            }
        }
        break;
    case SPOOL_UP:
    case THROTTLE_UNLIMITED:
    case SPOOL_DOWN:
        // set motor output based on thrust requests
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
				if(i>3) servo_out[i-4] = calc_angle_to_pwm(_servo_mix_out[i]);
            }
        }
        break;
    }

    // send output to each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, motor_out[i]);
			if(i>3) rc_write(i, servo_out[i-4]);
        }
    }
}

float AP_MotorsHAU::get_current_limit_max_throttle()
{
    return 1.0f;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
void AP_MotorsHAU::output_armed_stabilizing()
{
        uint8_t i;                          // general purpose counter
        float   roll_thrust;                // roll thrust input value, +/- 1.0
        float   pitch_thrust;               // pitch thrust input value, +/- 1.0
        float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
        float   throttle_thrust;            // throttle thrust input value, +/- 1.0
        float   forward_thrust;             // forward thrust input value, +/- 1.0
        //float   lateral_thrust;             // lateral thrust input value, +/- 1.0

        roll_thrust = _roll_in;
        pitch_thrust = _pitch_in;
        yaw_thrust = _yaw_in;
        throttle_thrust = get_throttle_bidirectional();
        forward_thrust = _forward_in;
        //lateral_thrust = _lateral_in;

        float vthrust_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
        float hthrust_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor

        // initialize limits flags
        limit.roll_pitch = false;
        limit.yaw = false;
        limit.throttle_lower = false;
        limit.throttle_upper = false;

        // sanity check throttle is above zero and below current limited throttle
        if (throttle_thrust <= -_throttle_thrust_max) {
            throttle_thrust = -_throttle_thrust_max;
            limit.throttle_lower = true;
        }
        if (throttle_thrust >= _throttle_thrust_max) {
            throttle_thrust = _throttle_thrust_max;
            limit.throttle_upper = true;
        }

		/* float m1_total = sqrt(pow(ctY 	* mtHAU.tYawp + ctF * mtHAU.tForward,  2)  +  pow(ctR*mtHAU.tRoll + ctP*mtHAU.tPitch + ctU*mtHAU.tUp + ctD*mtHAU.tDown, 2));
    float m2_total = sqrt(pow(ctY 	* mtHAU.tYawp + ctF * mtHAU.tBackward, 2)  +  pow(ctR*mtHAU.tRoll + ctP*mtHAU.tPitch + ctU*mtHAU.tUp + ctD*mtHAU.tDown, 2));
    float m3_total = sqrt(pow(-ctY 	* mtHAU.tYawn + ctF * mtHAU.tForward,  2)  +  pow(ctR*mtHAU.tRoll + ctP*mtHAU.tPitch + ctU*mtHAU.tUp + ctD*mtHAU.tDown, 2));
    float m4_total = sqrt(pow(-ctY 	* mtHAU.tYawn + ctF * mtHAU.tBackward, 2)  +  pow(ctR*mtHAU.tRoll + ctP*mtHAU.tPitch + ctU*mtHAU.tUp + ctD*mtHAU.tDown, 2)); */
	
        // calculate vertical thrust for each motor (includes roll, pitch and throttle (up and down)
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                vthrust_out[i] = roll_thrust * _roll_factor[i] +
								pitch_thrust * _pitch_factor[i] +
								throttle_thrust * _throttle_factor[i];

            }
        }

        // calculate linear command for each motor
        // linear factors should be 0.0 or 1.0 for now
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                hthrust_out[i] = yaw_thrust * _yaw_factor[i] +
                                forward_thrust * _forward_factor[i];
            }
        }
		
	//servo tilt calculation
	//float s1_angle = 90.0 +  angle_compensation*atan((mtHAU.tRoll + mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown)/(1.1+mtHAU.tYaw + (mtHAU.tForward) + mtHAU.tRoll + mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown))*180/3.142;
    //float s2_angle = 90.0 +  angle_compensation*atan((-mtHAU.tRoll - mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown)/(1.1+mtHAU.tYaw + (mtHAU.tBackward) - mtHAU.tRoll - mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown))*180/3.142;
    //float s3_angle = 90.0 +  angle_compensation*atan((-mtHAU.tRoll + mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown)/(1.1-mtHAU.tYaw + (mtHAU.tForward) - mtHAU.tRoll + mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown))*180/3.142;
    //float s4_angle = 90.0 +  angle_compensation*atan((mtHAU.tRoll - mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown)/(1.1-mtHAU.tYaw + (mtHAU.tBackward) + mtHAU.tRoll - mtHAU.tPitch - mtHAU.tUp + mtHAU.tDown))*180/3.142; 
	
	//servo constrain angle to 0 deg and 180 deg 
	//s1_angle = constrain_float(s1_angle,0,180);
	//s2_angle = constrain_float(s2_angle,0,180);
	//s3_angle = constrain_float(s3_angle,0,180);
	//s4_angle = constrain_float(s4_angle,0,180);
	
		for (i=0; i<4; i++) {
            if (motor_enabled[i+4]) {
				_servo_vertical_mixing = roll_thrust * _roll_sfactor[i] +
										 pitch_thrust * _pitch_sfactor[i];
								
				_servo_horizontal_mixing =  yaw_thrust * _yaw_sfactor[i] +
											forward_thrust * _forward_sfactor[i];
											
				_servo_mixing = _servo_vertical_mixing / (1.1f + _servo_horizontal_mixing + _servo_vertical_mixing);
								
                _servo_mix_out[i] =  constrain_float(90.0f + angle_compensation * ToDeg(atanf(_servo_mixing)),0.0f,180.0f);
            }
        }

        // Calculate final output for each motor
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
				//_thrust_rpyt_mixing = safe_sqrt(sq(hthrust_out[i])  +  sq(vthrust_out[i]));
				_thrust_rpyt_mixing = norm(hthrust_out[i],vthrust_out[i]);
                _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_mixing,0.0f,1.0f);
            }
        }

    const AP_BattMonitor &battery = AP::battery();

	// Current limiting
    if (_batt_current_max <= 0.0f || !battery.has_current()) {
        return;
    }

    float _batt_current = battery.current_amps();

    float _batt_current_delta = _batt_current - _batt_current_last;

    float loop_interval = 1.0f/_loop_rate;

    float _current_change_rate = _batt_current_delta / loop_interval;

    float predicted_current = _batt_current + (_current_change_rate * loop_interval * 5);

    float batt_current_ratio = _batt_current/_batt_current_max;

    float predicted_current_ratio = predicted_current/_batt_current_max;
    _batt_current_last = _batt_current;

    if (predicted_current > _batt_current_max * 1.5f) {
        batt_current_ratio = 2.5f;
    } else if (_batt_current < _batt_current_max && predicted_current > _batt_current_max) {
        batt_current_ratio = predicted_current_ratio;
    }
    _output_limited += (loop_interval/(loop_interval+_batt_current_time_constant)) * (1 - batt_current_ratio);

    _output_limited = constrain_float(_output_limited, 0.0f, 1.0f);

    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] *= _output_limited;
        }
    }
}

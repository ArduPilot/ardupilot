/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
 
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

#include "Copter.h"

/*

 * control_recovery.pde - init and run calls for attitude re-initialization flight mode

 */


// Ideas from Control_Flip code and found out its working from any initial conditions since AC has good navigation system
// Replaced original sport mode with recovery mode to test it without modifying MAVLink or Mission Planner
// Recovery mode is made to recovery or re-initialize quadrotor's attitude to stable hover State(Up-Right)
// it's fully automatic and sets mode to Auto or Stabilize Mode

#define Recovery_Angle 300 // consider successful recovery when attitude is within 3 degrees from 0 degree
#define Recovery_Rate 30000 // Recovery Angular Rates (Attitude Control Library has Rate Limit)
#define SlowDown_Angle 4500 // Euler Angle Criteria which is for slow rates down 
                            // less than this might show some overshoot but also can work as an automatic speed brake

// Recovery Mode Initialize

bool Copter::recovery_init(bool ignore_checks)

{
    // this mode supposed to work at any situation as it is of stabilize mode
    // but do not allow this mode for underpowered vehicle
    return true;

}

// Recovery Mode Run

void Copter::recovery_run()

{

    // if landed set throttle to zero and exit immediately

    if(ap.land_complete) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false, g.throttle_filt);

        return;
    }


    // To-Do: Arm Motors when detects launch

    // Launch Detection Code Implemented to Stabilize mode

    //To-Do: get attitude and decide which direction to turn to make faster recovery

    //and make vehicle hovering state then set mode to auto if possible

    bool Recovery_Success = false; //Recovery Check Flag

    int32_t roll_angle = ahrs.roll_sensor;

    int32_t pitch_angle = ahrs.pitch_sensor;

    int8_t roll_dir = 0;  // Roll Direction  (Right = 1, Left = -1)

    int8_t pitch_dir = 0; // Pitch Direction (Up = 1, Down = -1)

    //Ardupilot has good navigation filter so this works too without singularity.

    // 1. Using Euler Angles 

    if ( roll_angle >= 18000) {

    roll_dir = -1;

    else if (18000 <  roll_angle || roll_angle < 0 ) {

    roll_dir = 1;

        }

    }

    if (pitch_angle >= 18000) {

    pitch_dir = -1;

    else if (18000 <  pitch_angle || pitch_angle < 0) {

    pitch_dir = 1;

        }
    }

    // commands to recover attitude   

    // control angular rate first

    attitude_control.rate_bf_roll_pitch_yaw(Recovery_Rate * roll_dir, Recovery_Rate * pitch_dir, 0.0);

    // throttle assist (Current : 75 %)

    attitude_control.set_throttle_out(750, false, g.throttle_filt);

    // and then control vehicle attitude

    if (fabsf((float) ahrs.pitch_sensor) <= SlowDown_Angle) {

        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(0.0, 0.0, 0.0);

        attitude_control.rate_bf_roll_pitch_yaw(Recovery_Rate * roll_dir, 0.0 , 0.0);

        attitude_control.set_throttle_out(650, false, g.throttle_filt);


    if (fabsf((float) ahrs.roll_sensor) <= SlowDown_Angle) {

        attitude_control.rate_bf_roll_pitch_yaw(0.0, 0.0, 0.0);

        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(0.0, 0.0, 0.0);

        }   

    }

    else if ( (fabsf((float) ahrs.roll_sensor) <= SlowDown_Angle) && (fabsf((float) ahrs.pitch_sensor) > SlowDown_Angle) )

    {

            attitude_control.angle_ef_roll_pitch_rate_ef_yaw(0.0, 0.0, 0.0);

            attitude_control.rate_bf_roll_pitch_yaw(0.0, Recovery_Rate * pitch_dir , 0.0);

            // pos_control.set_target_to_stopping_point_z();

            attitude_control.set_throttle_out(650, false, g.throttle_filt);

        if (fabsf((float) ahrs.pitch_sensor <= SlowDown_Angle))

        {   

            attitude_control.rate_bf_roll_pitch_yaw(0.0, 0.0, 0.0);

            attitude_control.angle_ef_roll_pitch_rate_ef_yaw(0.0, 0.0, 0.0);

        }

    }

    
    // check attitude values for recovery successful

    if (fabsf(0.0f - (float) ahrs.roll_sensor) <= Recovery_Angle) {

        if (fabsf(0.0f - (float) ahrs.pitch_sensor) <= Recovery_Angle) {

            if (fabsf(0.0f - (float) ahrs.get_gyro().z) <= 5.0f ) {


                    // throttle down and set angular rate 0.

                    // attitude_control.set_throttle_out(500, false, g.throttle_filt);

                    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(0.0, 0.0, 0.0);

                    attitude_control.rate_bf_roll_pitch_yaw(0.0, 0.0, 0.0);

                    Recovery_Success = true;

                }

            }

        }
    
    
    if (Recovery_Success) {

        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(0, 0, 0);

        attitude_control.rate_bf_roll_pitch_yaw(0.0, 0.0, 0.0);

        attitude_control.relax_bf_rate_controller();

        attitude_control.set_throttle_out(600, false, g.throttle_filt);


    // make this parameterized what to do after re-initialization
    // 1. GPS Position Availiable & There's a mission set mode auto

      if (position_ok() && mission.num_commands() > 1) {             
      
      //gcs_send_text_P(SEVERITY_HIGH,PSTR("Auto Mode Engaged"));
      
      set_mode(AUTO);

        }

    // 2. No GPS or Position isn't OK but have to control height, linear velocity (unfinished)

      else {
        // height, linear velocity control code needs here(thinking about not using GPS)
        const Vector3f& curr_pos = inertial_nav.get_position();
        pos_control.set_pos_target(curr_pos);
        
        //gcs_send_text_P(SEVERITY_HIGH,PSTR("Stabilize Mode Engaged"));  
        
        attitude_control.set_throttle_out(500, false, g.throttle_filt);
        
        set_mode(STABILIZE);

        }
    }

}; //End of Recovery Mode

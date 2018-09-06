#include "mode.h"
#include "Rover.h"

void ModeHold::update()
{
    float throttle = 0.0f;
    float steering_out = 0.0f; 

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(throttle);
    }

    // if we have a sail we should let it out and steer into the wind
    if (g2.motors.has_sail()){
        // relax mainsail
        rover.sailboat_set_mainsail(100);
        
        // Only steer into wind if armed and a sterring angle is deffined
        if(hal.util->get_soft_armed() && is_positive(g2.sailboat_hold_angle)){
            if(is_zero(g2.sailboat_hold_angle)){
                // call heading controller
                steering_out = attitude_control.get_steering_out_heading(g2.windvane.get_absolute_wind_direction_rad(),
                                                                        g2.pivot_turn_rate,
                                                                        g2.motors.limit.steer_left,
                                                                        g2.motors.limit.steer_right,
                                                                        rover.G_Dt);
            } else {
                // if hold angle is not zero use sailboat heading controler, this allows aux switch and rudder input tacking, pass the wind direciton as the defualt heading in case of error
                steering_out = attitude_control.get_steering_out_heading(radians(rover.sailboat_calc_heading(g2.windvane.get_absolute_wind_direction_rad()) / 100.0f),
                                                                        g2.pivot_turn_rate,
                                                                        g2.motors.limit.steer_left,
                                                                        g2.motors.limit.steer_right,
                                                                        rover.G_Dt);
            }
        }
    }
    
    // hold position - stop motors and center steering
    g2.motors.set_throttle(throttle);
    g2.motors.set_steering(steering_out  * 4500.0f);
}

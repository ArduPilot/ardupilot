#include "Rover.h"

/*
To Do List
 ----- short term
 - add calibration code for analog wind Vane
 - setup log of relevant sailing variables
 - consider drag vs lift sailing differences, ie upwind sail is like wing, dead down wind sail is like parachute
 - auto mode
 - Set up PID for the mainsail, So that it will hold at a maximum heal value, by sheeting out and/or changing heading (possibly some sort of ratio parameter here, ie only sheet out or only bear away or combination)
 - allow tack on geofence for auto mode use on restricted water
 - add sailing output messages, 'On indiret route','Starting Tack','Tack Complete' ? 

 ----- Long term
 - max speed paramiter and contoller, for maping you may not want to go too fast
 - add wind speed sensor
 - mavlink sailing messages
 - sailing loiter, boat stays inside loiter circle, more like plane than rover, will carry on sailing but stay close to a point, cant slow down too much or will lose steering
 - motor sailing, some boats may also have motor, we need to decide at what point we would be better of just motoring in low wind, or for a tight loiter, or to hit waypoint exactly, or if stuck head to wind, or to reverse esc
 - smart decision making, ie tack on windshifts, what to do if stuck head to wind
 - sailing 'mode' switching ie in light wind you may have larger vmg by pointing lower and going faster, where as it may be better to go high and slow in heavy wind, tricky because very boat dependent
 - some sailing codes track waves to try and 'surf' and to allow tacking on a flat bit, not sure if there is much gain to be had here
 - add some sort of pitch monitoring to prevent nose diving in heavy weather
 - pitch PID for hydrofoils
 - more advanced sail control, ie twist
 - independent sheeting for main and jib
 - wing type sails with 'elevator' control
 - tack on depth sounder info to stop sailing into shallow water on indirect sailing routes
 - add option to do proper tacks, ie tacking on flat spot in the waves, or only try once at a certain speed, or some better method than just changing the desired heading suddenly
*/

// directly set a mainsail value (used for manual modes)
void Rover::sailboat_set_mainsail(float mainsail)
{
    if (!g2.motors.has_sail()) {
        return;
    }
    g2.motors.set_mainsail(mainsail);
}

// update mainsail's desired angle based on wind speed and direction
void Rover::sailboat_update_mainsail()
{
    if (!g2.motors.has_sail()) {
        return;
    }

    // + is wind over starboard side, - is wind over port side, but as the sails are sheeted the same on each side it makes no difrence so take abs
    float wind_dir_apparent = fabsf(g2.windvane.get_apparent_wind_direction_rad());

    // Convert to degrees as all parameters are in degrees and its easyer than converting them all to radians
    wind_dir_apparent = degrees(wind_dir_apparent);
    
    // set the main sail to the ideal angle to the wind
    float mainsail_angle = wind_dir_apparent - g2.sail_angle_ideal;

    // make sure between allowable range
    mainsail_angle = constrain_float(mainsail_angle, g2.sail_angle_min, g2.sail_angle_max);

    // linear interpolate mainsail value (0 to 100) from wind angle mainsail_angle
    float mainsail = linear_interpolate(0.0f, 100.0f, mainsail_angle, g2.sail_angle_min, g2.sail_angle_max);

    g2.motors.set_mainsail(mainsail);
}

// Should we take a indirect navigaion route, either to go upwind or in the future for speed 
bool Rover::sailboat_update_indirect_route(float desired_heading)
{
    if (!g2.motors.has_sail()) {
        return false;
    }
    desired_heading = radians(desired_heading / 100.0f);
    _sailboat_indirect_route = false;
    
    // Check if desired heading is in the no go zone, if it is we can't go direct
    if (fabsf(wrap_PI((g2.windvane.get_absolute_wind_direction_rad() - desired_heading))) <= radians(g2.sail_no_go)){
        _sailboat_indirect_route = true; 
    }  
    
    return _sailboat_indirect_route;
}    

// If we can't sail on the desired heading then we should pick the best heading that we can sail on 
float Rover::sailboat_calc_heading(float desired_heading)
{
    if (!g2.motors.has_sail()) {
        return desired_heading;
    }
    
    desired_heading = radians(desired_heading / 100.0f);

    
    /* 
        Until we get more fancy logic for best posible speed just assume we can sail upwind at the no go angle 
        Just set off on which ever of the no go angles is closeer, once the end destination is within a single tack it will switch back to direct route method 
        This should result in a long leg with a single tack to get to the destination, with tacks on fence.
        
        Need to add some logic to stop it from tacking back towards fence once it has been bounced off, posibly a miniumm distance and time between tacks or something
    */
    
    // left and right no go headings looking upwind
    float left_no_go_heading = wrap_2PI(g2.windvane.get_absolute_wind_direction_rad() + radians(g2.sail_no_go));
    float right_no_go_heading = wrap_2PI(g2.windvane.get_absolute_wind_direction_rad() - radians(g2.sail_no_go));
    
    // Caculate what tack we are on if it has been too long since we knew
    if (_sailboat_current_tack ==  _tack::Unknown || (AP_HAL::millis() - _sailboat_heading_last_run) > 1000){
        if (g2.windvane.get_apparent_wind_direction_rad() < 0){
            _sailboat_current_tack = _tack::Port;
        } else {
            _sailboat_current_tack = _tack::STBD;
        }   
    }
    _sailboat_heading_last_run = AP_HAL::millis();
    
    // Allow force tack from rudder input 
    float steering_in = rover.channel_steer->norm_input();
    if (fabsf(steering_in) > 0.9f && !_sailboat_tack && !_sailboat_tacking){
       
        switch (_sailboat_current_tack){
            case _tack::Unknown: //should never be called but default to port tack       
            case _tack::Port: {
                if (steering_in < -0.9f){ // if were on port a left hand steering input would be a tack
                    _sailboat_tack = true;
                }
                break;
            } 
            case _tack::STBD: {
                if (steering_in > 0.9f){ // if on stbd right hand turn is a tack
                    _sailboat_tack = true;
                }
                break;
            }  
        }        
    }
    
    // Add paramiter for maximum cross track error befoe tack

    // Are we due to tack?
    if (_sailboat_tack){

        // Pick a heading for the new tack 
        switch (_sailboat_current_tack){
            case _tack::Unknown: //should never be called but default to port tack       
            case _tack::Port: {
                _sailboat_new_tack_heading = degrees(right_no_go_heading) * 100.0f;
                _sailboat_current_tack = _tack::STBD;  
                break;
            } 
            case _tack::STBD: {
                _sailboat_new_tack_heading = degrees(left_no_go_heading) * 100.0f;    
                _sailboat_current_tack = _tack::Port;  
                break;
            }  
        }

        _sailboat_tack = false;  
        _sailboat_tacking = true;
        _sailboat_tack_stat_time = AP_HAL::millis();  		
    }
    
    // If were in the process of a tack we should not change the target heading 
    if (_sailboat_tacking){
        // Check if we have tacked round enough or if we have timed out, add some logic to look at aparent wind angle
        if (AP_HAL::millis() - _sailboat_tack_stat_time > 10000.0f || fabsf(wrap_180_cd(_sailboat_new_tack_heading - (degrees(ahrs.yaw_sensor) * 100.0f))) < (20.0f * 100.0f)){
            _sailboat_tacking = false; 
        }
        desired_heading = _sailboat_new_tack_heading;
    } else {
        // Set new heading
         switch (_sailboat_current_tack){
            case _tack::Unknown: //should never be called but default to port tack       
            case _tack::Port: {
                    desired_heading = degrees(left_no_go_heading) * 100.0f;
                break;
            } 
            case _tack::STBD: {
                    desired_heading = degrees(right_no_go_heading) * 100.0f;
                break;
            }  
        }
    }

    return desired_heading;
}

float Rover::sailboat_acro_tack()
{       
    // Wait until tack is completed
    // Check if we have tacked round enough or if we have timed out
    if (_sailboat_tacking){
        if (AP_HAL::millis() - _sailboat_tack_stat_time > 10000.0f || fabsf(wrap_PI(_sailboat_new_tack_heading_rad - ahrs.yaw_sensor)) < radians(5.0f)){
            _sailboat_tacking = false; 
        }
    }    
    
    // intiate tack
    if (_sailboat_tack) {
        // Match the curent angle to the true wind on the new tack 
        _sailboat_new_tack_heading_rad = wrap_2PI(ahrs.yaw + -2.0f * wrap_PI((g2.windvane.get_absolute_wind_direction_rad() - ahrs.yaw_sensor)));
                     
        _sailboat_tack = false;  
        _sailboat_tacking = true;
        _sailboat_tack_stat_time = AP_HAL::millis();       
    }    
    
    return _sailboat_new_tack_heading_rad;
}

// Velocity Made Good, this is the speed we are travling towards the desired destination
// Not sure how usefull this is at this stage, but would be usefull to log
float Rover::sailboat_VMG(float target_heading)
{
    // https://en.wikipedia.org/wiki/Velocity_made_good

    float speed;
    g2.attitude_control.get_forward_speed(speed);

    float vmg = speed * cosf(wrap_PI(target_heading - ahrs.yaw_sensor));

    return vmg;
}

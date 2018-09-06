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
    
    // Use Pid controller to sheet out 
    float pid_offset =  g2.attitude_control.get_sail_out_from_heel(g2.sail_heel_angle_max, G_Dt) * 100.0f;

    mainsail = constrain_float((mainsail+pid_offset), 0.0f ,100.0f);
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
    // add 10 deg padding to try and avoid constant switching between methods
    if (fabsf(wrap_PI((g2.windvane.get_absolute_wind_direction_rad() - desired_heading))) <= radians(g2.sail_no_go + 10.0f)){
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
        Just set off on which ever of the no go angles is on the corect tack, once the end destination is within a single tack it will switch back to direct route method 
        This should result in a long leg with a single tack to get to the destination.
        Tack can be trigerd by geo fence, aux switch and max cross track error
        
        Need to add some logic to stop it from tacking back towards fence once it has been bounced off, posibly a miniumm distance and time between tacks or something
    */
    
    float left_no_go_heading = 0.0f;
    float right_no_go_heading = 0.0f;
    
    // left and right no go headings looking upwind
    if (rover.control_mode == &rover.mode_hold){
        // Use upwind tacking angles
        left_no_go_heading = wrap_2PI(g2.windvane.get_absolute_wind_direction_rad() + radians(g2.sail_no_go));
        right_no_go_heading = wrap_2PI(g2.windvane.get_absolute_wind_direction_rad() - radians(g2.sail_no_go));
    } else {
        // In hold mode use hold angle
        left_no_go_heading = wrap_2PI(g2.windvane.get_absolute_wind_direction_rad() + radians(g2.sailboat_hold_angle));
        right_no_go_heading = wrap_2PI(g2.windvane.get_absolute_wind_direction_rad() - radians(g2.sailboat_hold_angle));
    }

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
    
    // Maximum cross track error before tack, this efectively defines a 'corridor' of width 2*sailboat_auto_xtrack_tack that the boat will stay within, disable if tacking or in hold mode
    if (fabsf(rover.nav_controller->crosstrack_error()) >= g2.sailboat_auto_xtrack_tack && !is_zero(g2.sailboat_auto_xtrack_tack) && !_sailboat_tack && !_sailboat_tacking && rover.control_mode != &rover.mode_hold){
        // Make sure the new tack will reduce the cross track error        
        // If were on starbard tack we a travling towards the left hand boundary
        if (rover.nav_controller->crosstrack_error() > 0 && _sailboat_current_tack == _tack::STBD){
            _sailboat_tack = true;
        }
        // If were on port tack we a travling towards the right hand boundary
        if (rover.nav_controller->crosstrack_error() < 0 && _sailboat_current_tack == _tack::Port){
            _sailboat_tack = true;
        }
    }    
        
    // Are we due to tack?
    // Need some code that will double check that we did tack if we ment to, ie in light wind it may get stuck head to wind, we could program difftent tacking methods depending on wind, ie fast, slow, gibe right round in very light wind, or minimum speed, so bearaway to acellerate abit. 
    // Could add a do_tack routine that takes the current and target heading and gives target rates depending on conditons
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
    
    // If were in the process of a tack we should not change the target heading, (not sure if this is a good idea or not), the target shouldent change but too much while were are tacking, except if the vane provides poor readings as we are tacking, duno
    if (_sailboat_tacking){
        // Check if we have tacked round enough or if we have timed out, add some logic to look at aparent wind angle
        // not sure if the time out is nessisary
        if (AP_HAL::millis() - _sailboat_tack_stat_time > 10000.0f || fabsf(wrap_180_cd(_sailboat_new_tack_heading - ahrs.yaw_sensor)) < (10.0f * 100.0f)){
            _sailboat_tacking = false; 
            // If we timed out and did not reached the desired heading we canot be sure what tack we are on
            if(AP_HAL::millis() - _sailboat_tack_stat_time > 10000.0f){
                _sailboat_current_tack = _tack::Unknown;        
            }    
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
    
    // intiate tack
    if (_sailboat_tack) {
        // Match the curent angle to the true wind on the new tack 
        _sailboat_new_tack_heading_rad = wrap_2PI(ahrs.yaw + 2.0f * wrap_PI((g2.windvane.get_absolute_wind_direction_rad() - ahrs.yaw)));
                     
        _sailboat_tack = false;  
        _sailboat_tacking = true;
        _sailboat_tack_stat_time = AP_HAL::millis();       
    }    
    
    // Wait until tack is completed
    // Check if we have tacked round enough or if we have timed out
    // time out needed for acro as the pilot is not in control while tacking
    if (_sailboat_tacking ){
        if (AP_HAL::millis() - _sailboat_tack_stat_time > 10000.0f || fabsf(wrap_PI(_sailboat_new_tack_heading_rad - ahrs.yaw)) < radians(5.0f)){
            _sailboat_tacking = false; 
        }
    }
    
    return _sailboat_new_tack_heading_rad;
}

float Rover::sailboat_update_rate_max(float rate_max)
{
    if (!g2.motors.has_sail()) {
        return rate_max;
    }
    
    // if were just travling in a 'straight line' reduce the maximum allowed rate to smooth out heading response to wind changes, use normal max rate for tacking
    if(!_sailboat_tack && !_sailboat_tacking){
        rate_max = g2.sailboat_straight_rate;    
    }  
    
    return rate_max;
}

// Velocity Made Good, this is the speed we are travling towards the desired destination
// Not sure how usefull this is at this stage, but would be usefull to log
float Rover::sailboat_VMG(float target_heading)
{
    // https://en.wikipedia.org/wiki/Velocity_made_good

    float speed;
    g2.attitude_control.get_forward_speed(speed);

    float vmg = speed * cosf(wrap_PI(target_heading - ahrs.yaw));

    return vmg;
}

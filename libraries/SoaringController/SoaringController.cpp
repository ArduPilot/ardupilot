#include "SoaringController.h"

extern const AP_HAL::HAL& hal;


// ArduSoar parameters
const AP_Param::GroupInfo SoaringController::var_info[] = {
    // @Param: SOAR_ACTIVE
    // @DisplayName: Is the soaring mode active or not
    // @Description: Toggles the soaring mode on and off
    // @Units: boolean
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ACTIVE", 0, SoaringController, soar_active, 0),
     
    // @Param: THERMAL_VSPEED
    // @DisplayName: Vertical v-speed
    // @Description: Rate of climb to trigger themalling speed
    // @Units: m/s
    // @Range: 0 10
    // @User: Advanced 
    AP_GROUPINFO("VSPEED", 1, SoaringController, thermal_vspeed, 0.7f),

    // @Param: THERMAL_Q1
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for strength
    // @Units: 
    // @Range: 0 10
    // @User: Advanced 
    AP_GROUPINFO("Q1", 2, SoaringController, thermal_q1, 0.001f),
        
    // @Param: THERMAL_Q2
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for position and radius
    // @Units: 
    // @Range: 0 10
    // @User: Advanced 
    AP_GROUPINFO("Q2", 3, SoaringController, thermal_q2, 0.03f),
    
    // @Param: THERMAL_R
    // @DisplayName: Measurement noise
    // @Description: Standard deviation of noise in measurement
    // @Units: 
    // @Range: 0 10
    // @User: Advanced 
    
    AP_GROUPINFO("R", 4, SoaringController, thermal_r, 0.45f),
    
    // @Param: DIST_AHEAD
    // @DisplayName: Measurement noise
    // @Description: Standard deviation of noise in measurement
    // @Units: metres
    // @Range: 0 100
    // @User: Advanced 
    AP_GROUPINFO("DIST_AHEAD", 5, SoaringController, thermal_distance_ahead, 5.0f),
    
    // @Param: MIN_THERMAL_S
    // @DisplayName: Minimum thermalling time
    // @Description: Minimum number of seconds to spend thermalling
    // @Units: seconds
    // @Range: 0 32768
    // @User: Advanced 
    AP_GROUPINFO("MIN_THML_S", 6, SoaringController, min_thermal_s, 20),
    
    // @Param: MIN_CRUISE_S
    // @DisplayName: Minimum cruising time
    // @Description: Minimum number of seconds to spend cruising
    // @Units: seconds
    // @Range: 0 32768
    // @User: Advanced 
    AP_GROUPINFO("MIN_CRSE_S", 7, SoaringController, min_cruise_s, 30),
    
    // @Param: POLAR_CD0
    // @DisplayName: Zero lift drag coef.
    // @Description: Zero lift drag coefficient
    // @Units: Non-dim.
    // @Range: 0 0.5
    // @User: Advanced 
    AP_GROUPINFO("POLAR_CD0", 8, SoaringController, polar_CD0, 0.027),
    
    // @Param: POLAR_B
    // @DisplayName: Induced drag coeffient
    // @Description: Induced drag coeffient
    // @Units: Non-dim.
    // @Range: 0 0.5
    // @User: Advanced 
    AP_GROUPINFO("POLAR_B", 9, SoaringController, polar_B, 0.031),
    
    // @Param: POLAR_K
    // @DisplayName: Cl factor
    // @Description: Cl factor 2*m*g/(rho*S)
    // @Units: m^2 s^-2
    // @Range: 0 0.5
    // @User: Advanced 
    AP_GROUPINFO("POLAR_K", 10, SoaringController, polar_K, 25.6),
    
    // @Param: ALT_MAX
    // @DisplayName: Maximum soaring altitude
    // @Description: Don't thermal any higher than this.
    // @Units: m
    // @Range: 0 1000.0
    // @User: Advanced 
    AP_GROUPINFO("ALT_MAX", 11, SoaringController, alt_max, 350.0),
    
    // @Param: ALT_MIN
    // @DisplayName: Minimum soaring altitude
    // @Description: Don't get any lower than this.
    // @Units: m
    // @Range: 0 1000.0
    // @User: Advanced 
    AP_GROUPINFO("ALT_MIN", 12, SoaringController, alt_min, 50.0),
    
    // @Param: ALT_CUTOFF
    // @DisplayName: Maximum power altitude
    // @Description: Cut off throttle at this alt.
    // @Units: m
    // @Range: 0 1000.0
    // @User: Advanced 
    AP_GROUPINFO("ALT_CUTOFF", 13, SoaringController, alt_cutoff, 250.0),

    AP_GROUPEND
};
    
void SoaringController::get_target(Location &wp)
{
    wp=_prev_update_location;
    location_offset(wp,ekf.X[2],ekf.X[3]);
}
bool SoaringController::suppress_throttle()
{
    if (_throttle_suppressed && (_alt<alt_min)) {
        // Time to throttle up
        _throttle_suppressed=false;
    }
    else if ((!_throttle_suppressed) && (_alt>alt_cutoff)) {
        // Start glide
        _throttle_suppressed=true;
        // Zero the pitch integrator - the nose is currently raised to climb, we need to go back to glide.
        _spdHgt->reset_pitch_I();
        _cruise_start_time_us = AP_HAL::micros64();
        // Reset the filtered vario rate - it is currently elevated due to the climb rate and would otherwise take a while to fall again,
        // leading to false positives.
        _filtered_vario_reading = 0;
    }
          
    return _throttle_suppressed;
}
bool SoaringController::get_throttle_suppressed()
{
    return _throttle_suppressed;
}

void SoaringController::set_throttle_suppressed(bool suppressed)
{
    _throttle_suppressed = suppressed;
}

bool SoaringController::check_thermal_criteria()
{
    return(soar_active && ((AP_HAL::micros64() - _cruise_start_time_us ) > ((unsigned)min_cruise_s * 1e6)) && _filtered_vario_reading > thermal_vspeed && _alt < alt_max && _alt>alt_min);
}
bool SoaringController::check_cruise_criteria()
{
    float thermalability = (ekf.X[0]*exp(-pow(_loiter_rad/ekf.X[1],2)))-EXPECTED_THERMALLING_SINK; 
    if (soar_active && (AP_HAL::micros64() -_thermal_start_time_us) > ((unsigned)min_thermal_s * 1e6) && thermalability < McCready(_alt)) {
        hal.console->printf("Thermal weak, recommend quitting: W %f R %f th %f alt %f Mc %f\n",ekf.X[0],ekf.X[1],thermalability,_alt,McCready(_alt));
        return true;
    }
    else if (soar_active && (_alt>alt_max || _alt<alt_min)) {
        hal.console->printf("Out of allowable altitude range, beginning cruise. Alt = %f\n",_alt);
        return true;
    }
    return false;
}
bool SoaringController::check_init_thermal_criteria()
{
    if (soar_active && (AP_HAL::micros64() - _thermal_start_time_us) < ((unsigned)min_thermal_s * 1e6)) {
        return true;
    }
    return false;
        
}
void SoaringController::init_thermalling()
{
    // Calc filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode
    float r[1][1] = {{(float)pow(thermal_r,2)}};
    float cov_q1 = (float)pow(thermal_q1,2); // State covariance
    float cov_q2 = (float)pow(thermal_q2,2); // State covariance
    float q[N][N] = {{cov_q1, 0, 0, 0},{0, cov_q2, 0, 0},{0,0, cov_q2,0},{0,0,0,cov_q2}};
    float p[N][N] = {{INITIAL_STRENGTH_COVARIANCE, 0,                         0,                           0},
                    {0,                           INITIAL_RADIUS_COVARIANCE, 0,                           0},
                    {0,                           0,                         INITIAL_POSITION_COVARIANCE, 0},
                    {0,                           0,                         0,                           INITIAL_POSITION_COVARIANCE }}; //Covariance matrix
         
    // New state vector filter will be reset. Thermal location is placed in front of a/c 
    float xr[] = {INITIAL_THERMAL_STRENGTH, INITIAL_THERMAL_RADIUS, (float)(thermal_distance_ahead*cos(_ahrs.yaw)), (float)(thermal_distance_ahead*sin(_ahrs.yaw))};

    if (thermal_distance_ahead < 0)
    {
        xr[2] = 0; xr[3] = 0; // Thermal location is placed at current location
        ekf.reset(xr,p,q,r);
        _nsamples = (unsigned) -thermal_distance_ahead * rate_hz;
        if (_nsamples > EKF_MAX_BUFFER_SIZE) _nsamples = EKF_MAX_BUFFER_SIZE;
        int k = _ptr - _nsamples;
        if (k < 0) k += EKF_MAX_BUFFER_SIZE; // wrap around
        for (unsigned i = 0; i < _nsamples; i++)
        {
            ekf.update(_ekf_buffer[k][0], _ekf_buffer[k][1], _ekf_buffer[k][2]);
            k = (k + 1) % EKF_MAX_BUFFER_SIZE;
        }
    } else {
        // Also reset covariance matrix p so filter is not affected by previous data
        ekf.reset(xr,p,q,r);
    }
    float head_sin = sinf(_ahrs.yaw);
    float head_cos = cosf(_ahrs.yaw);
    float xprod = ekf.X[2] * head_cos -  ekf.X[3] * head_sin;
    _sign = xprod <= 0 ? 1.0 : -1.0;
    
    _ahrs.get_position(_prev_update_location);
    _prev_update_time = AP_HAL::micros64();
    _thermal_start_time_us = AP_HAL::micros64();
}
   
void SoaringController::init_cruising()
{
    _cruise_start_time_us = AP_HAL::micros64();
    // Start glide. Will be updated on the next loop.
    _throttle_suppressed = true;
}

void SoaringController::update_thermalling(float loiter_radius)
{
    _loiter_rad = loiter_radius;
    struct Location current_loc;
    _ahrs.get_position(current_loc);

    if (_new_data) {
        float dx = get_offset_north(_prev_update_location, current_loc);  // get distances from previous update
        float dy = get_offset_east(_prev_update_location, current_loc);

        // Wind correction
        Vector3f wind = _ahrs.wind_estimate();
        float dx_w = wind.x * (AP_HAL::micros64() - _prev_update_time) * 1e-6;
        float dy_w = wind.y * (AP_HAL::micros64() - _prev_update_time) * 1e-6;
        dx -= dx_w;
        dy -= dy_w;

        if (0) {
            // Print filter info for debugging
            //hal.console->printf_P(PSTR("%f %f %f "),netto_rate, dx, dy);

            int i;
            for (i=0;i<4;i++) {
                hal.console->printf("%e ",ekf.P[i][i]);
            }
            for (i=0;i<4;i++) {
                hal.console->printf("%e ",ekf.X[i]);
            }
            //hal.console->printf_P(PSTR("%ld %ld %f %f %f %f\n"),current_loc.lng, current_loc.lat, airspeed.get_airspeed(), alt, ahrs.roll, climb_rate_unfilt);
        }
        else {
             // write log - save the data.
            log_tuning.time_us = AP_HAL::micros64(); //hal.util->get_system_clock_ms();
            log_tuning.netto_rate = _vario_reading;
            log_tuning.dx = dx;
            log_tuning.dy = dy;
            log_tuning.x0 = ekf.X[0];
            log_tuning.x1 = ekf.X[1];
            log_tuning.x2 = ekf.X[2];
            log_tuning.x3 = ekf.X[3];
            log_tuning.lat = current_loc.lat;
            log_tuning.lng = current_loc.lng;
            log_tuning.alt = _alt;
            log_tuning.dx_w = dx_w;
            log_tuning.dy_w = dy_w;
            log_tuning.nsamples = _nsamples;
        }
        log_data(); 
        ekf.update(_vario_reading,dx, dy);                              // update the filter
         
        _prev_update_location = current_loc;      // save for next time
        _prev_update_time = AP_HAL::micros64(); // hal.util->get_system_clock_ms();
        _new_data = false;
    }
}
void SoaringController::update_cruising()
{
    // Do nothing
}
void SoaringController::update_vario()
{
    Location current_loc;
    _ahrs.get_position(current_loc);
    _alt = (current_loc.alt-_ahrs.get_home().alt)/100.0f;
    
    uint8_t action = 254;
    uint64_t solve_time = AP_HAL::micros64();
    
    if (!(_alt==_last_alt)) { // if no change in altitude then there will be no update of ekf buffer
    // Both filtered total energy rates and unfiltered are computed for the thermal switching logic and the EKF
        float aspd;
        float roll = _ahrs.roll;
        if (!_ahrs.airspeed_estimate(&aspd)) {
            aspd = 0.5f*(aparm.airspeed_min+aparm.airspeed_max); //
        }
        _aspd_filt = ASPD_FILT * aspd + (1-ASPD_FILT)*_aspd_filt;
        float total_E = _alt+0.5*_aspd_filt*_aspd_filt/GRAVITY_MSS;                                                     // Work out total energy
        float sinkrate = correct_netto_rate(0.0f, (roll+_last_roll)/2, _aspd_filt);                                     // Compute still-air sinkrate
        _vario_reading = (total_E - _last_total_E) / ((AP_HAL::micros64() - _prev_vario_update_time) * 1e-6) + sinkrate; //*1000.0/(hal.util->get_system_clock_ms()-_prev_vario_update_time) + sinkrate; // Unfiltered netto rate
        _filtered_vario_reading = TE_FILT*_vario_reading+(1-TE_FILT)*_filtered_vario_reading;                           // Apply low pass timeconst filter for noise
        _displayed_vario_reading = TE_FILT_DISPLAYED*_vario_reading+(1-TE_FILT_DISPLAYED)*_displayed_vario_reading;
        
        float dx = get_offset_north(_prev_vario_update_location, current_loc);  // get distances from previous update
        float dy = get_offset_east(_prev_vario_update_location, current_loc);
        Vector3f wind = _ahrs.wind_estimate();
        float dx_w = wind.x * (AP_HAL::micros64() - _prev_vario_update_time) * 1e-6;
        float dy_w = wind.y * (AP_HAL::micros64() - _prev_vario_update_time) * 1e-6;
        dx -= dx_w;
        dy -= dy_w;
        _ekf_buffer[_ptr][0] = _vario_reading;
        _ekf_buffer[_ptr][1] = dx;
        _ekf_buffer[_ptr][2] = dy;
        _ptr = (_ptr + 1) % EKF_MAX_BUFFER_SIZE;
                _last_alt = _alt;                                       // Store variables
        _last_roll=roll;
        _last_aspd = aspd;
        _last_total_E = total_E;
        //_vario_update_period_us = AP_HAL::micros64() - _prev_vario_update_time;
        _prev_vario_update_location = current_loc;
        _prev_vario_update_time = AP_HAL::micros64(); //hal.util->get_system_clock_ms();
        _new_data=true;
        
        log_vario_tuning.time_us = AP_HAL::micros64(); //hal.util->get_system_clock_ms();
        log_vario_tuning.aspd_raw = aspd;
        log_vario_tuning.aspd_filt = _aspd_filt;
        log_vario_tuning.alt = _alt;
        log_vario_tuning.roll = roll;
        log_vario_tuning.raw = _vario_reading;
        log_vario_tuning.filt = _filtered_vario_reading;
        log_vario_tuning.wind_x = wind.x;
        log_vario_tuning.wind_y = wind.y;
        log_vario_tuning.dx = dx;
        log_vario_tuning.dy = dy;
        log_vario_tuning.ptr = _ptr;
        log_vario_tuning.action = action;
        log_vario_tuning.head1 = HEAD_BYTE1;
        log_vario_tuning.head2 = HEAD_BYTE2;
        log_vario_tuning.msgid = _msgid2;
        _dataflash->WriteBlock(&log_vario_tuning, sizeof(log_vario_tuning));
    }
}

float SoaringController::correct_netto_rate(float climb_rate, float phi, float aspd) {
    // Remove aircraft sink rate
    float CL0;  // CL0 = 2*W/(rho*S*V^2)
    float C1;   // C1 = CD0/CL0
    float C2;   // C2 = CDi0/CL0 = B*CL0
    float netto_rate;
    float cosphi;
    CL0 = polar_K/(aspd*aspd);  
    C1 = polar_CD0/CL0;  // constant describing expected angle to overcome zero-lift drag
    C2 = polar_B*CL0;    // constant describing expected angle to overcome lift induced drag at zero bank

    cosphi = (1 - phi*phi/2); // first two terms of mclaurin series for cos(phi)
    netto_rate = climb_rate + aspd*(C1 + C2/(cosphi*cosphi));  // effect of aircraft drag removed

    //Remove acceleration effect - needs to be tested.
    //float temp_netto = netto_rate;
    //float dVdt = SpdHgt_Controller->get_VXdot();
    //netto_rate = netto_rate + aspd*dVdt/GRAVITY_MSS;
    //hal.console->printf_P(PSTR("%f %f %f %f\n"),temp_netto,dVdt,netto_rate,barometer.get_altitude());
    return netto_rate;
}
 
float SoaringController::McCready(float alt) {
    return thermal_vspeed; 
    float XP[] = {500, 3000};
    float YP[] = {0, 4};
    int n = 2;
    // Linear interpolation (without extrap)
    if (alt<=XP[0]) { 
        return YP[0];
    }
    else if (alt>=XP[n-1]){
        return YP[n-1];
    }
    else {
        for (int i=0;i<n;i++) {
            if (alt>=XP[i]) {
                return (((alt-XP[i]) * (YP[i+1]-YP[i]) /(XP[i+1]-XP[i])) + YP[i]);
            }     
        }
    }
    return -1.0; // never happens   
 }
        
// log the contents of the log_tuning structure to dataflash
void SoaringController::log_data()
{
    log_tuning.head1 = HEAD_BYTE1;
    log_tuning.head2 = HEAD_BYTE2;
    log_tuning.msgid = _msgid;
    _dataflash->WriteBlock(&log_tuning, sizeof(log_tuning));
}
bool SoaringController::is_active()
{
    return soar_active;
}
    
    

    
    
    
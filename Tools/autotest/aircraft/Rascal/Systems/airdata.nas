var last_time = 0.0;
var last_speed = 0.0;
var speed_sensed = 0.0;
var sensor_step = 1.0;
var speed_filt = 0.0;
var accel_filt = 0.0;

var compute_airspeed_accel = func( speed_filt, dt ) {
    # print ( "computing forward acceleration ", dt );

    var delta_speed = speed_filt - last_speed;
    last_speed = speed_filt;

    var accel = delta_speed / dt;

    return accel;
}


var update_airdata = func( dt ) {
    # crude model of a noisy electronic pitot tube
    sensed_speed = getprop("/velocities/airspeed-kt");
    var r = rand();
    if ( r < 0.3333 ) {
	sensed_speed = sensed_speed - sensor_step;
    } elsif ( r > 0.6666 ) {
	sensed_speed = sensed_speed + sensor_step;
    }

    speed_filt = 0.97 * speed_filt + 0.03 * sensed_speed;

    var sensed_accel = 0.0;
    if ( dt > 0 ) {
	sensed_accel = compute_airspeed_accel( speed_filt, dt );
    }

    accel_filt = 0.97 * accel_filt + 0.03 * sensed_accel;

    setprop("/accelerations/airspeed-ktps", accel_filt);
}

round10 = func(v) {
	if (v == nil) return 0;
	return 0.1*int(v*10);
}

round100 = func(v) {
	if (v == nil) return 0;
	return 0.01*int(v*100);
}

var update_vars = func( dt ) {
    asl_ft = getprop("/position/altitude-ft");
    ground = getprop("/position/ground-elev-ft");
    agl_m = (asl_ft - ground) * 0.3048;

    setprop("/apm/altitude", round10(agl_m));

    setprop("/apm/pitch",   round10(getprop("/orientation/pitch-deg")));
    setprop("/apm/roll",    round10(getprop("/orientation/roll-deg")));
    setprop("/apm/heading", round10(getprop("/orientation/heading-deg")));

    setprop("/apm/aileron",  round100(getprop("/surface-positions/right-aileron-pos-norm"))); 
    setprop("/apm/elevator", round100(getprop("/surface-positions/elevator-pos-norm"))); 
    setprop("/apm/rudder",   round100(getprop("/surface-positions/rudder-pos-norm"))); 
    setprop("/apm/throttle", round10(getprop("/engines/engine/rpm")));

    setprop("/apm/groundspeed", round10(0.514444444*getprop("/instrumentation/gps/indicated-ground-speed-kt")));

    # airspeed-kt is actually in feet per second (FDM NET bug)
    setprop("/apm/airspeed", round10(0.3048*getprop("/velocities/airspeed-kt")));
}

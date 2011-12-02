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

round10 = func(v) {
	if (v == nil) return 0;
	return 0.1*int(v*10);
}

round100 = func(v) {
	if (v == nil) return 0;
	return 0.01*int(v*100);
}

var update_quad = func( ) {
    asl_ft = getprop("/position/altitude-ft");
    ground = getprop("/position/ground-elev-ft");
    agl_m = (asl_ft - ground) * 0.3048;

    setprop("/apm/altitude", round10(agl_m));

    setprop("/apm/pitch",   round10(getprop("/orientation/pitch-deg")));
    setprop("/apm/roll",    round10(getprop("/orientation/roll-deg")));
    setprop("/apm/heading", round10(getprop("/orientation/heading-deg")));

    # airspeed-kt is actually in feet per second (FDM NET bug)
    setprop("/apm/airspeed", round10(0.3048*getprop("/velocities/airspeed-kt")));

    setprop("/apm/motor_right",  round10(getprop("/engines/engine[0]/rpm")/10.0));
    setprop("/apm/motor_left",   round10(getprop("/engines/engine[1]/rpm")/10.0));
    setprop("/apm/motor_front",  round10(getprop("/engines/engine[2]/rpm")/10.0));
    setprop("/apm/motor_back",   round10(getprop("/engines/engine[3]/rpm")/10.0));
}

var main_loop = func {
    update_quad();
    settimer(main_loop, 0);
}


setlistener("/sim/signals/fdm-initialized",
	    func {
		main_loop();
	    });

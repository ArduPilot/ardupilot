void servo_pic()		// Servo operated camera
{
	APM_RC.OutputCh(CAM_SERVO,1500 + (333));	// Camera click, not enough - add more, wring way - put a minus before bracket number (-300)
	delay(250);				// delay
	APM_RC.OutputCh(CAM_SERVO,1500);		// Return servo to mid position
}

void relay_picture()		// basic relay activation
{
	relay_on();
	delay(250);		// 0.25 seconds delay
	relay_off();
}

void throttle_pic()		// pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
{
	g.channel_throttle.radio_out = g.throttle_min;
	if (thr_pic = 10){
		servo_pic();	// triggering method
		thr_pic = 0;
		g.channel_throttle.radio_out = g.throttle_cruise;
	}
	thr_pic++;
}

void distance_pic()		// pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
{
	g.channel_throttle.radio_out = g.throttle_min;
	if (wp_distance < 3){
		servo_pic();	// triggering method
		g.channel_throttle.radio_out = g.throttle_cruise;
	}
}

void NPN_trigger()		// hacked the circuit to run a transistor? use this trigger to send output.
{
	// To Do: Assign pin spare pin for output
	digitalWrite(camtrig, HIGH);
	delay(50);
	digitalWrite(camtrig, LOW);
}

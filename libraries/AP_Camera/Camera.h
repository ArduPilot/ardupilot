// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	Camera.h
/// @brief	Photo or video camera manager, with EEPROM-backed storage of constants.

#ifndef CAMERA_H
#define CAMERA_H

#include <AP_Common.h>

/// @class	Camera
/// @brief	Object managing a Photo or video camera
class Camera{
protected:
	AP_Var_group    _group;		// must be before all vars to keep ctor init order correct

public:
	/// Constructor
	///
	/// @param key      EEPROM storage key for the camera configuration parameters.
	/// @param name     Optional name for the group.
	///
	Camera(AP_Var::Key key, const prog_char_t *name) :
		_group(key, name),
		mode        (&_group, 0, 0, name ? PSTR("MODE")         : 0), // suppress name if group has no name
		trigger_type(&_group, 1, 0, name ? PSTR("TRIGGER_MODE") : 0),
		picture_time	(0),			// waypoint trigger variable
		thr_pic			(0),			// timer variable for throttle_pic
		camtrig			(83),			// PK6 chosen as it not near anything so safer for soldering
//		camera_target	(home),			// use test target for now
		gimbal_type (1),
		keep_cam_trigg_active_cycles (0)
	{}

	// move the camera depending on the camera mode
	void move();

	// single entry point to take pictures
	void trigger_pic();

	// de-activate the trigger after some delay, but without using a delay() function
	void trigger_pic_cleanup();

	// call this from time to time to make sure the correct gimbal type gets choosen
	void update_camera_gimbal_type();

    // set camera orientation target
	void set_target(struct	Location target);

	int picture_time;					// waypoint trigger variable

private:
	uint8_t keep_cam_trigg_active_cycles; // event loop cycles to keep trigger active
	struct	Location camera_target;		// point of interest for the camera to track
//	struct	Location GPS_mark;			// GPS POI for position based triggering
	int thr_pic;						// timer variable for throttle_pic
	int camtrig;						// PK6 chosen as it not near anything so safer for soldering

	AP_Int8		mode;			// 0=do nothing, 1=stabilize, 2=track target, 3=manual, 4=simple stabilize test
	AP_Int8		trigger_type;	// 0=Servo, 1=relay, 2=throttle_off time, 3=throttle_off waypoint 4=transistor
	uint8_t 	gimbal_type;	// 0=pitch & roll (tilt & roll), 1=yaw & pitch(pan & tilt), 2=pitch, roll & yaw (to be added)

	void servo_pic();		// Servo operated camera
	void relay_pic();		// basic relay activation
	void throttle_pic();	// pictures blurry? use this trigger. Turns off the throttle until for # of cycles of medium loop then takes the picture and re-enables the throttle.
	void distance_pic();	// pictures blurry? use this trigger. Turns off the throttle until closer to waypoint then takes the picture and re-enables the throttle.
	void NPN_pic();			// hacked the circuit to run a transistor? use this trigger to send output.

};

#endif /* CAMERA_H */

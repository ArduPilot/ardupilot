#define XTRACK_GAIN 10 					// Amount to compensate for crosstrack (degrees/100 per meter)
#define XTRACK_ENTRY_ANGLE 3000			// Max angle used to correct for track following	degrees*100
#include <GPS.h>						// ArduPilot GPS Library
#include "Waypoints.h"						// ArduPilot Waypoints Library
#include "WProgram.h"

#define T7 10000000

class Navigation {
public:
	Navigation(GPS *withGPS);
	
	void 		update_gps(void);					// called 50 Hz
	void		set_home(Waypoints::WP loc);
	void		set_next_wp(Waypoints::WP loc);
	void		hold_course(int8_t b);				// 1 = hold a current course, 0 disables course hold
	long 		get_distance(Waypoints::WP *loc1, Waypoints::WP *loc2);
	long 		get_bearing(Waypoints::WP *loc1, Waypoints::WP *loc2);
	
	long		bearing;							// deg * 100 : 0 to 360 current desired bearing to navigate
	long		distance;							// meters : distance plane to next waypoint
	long		altitude_above_home;				// meters * 100 relative to home position
	long		total_distance;						// meters : distance between waypoints
	long 		bearing_error; 						// deg * 100 : 18000 to -18000 	
	long 		altitude_error; 					// deg * 100 : 18000 to -18000 	

	int16_t		loiter_sum;
	Waypoints::WP 	home, location, prev_wp, next_wp;

private:
	void 		calc_long_scaling(int32_t lat);
	void 		calc_bearing_error(void);
	void		calc_altitude_error(void);
	void		calc_distance_error(void);
	void 		update_crosstrack(void);
	void 		reset_crosstrack(void);
	int32_t		wrap_360(int32_t error);			// utility

	int16_t		_old_bearing;						// used to track delta on the bearing
	GPS			*_gps;
	long 		_crosstrack_bearing;				// deg * 100 : 0 to 360 desired angle of plane to target
	float		_crosstrack_error;					// deg * 100 : 18000 to -18000  meters we are off trackline
	long 		_hold_course;						// deg * 100 dir of plane
	long 		_target_altitude;			// used for 
	long 		_offset_altitude;			// used for 
	float		_altitude_estimate;
	float 		_scaleLongUp;				// used to reverse longtitude scaling
	float 		_scaleLongDown;				// used to reverse longtitude scaling	
	int16_t		_loiter_delta;
};

// Mission example:

#define WP_RADIUS 5 		// What is the minimum distance to reach a waypoint?
#define ALT_TO_HOLD	-1		// Altitude to hold above home in meters

							// Enter -1 to maintain current altitude when returning to home
// The mission:
float mission[][6] = {
	//	CMD					options	P1		Alt		Lat			Long
	{MAV_CMD_NAV_TAKEOFF,  	0, 		0,  	3.0,	0, 		  	0},					// 1
	{MAV_CMD_NAV_LAND, 	 	0,		0,		0,		0,          0},					// 7 LAND!
};

// Mission example:

#define WP_RADIUS 3 		// What is the minimum distance to reach a waypoint?
#define ALT_TO_HOLD	-1		// Altitude to hold above home in meters
							// Enter -1 to maintain current altitude when returning to home


// The mission:
float mission[][5] = {
{MAV_CMD_NAV_TAKEOFF,           0, 		6, 		0, 			0},					// pitch 20, Altitude meters
{MAV_CMD_NAV_WAYPOINT, 			0, 		6, 		37.776849, -122.405752},		// go to waypoint
{MAV_CMD_CONDITION_YAW, 		180, 	5,	 	1, 			1},					// 180°, 5 seconds, clockwise, relative
{MAV_CMD_NAV_RETURN_TO_LAUNCH,	0, 		0, 		0, 			0},					// no options for RTL
{MAV_CMD_NAV_LAND, 				0, 		0, 		0,          0},		//
};

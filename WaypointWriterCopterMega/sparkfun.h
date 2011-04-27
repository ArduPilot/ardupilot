// Mission example:

#define WP_RADIUS 5 		// What is the minimum distance to reach a waypoint?
#define ALT_TO_HOLD	-1		// Altitude to hold above home in meters

/*							// Enter -1 to maintain current altitude when returning to home
// The mission:
float mission[][6] = {
	//	CMD					options	P1		Alt		Lat			Long
	{MAV_CMD_NAV_TAKEOFF,  	0, 		0,  	3.0,	0, 		  	0},					// 1
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.0, 	40.065219,	-105.209760},		// 2 turn 1
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.0, 	40.064561,	-105.209798},		// 3 turn 2
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.0, 	40.064511,	-105.210402},		// 4 turn 3
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.0, 	40.065167,	-105.210453},		// 5 turn 4 with delay
	{MAV_CMD_NAV_WAYPOINT, 	0, 		5,		3.0, 	40.065189,	-105.210007},		// 6 Land WP with delay
	{MAV_CMD_NAV_LAND, 	 	0,		0,		0,		0,          0},					// 7 LAND!
};
*/
///*
							// Enter -1 to maintain current altitude when returning to home
// The mission:
float mission[][6] = {
	//	CMD					options	P1		Alt		Lat			Long
	{MAV_CMD_NAV_TAKEOFF,  	0, 		0,  	3.0,	0, 		  	0},					// 1
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.0, 	40.065219,	-105.209760},		//  turn 1

	{MAV_CMD_NAV_WAYPOINT, 	0, 		4,		3.0, 	40.064561,	-105.209798},		//  turn 2 pause
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.0, 	40.064508,	-105.209808},		//  turn 2

	{MAV_CMD_NAV_WAYPOINT, 	0, 		4,		3.0, 	40.064507,	-105.210303},		//  turn 3 pause
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.0, 	40.064524,	-105.210464},		//  turn 3

	{MAV_CMD_NAV_WAYPOINT, 	0, 		1,		3.0, 	40.065092,	-105.210483},		//  turn 4 pause
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.0, 	40.065191,	-105.210349},		//  turn 4

	{MAV_CMD_NAV_WAYPOINT, 	0, 		4,		3.0, 	40.065189,	-105.210007},		//  Land WP with delay
	{MAV_CMD_NAV_LAND, 	 	0,		0,		0,		0,          0},					//  LAND!
};
//*/

/*
// The mission:
float mission[][6] = {
	//	CMD					options	P1		Alt		Lat			Long
	{MAV_CMD_NAV_TAKEOFF,  	0, 		0,  	6.0,	0, 		  	0},					// 1
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		6.0, 	40.065219,	-105.209760},		// 2 turn 1
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		6.0, 	40.064561,	-105.209798},		// 3 turn 2
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		6.0, 	40.064511,	-105.210402},		// 4 turn 3
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		6.0, 	40.065167,	-105.210453},		// 5 turn 4 with delay
	{MAV_CMD_NAV_WAYPOINT, 	0, 		5,		6.0, 	40.065189,	-105.210007},		// 6 Land WP with delay
	{MAV_CMD_NAV_LAND, 	 	0,		0,		0,		0,          0},					// 7 LAND!
};
//*/
// Mission example:

#define WP_RADIUS 3 		// What is the minimum distance to reach a waypoint?
#define ALT_TO_HOLD	-1		// Altitude to hold above home in meters
							// Enter -1 to maintain current altitude when returning to home

// The mission:
float mission[][6] = {
	//	CMD					options	P1		Alt		Lat			Long
	{MAV_CMD_NAV_TAKEOFF,  	0, 		0,  	3.2,	0, 		  	0},					// 1
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.2, 	37.716899,	-122.381898},		// 2 turn 1
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.2, 	37.717475,	-122.381394},		// 3 turn 2
	{MAV_CMD_NAV_WAYPOINT, 	0, 		0,		3.2, 	37.717149,	-122.380819},		// 4 turn 3
	{MAV_CMD_NAV_WAYPOINT, 	0, 		2,		3.2, 	37.716592,	-122.381358},		// 5 turn 4 with delay
	{MAV_CMD_NAV_WAYPOINT, 	0, 		5,		3.2, 	37.716752,	-122.381632},		// 6 Land WP with delay
	{MAV_CMD_NAV_LAND, 	 	0,		0,		0,		0,          0},					// 7 LAND!
};


/*
command #: 0 id:16 op:0 p1:0 p2:5007 	p3:377659180 p4:-1224329500
command #: 1 id:22 op:0 p1:0 p2:220 	p3:0 		 p4:0
command #: 2 id:16 op:0 p1:0 p2:220 	p3:377168992 p4:-1223819008
command #: 3 id:16 op:0 p1:0 p2:220 	p3:377174752 p4:-1223813888
command #: 4 id:16 op:0 p1:0 p2:220 	p3:377171488 p4:-1223808256
command #: 5 id:16 op:0 p1:2 p2:220 	p3:377165920 p4:-1223813504
command #: 6 id:16 op:0 p1:5 p2:220 	p3:377167520 p4:-1223816320
command #: 7 id:21 op:0 p1:0 p2:0 		p3:0 		 p4:0

*/


/*
Sparkfun
*/
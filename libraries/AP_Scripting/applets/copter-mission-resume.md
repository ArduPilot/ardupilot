# Mission Resume for Copter applet

This script makes possible to continue a mission from the exact point where a battery failsafe occured. (usefull for spraying, surveying)
It inserts a waypoint at the location where a battery failsafe occurs and stores its waypoint index in a file on the SD card. If the waypoint index stored in that file is non-zero at boot time, the mission will resume at the inserted waypoint.

I tested in SITL with a fair amount of success but implore you to do the same before implementing this on a large agricultural vehicle. There is an underlying assumption that the path from the takeoff location to any point in the mission will be clear of obstacles. If you cannot ensure obstacle clearance, DO NOT use this script!
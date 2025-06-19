This script is intended to allow easy, unpre-planned operation at any location with the protection of a do-land-start autoland sequence for failsafes that accounts for takeoff direction (ie wind direction). Final approach objects must be considered before you launch.

If enabled by AUTOLAND_ENABLE =1, setups up an autotakeoff waypoint as first waypoint and upon Arming , adds mission items consisting of:  DO_LAND_START,Final Approach WP opposite bearing from HOME of heading used during takeoff, to AUTOLAND_WP_ALT above home,  and at AUTOLAND_WP_DIST distancee, and a LAND waypoint at HOME and stops until next disarm/boot. Warnings are given if the AUTOLAND parameters are not set.

In use you just arm and switch to AUTO, and then switch to other flight modes after takeoff is completed to fly around.....relatively assured that a failsafe (assuming defaults for Failsafe) will result in an autolanding in the correct direction. You can also just switch back to AUTO or RTL to do an autoland. 

Caveats: be sure you have tested and setup autoland and AUTOLAND parameters. Setting AUTOLAND_WP_ALT and _WP_DIST for a good glide path on a final approach is required (be aware of possible obstructions when using). Values of 400 meters distance and 55 meters altitude work well for typcial 1m wingspan/1 kg foam planes. RTL_AUTOLAND must be set and a value = 2 is recommended also.

Be aware of obstacles that might come into play on a final approach. Be aware that occasionally final approach waypoints may not be exactly in line with the takeoff if the GPS signal quality is compromised during takeoff.

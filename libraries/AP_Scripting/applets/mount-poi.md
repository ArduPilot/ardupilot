# Mount POI

This script displays the location (lat, lon and altitude) that the gimbal is currently pointing towards

# Parameters

POI_DIST_MAX : POI's max distance (in meters) from the vehicle

# How To Use

1. Set RCx_OPTION to 300 (scripting1) to allow triggering the POI calculation from an auxiliary switch
2. Optionally set POI_DIST_MAX to the maximum distance (in meters) that the POI point could be from the vehicle
3. Fly the vehicle and point the camera gimbal at a point on the ground
4. Raise the RC auxiliary switch and check the GCS's messages tab for the latitude, longitude and alt (above sea-level)

# How It Works

The script's algorithm is implemented as follows

1. Get the POI_DIST_MAX and TERRAIN_SPACING parameter values
2. Get the vehicle Location (lat, lon, height above sea-level), initialise test-loc and prev-test-loc
3. Get the vehicle's current alt-above-terrain
4. Get gimbal attitude (only pitch and yaw are used)
5. The test_loc variable is initialised to the vehicle's current location
6. The prev_test_loc variable is a backup of test_loc
7. test_loc is moved along the line defined by the gimbal's pitch and yaw by TERRAIN_SPACING (meters)
8. Get the terrain's altitude (above sea-level) at test_loc
9. Steps 6, 7 and 8 are repeated until test_loc's altitude falls below the terrain altitude
10. Interpolate between test_loc and prev_test_loc to find the lat, lon, alt (above sea-level) where alt-above-terrain is zero
11. Display the POI to the user

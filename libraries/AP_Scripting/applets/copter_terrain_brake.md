# Copter Loiter Brake

This script implements an emergency change to BRAKE mode in copter if
you are in LOITER mode and break a terrain altitude limit. The script
is useful when flying in LOITER mode in steep terrain.

# Parameters

The script adds the following parameters to control it's behaviour.

## TERR_BRK_ENABLE

This must be set to 1 to enable the script.

## TERR_BRK_ALT

This is the terrain altitude threshold for engaging BRAKE mode. The
onboard terrain system must be enabled with TERRAIN_ENABLE=1 and
terrain must have either been preloaded to the vehicle (see
https://terrain.ardupilot.org ) or be available from the ground
station over MAVLink.

Make sure you set sufficient margin to cope with obstacles such as
trees or any local towers or other obstacles.

## TERR_BRK_HDIST

This is the distance from home for the BRAKE checking to be
enabled. The default of 100 meters is good for most operations. This
threshold allows you to take over in LOITER mode for low altitude
operations and takeoff/landing when close to home.

## TERR_BRK_SPD

This is a speed threshold BRAKE checking to be enabled. If both the
horizontal speed and the descent rate are below this threshold then
BRAKE will not be engaged. This defaults to zero which means no speed
checking is performed.

You should set this to a small value if you want to be able to recover
from BRAKE mode by climbing straight up in LOITER mode. A value of 0.5
m/s is recommended. The value needed will be dependent on the amount
of noise there is in your velocity measurement and how gusty the wind
is, but 0.5 should work in most applications.

If you set this value then to recover in LOITER mode you should raise
the throttle stick to demand climb before you switch back to LOITER
mode. The positive climb rate means BRAKE will not re-engage.

# Operation

Install the lua script in the APM/SCRIPTS directory on the flight
controllers microSD card. Review the above parameter descriptions and
decide on the right parameter values for your vehicle and operations.

Make sure TERRAIN_ENABLE is 1 and you should preload terrain data for
the flight area from https://terrain.ardupilot.org

It is strongly recommended that you set TERRAIN_SPACING=30 and preload
the SRTM1 terrain data for 30m horizontal resolution of terrain data.

When the system engages you will see a message like this
 "Terrain 29.2m - BRAKE"
where in this example you are 29.2m above the terrain.

To recover you could use GUIDED mode, or RTL (make sure you have set
RTL_ALT_TYPE to terrain) or if you have set TERR_BRK_SPD to a positive
value then you could raise the throttle stick and switch back to
LOITER mode.

If the system is continually giving false positives then set
TERR_BRK_ENABLE to zero to disable.

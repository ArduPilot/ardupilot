#!/bin/sh
# FlightGear view for the heliquad variable-pitch quad helicopter
# (Copter FRAME_CLASS 13, heli-quad SITL frame).
# Start SITL with the flightgear view output enabled, then run this.

AUTOTESTDIR=$(dirname $0)

# FlightGear evaluates --timeofday=noon at the starting position, and the
# external FDM then teleports the model to wherever SITL's home is -- so
# the start position must match SITL home or "noon" can land at night.
# Default is ArduPilot's default home (CMAC); override for other homes,
# e.g. FG_START_POS="--airport=KSFO".  Note the packaged FlightGear base
# data only ships San Francisco area scenery; elsewhere is ocean unless
# scenery has been fetched (e.g. one run with terrasync enabled).
FG_START_POS=${FG_START_POS:-"--lat=-35.363261 --lon=149.165230"}

# --in-air skips FGATCManager registering the (externally driven) user
# aircraft with ground/tower ATC, which otherwise spams "AI error:
# requesting ATC instruction for aircraft without traffic record" once
# SITL moves the aircraft away from where FlightGear placed it.
nice fgfs \
    --native-fdm=socket,in,10,,5503,udp \
    --fdm=external \
    --aircraft=heliquad \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    $FG_START_POS \
    --in-air \
    --geometry=650x550 \
    --bpp=32 \
    --disable-hud-3d \
    --disable-horizon-effect \
    --timeofday=noon \
    --disable-sound \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --disable-terrasync \
    --prop:/sim/traffic-manager/enabled=false \
    --prop:/sim/ai/scenarios-enabled=false \
    --fog-disable \
    --disable-specular-highlight \
    --disable-anti-alias-hud \
    --wind=0@0 \
    $*

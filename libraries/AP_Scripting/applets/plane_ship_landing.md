# Plane Ship Landing

This script adds ship landing to quadplanes.

## Equipment Needed

To use ship landing you need a beacon setup on the landing
platform. This beacon should be a flight controller running the
ArduPilot Rover firmware. You can set the FRAME_CLASS on the beacon to
2 to make it a "boat" so the right icon shows in the ground station.

The beacon flight controller needs the following:
 - a reliable yaw source. Using dual-F9P GPS with moving baseline yaw is recommended if the moving platform will cause problems with compasses. See this page for how to set that up: https://ardupilot.org/rover/docs/common-gps-for-yaw.html. 
 - a radio setup so that the aircraft can see GLOBAL_POSITION_INT mavlink messages from the beacon flight controller. There are multiple methods of achieving that, see the section below on radio setup
 - the beacon needs a different mavlink system ID to the aircraft and the GCS. You set this with the SYSID_THISMAV parameter. In the example below I have set SYSID_THISMAV=17
 - the beacon can be offset from the actual landing location. The beacon should be placed for optimal radio performance, then the instructions below can be used to setup the actual landing location relative to the beacon.

## Radio Setup

To use ship landing you need a radio link between the aircraft and the beacon, and also a radio link between the ground control station (GCS) and both the aircraft and the beacon.

The radio communication needs to be setup so that:
 - the ground station can see mavlink packets from both the beacon and the aircraft
 - the aircraft can see mavlink packets from the beacon (the only packet the aircraft needs from the beacon is the GLOBAL_POSITION_INT message)
There are several ways to setup this type of radio link:
 - you can use WiFi, with a multicast network
 - you can use a mesh-capable radio
 - you can setup your GCS to forward GLOBAL_POSITION_INT packets from the beacon to the aircraft (for example, mavproxy can do this via the fwdpos setting)

For mesh capable radios, you could use the RFDesign multi-point firmware on three RFD900x radios, with the radios setup for broadcast.
Alternatively you could use the RFD900x relay firmware from here https://firmware.ardupilot.org/SiK/RFD900x-relay/ with the GCS set as NODE=1, the beacon set as NODE=0 and the aircraft set as NODE=2.

## Parameters

The following key parameters need to be set on the aircraft:
 - SCR_ENABLE=1
 - SCR_HEAP_SIZE=100000
 - SHIP_ENABLE=1
 - FOLL_ENABLE=1
 - FOLL_SYSID=17  # this needs to be the SYSID of the beacon set above
 - Q_RTL_MODE=0
 - RTL_AUTOLAND=0

Note that you will need to refresh parameters and reboot for these to all take effect. The SHIP_ENABLE, SHIP_LAND_ANGLE and SHIP_AUTO_OFS parameters will appear when the script is running correctly.

## Beacon Setup

Apart from the yaw source, the beacon should also be set to GPS altitude. 
 - EK3_SRC1_POSZ=3

## Land Angle

You can choose the approach angle of the aircraft to the ship. The
default is SHIP_LAND_ANGLE=0 which means land from behind the ship. A
value of 90 will mean that the aircraft approaches the ship from the
left hand side. A value of -90 means it approaches from the right hand
size. A value of 180 means the aircraft will approach the landing from
the front of the ship.

You should choose a SHIP_LAND_ANGLE value to avoid obstructions on the
ship, for example masts. A good value would mean that if you need to
abort the landing that flying straight ahead will leave plenty of
clearance to obstacles.

## Testing Beacon

When the aircraft can see the beacon position it will print a message like this:
  "Have beacon"
if you lose the connection to the beacon (after 3 seconds) you will see a message 
 "Lost Beacon"
You will also get an arming failure if you try to arm without the beacon working.

## Landing Offset

It is important to set the correct values in FOLL_OFS_X, FOLL_OFS_Y and FOLL_OFS_X on the aircraft for the landing point relative to the beacon. These values are in meters, in front-right-down format.
The easiest way to set these is to place the aircraft in the correct landing location with the beacon working and then set the parameter SHIP_AUTO_OFS to 1. When this parameter is set to 1 then the ship landing lua script will calculate the right offset values and set them in the FOLL_OFS_X, FOLL_OFS_Y and FOLL_OFS_Z values.
The values are:
 - FOLL_OFS_X distance in front of the beacon to land (use a negative value for landing behind the beacon)
 - FOLL_OFS_Y distance to the right of the beacon to land (use a negative value for left)
 - FOLL_OFS_Z distance below the beacon to land (use a negative value to land above the beacon)

When the beacon is active you should see the HOME icon on the GCS move to match the landing position. The HOME position is continuously updated while you are flying which gives you a good way to ensure that the beacon is working properly before you land.

I recommend you use the SHIP_AUTO_OFS=1 method of getting the location before each flight. Look carefully at the message it gives when you set this parameter (use the messages tab in MissionPlanner):
  - Set follow offset (-10.82,3.29,0.46)

that message tells you the X, Y and Z offset it has calculated. Check that they are reasonable, paying close attention to the Z offset. If you get a bad Z offset (ie. a long way off from the actual height difference between the beacon and the aircraft) then you may need to reboot the beacon and/or aircraft to cope with GPS altitude drift.

## Takeoff Procedure

When SHIP_ENABLE=1 and the beacon is visible to the aircraft then an AUTO VTOL takeoff will use velocity matching, so the aircraft will hold its velocity relative to the beacon while ascending. This velocity matching is only done for an AUTO VTOL takeoff.

## Holdoff Position

A key part of ship landing is the "holdoff position". The holdoff position is where the aircraft will loiter while waiting for the pilot to command the landing via the movement of the throttle stick (see throttle stick information below).
The holdoff position is based on a few criteria:
 - the RTL_RADIUS in meters (negative for counter-clockwise loiter, positive for clockwise loiter)
 - if RTL_RADIUS is zero then the WP_LOITER_RAD parameter is used instead
 - the SHIP_LAND_ANGLE parameter, which controls the angle that the aircraft will approach the ship for landing. A value of zero means to approach from behind the ship. The holdoff loiter position will be setup so that the tangent of the circle intercepts the beacon landing point.
 - the Q_TRANS_DECEL parameter, which determines how fast the aircraft can slow down
 - the speed of the ship, the wind speed and the speed of the aircraft

## Landing Procedure

When you are ready to land you can switch the vehicle to RTL mode. When in RTL mode the aircraft will fly towards the landing location (you can see this location before you land from the HOME icon on the GCS, which moves with the beacon).
The aircraft will initially approach the "holdoff" position. The altitude of the holdoff position is set by the ALT_HOLD_RTL parameter (in centimeters above the landing location). A good value of this is around 9000, which is 90 meters above the landing location.
For the description below I will assume that SHIP_LAND_ANGLE=0 which means landing happens from behind the beacon. The approach and landing is rotated by the value of this parameter in degrees.
With SHIP_LAND_ANGLE=0 the holdoff position will be behind and above the beacon. The distance depends on the beacon speed, wind speed and the Q_TRANS_DECEL parameter (which controls the deceleration of the aircraft). 
Once the aircraft arrives at the holdoff position it will circle until the throttle stick is lowered below 40%. The throttle stick on the transmitter is used to control the landing sequence and also to abort the landing.
Throttle stick controls are:
 - throttle at 40% or above means to hold at the holdoff position (at ALT_HOLD_RTL height above beacon in centimeters)
 - throttle below 40% and above 10% means to descend to the approach altitude. The approach altitude is giving by Q_RTL_ALT in meters above the beacon. A good value for testing would be 40 meters.
 - throttle below 10% means to start landing approach once the aircraft is at the correct height and lined up with the landing location

Once the landing descent has started you may wish to abort the landing. To do this you need to have the Q_OPTIONS bit set for ThrLandControl (bit 15). When that option is set you can raise the throttle momentarily above 70% to enable throttle control for climb and descent rate. You can use this to slow the descent or climb back up. If you climb up past the Q_RTL_ALT approach altitude then the aircraft will go back to RTL circling at the holdoff location.

You may also want to enable horizontal repositioning with the Q_OPTIONS bit EnableLandResponsition (bit 17). If that is enabled then you can reposition the aircraft horizontally while landing to account for GPS position errors.

## Simple Mission

The simplest mission would be a single VTOL_TAKEOFF waypoint. Once the takeoff is complete the aircraft will immediately switch to RTL mode and go to the holdoff location. You should have the throttle stick above 40% to keep the aircraft circling at the holdoff location. This very simple mission is good for ship operations as it does not have specific latitude/longitude, so will work wherever the ship is.

## Simulation

To simulation ship landing you should set:
 - SIM_SHIP_ENABLE=1
 - SIM_SHIP_SPEED=5 # speed of ship in m/s
 - SIM_SHIP_DSIZE=50 # size of the simulated deck of the ship
 - SIM_SHIP_PSIZE=2000 # radius of the circular path the ship follows in meters
 - SIM_SHIP_OFS_X=5 # distance of the beacon in front of the aircraft at startup
 - SIM_SHIP_OFS_Y=0 # distance of the beacon to the right of the aircraft at startup

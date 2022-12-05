# Scripted Aerobatics

The lua script "plane_aerobatics.lua" implements scripted aerobatics,
allowing fixed wing aircraft to execute a number of aerobatic
manoeuvres either in AUTO mission or by triggering using pilot commands
using RC switches.

As always, but particularly with scripted aerobatics, test in SITL until 
you understand the function and behaviour of each manouver. You will need 
an appropriate aircraft, and be ready to take manual control if necessary!

## Available Manoeuvres

The following table gives the available manoeuvres. Each manoeuvre has
an ID number which is used in the AUTO mission or in the TRIKn_ID
parameters (described below). The present ground track is used as the track for the trick.
The "Turnaround" column indicates if the manoeuvre results in a course reversal, which impacts how it is used in 
AUTO missions. Once the trick is completed, the mode that was being used at the start of the trick is restored. If the mode is CRUISE, its
track and altitude are reset to the values present when the mode is restored. Tricks in AUTO missions require that they be performed between two waypoints to establish
the ground track.

| ID | Name                     | Arg1   | Arg2        | Arg3        | Arg4       | Turnaround |
| -- | ------------------------ | ------ | ----------  | -------     | ---------- | ---------- |
| 1  | Figure Eight             | radius | bank angle  |             |            | No         |
| 2  | Loop                     | radius | bank angle  | num loops   |            | No         |
| 3  | Horizontal Rectangle     | length | width       | radius      | bank angle | No         |
| 4  | Climbing Circle          | radius | height      | bank angle  |            | No         |
| 5  | vertical Box             | length | height      | radius      |            | No         |
| 6  | Immelmann (FastRoll)     | radius |             |             |            | Yes        |
| 7  | Straight Roll            | length | num rolls   |             |            | No         |
| 8  | Rolling Circle           | radius | num rolls   |             |            | No         |
| 9  | Half Cuban Eight         | radius |             |             |            | Yes        |
| 10 | Half Reverse Cuban Eight | radius |             |             |            | Yes        |
| 11 | Cuban Eight              | radius |             |             |            | No         |
| 12 | Humpty Bump              | radius | height      |             |            | Yes        |
| 13 | Straight Flight          | length | bank angle  |             |            | No         |
| 14 | Scale Figure Eight       | radius | bank angle  |             |            | No         |
| 15 | Immelmann Turn           | radius |             |             |            | Yes        |
| 16 | Split-S                  | radius |             |             |            | Yes        |
| 17 | Upline-45                | radius | height gain |             |            | No         |
| 18 | Downline-45              | radius | height loss |             |            | No         |
| 19 | Stall Turn(experimental) | radius | height      | direction   |            | Yes        |
| 20 | Procedure Turn           | radius | bank angle  | step-out    |            | Yes        |
| 21 | Derry Turn               | radius | bank angle  |             |            | No         |
| 23 | Half Climbing Circle     | radius | height      | bank angle  |            | Yes        |
| 25 | Laydown Humpty           | radius | height      |             |            | Yes        |
| 25 | Barrel Roll              | radius | length      | num spirals |            | No         |
| 26 | Straight Hold            | length | bank angle  |             |            | No         |
| 31 | Multi Point Roll         | length | num points  | hold frac   | pts to do  | No         |

Some notes about maneuver arguments (arg1 - arg4):
These are parameters each maneuver requires to execute. For example the length of a roll or radius of a loop (in meters), the number of rolls, the height of the maneuver, etc.
When setting up a multi-point roll, for example, the length is in meters, the number of points is 'how many points should the roll have during 360 degrees', the hold fraction is the amount of the maneuver 'not rolling', and the points to do is how many points do you wish to fly. For example, if we want '2 of 4 point roll' : two points of a four point roll. From upright roll through 90 degrees, pause, and then roll through a further 90 degrees - finishing inverted. And we want the maneuver to happen over 100m, and the pause between points to be 50% of the maneuver length. Then the four arguments would be?

Length = 100, num points = 4, hold fraction = 0.5, pts to do = 2.

Remember, the model is now exiting inverted so the next maneuver must be planned to start from this position.

Note: In the script you will find other (specialised) manouvers which do not appear in the 
'command table'. These tend to be specialised manouvers which may expect an inverted entry,a very high entry (270m), or 
finish inverted and will not end well if started upright at a low altitude! These 
manouvers are used in some of the schedules defined below. 


## Available Schedules (pre-defined sequences of manouvers)

See the Schedules subdirectory for a wide variety of pre-defined
full aerobatic schedules you can use and instructions for how to
install them.

## Loading the script

Put the plane_aerobatics.lua script on your microSD card in the
APM/SCRIPTS directory. You can use MAVFtp to do this.

Then set

 - SCR_ENABLE = 1
 - SCR_HEAP_SIZE = 300000
 - SCR_VM_I_COUNT = 200000

You will need to refresh parameters after setting SCR_ENABLE. Then
reboot to start scripting.

## Aircraft Setup

The aircraft needs to be setup to perform well in ACRO mode. You need
to enable the yaw rate controller by setting:

 - YAW_RATE_ENABLE = 1
 - ACRO_YAW_RATE = 90

The ACRO_YAW_RATE depends on the capabilities of the aircraft, but
around 90 degrees/second is reasonable.

You need to tune the yaw rate controller by flying in AUTOTUNE mode
and moving the rudder stick from side to side until the yaw rate
tuning is reported as being finished. For optimal results, log examination and manual adjustment
of axis tuning will sometimes be required, especially pitch and yaw tuning in order to get precise rolling circles. A future 
video on this optimization will be produced and posted in the ArduPilot YouTube channel.

## Use In AUTO Missions

To use in an AUTO mission you can create waypoint missions containing
NAV_SCRIPT_TIME elements (shown as SCRIPT_TIME in MissionPlanner). These mission items take the following arguments:

 - the command ID from the table above
 - the timeout in seconds
 - up to four arguments as shown in the above table

The aerobatics system will use the location of the previous and next
waypoints to line up the manoeuvre. You need to plan a normal
waypoint just before the location where you want to start the
manoeuvre, then the NAV_SCRIPT_TIME with the trick or schedule ID, a timeout that is long enough to allow the trick or schedule, and then a normal waypoint after the manoeuvre.

## Use with "tricks on a switch"

You can trigger the manoeuvres using RC switches, allowing you to have
up to 11 tricks pre-programmed on your transmitter ready for use in
fixed wing flight. You can trigger the tricks in the following flight
modes:

 - CIRCLE
 - STABILIZE
 - ACRO
 - FBWA
 - FBWB
 - CRUISE
 - LOITER

To setup tricks you need to first set the parameter TRIK_ENABLE to 1.

After setting TRIK_ENABLE, either restart scripting or reboot. Then
set TRIK_COUNT to the number of tricks you want to make available,
with a maximum of 11.

After setting TRIK_COUNT, reboot and refresh parameters. You will find
you will now have 5 parameters per trick.

 - TRIKn_ID
 - TRIKn_ARG1
 - TRIKn_ARG2
 - TRIKn_ARG3
 - TRIKn_ARG4

The ID parameter is the manoeuvre from the above table, and the arguments are the arguments to each manoeuvre.

Note: these parameters, if being loaded from a file, will not be present until scripting is enabled, the the LUA script has been
actually run, since they are created via the script, not the firmware.

Now you need to setup your two control channels. You should have one 3
position switch for activating tricks, and one knob or switch to
select tricks. It is best to use a knob for the trick selector so you can have up to 11 tricks.

Work out which RC input channel you want to use for activation (a 3 position switch) and set

 - RCn_OPTION = 300

Then work out what RC input channel you want to use for selection and set

 - RCn_OPTION = 301

## Flying with tricks

When the activation channel (the one marked with option 300) is in the
middle position then when you move the knob the GCS will display the
currently selected trick.

To activate a trick you move the activation channel to the top
position, and the trick will run.

Moving the activation switch to the bottom position cancels any
running trick and stops the trick system.

Changing flight modes will also cancel any active trick.

## Parameters

There are a number of parameters you can set to adjust the path
tracking. Some of the key parameters are:

 - AEROM_ANG_ACCEL : maximum angular acceleration in degrees per second per second. Reduce to give smoother flight, but more lag
 - AEROM_ANG_TC : time constant for correcting angular roll errors. Reduce for snappier rolls, but more risk of oscillation
 - AEROM_KE_ANG : knifeedge angle. This is the required pitch angle in knifeedge flight to hold height at cruise speed.
 - AEROM_ENTRY_RATE : roll rate in degrees per second for entering and exiting a roll change
 - AEROM_THR_MIN : minumum throttle percentage for all aerobatic maneuvers
 - AEROM_THR_BOOST: minumum throttle percentage for maneuvers marked as throttle boost
 - AEROM_YAW_ACCEL: maximum yaw acceleration in degrees per second per second. Lower to soften yaw control
 - AEROM_PATH_SCALE: scale factor for all maneuvers. A value above 1.0 will increase the size of the maneuvers. A value below 1.0 will decrease the size. A negative value will mirror the maneuvers, allowing a sequence designed for left-to-right to be flown right-to-left.

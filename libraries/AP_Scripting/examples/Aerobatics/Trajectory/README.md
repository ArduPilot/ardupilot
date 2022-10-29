# Scripted Aerobatics

The lua script "plane_aerobatics.lua" implements scripted aerobatics,
allowing fixed wing aircraft to execute a number of aerobatic
manoeuvres either in AUTO mission or by triggering using pilot commands
using RC switches.

## Available Manoeuvres

The following table gives the available manoeuvres. Each manoeuvre has
an ID number which is used in the AUTO mission or in the TRIKn_ID
parameters (described below).

| ID | Name                     | Arg1   | Arg2        | Arg3      | Arg4       | Turnaround |
| -- | ------------------------ | ------ | ----------  | -------   | ---------- | ---------- |
| 1  | Figure Eight             | radius | bank angle  |           |            | No         |
| 2  | Loop                     | radius | bank angle  | num loops |            | No         |
| 3  | Horizontal Rectangle     | length | width       | radius    | bank angle | No         |
| 4  | Climbing Circle          | radius | height      |           |            | No         |
| 5  | vertical Box             | length | height      | radius    |            | No         |
| 6  | Banked Circle            | radius | bank angle  | height    |            | No         |
| 7  | Straight Roll            | length | num rolls   |           |            | No         |
| 8  | Rolling Circle           | radius | num rolls   |           |            | No         |
| 9  | Half Cuban Eight         | radius |             |           |            | Yes        |
| 10 | Half Reverse Cuban Eight | radius |             |           |            | Yes        |
| 11 | Cuban Eight              | radius |             |           |            | No         |
| 12 | Humpty Bump              | radius | height      |           |            | Yes        |
| 13 | Straight Flight          | length | bank angle  |           |            | No         |
| 14 | Scale Figure Eight       | radius | bank angle  |           |            | No         |
| 15 | Immelmann Turn           | radius | roll rate   |           |            | Yes        |
| 16 | Split-S                  | radius | roll rate   |           |            | Yes        |
| 17 | Upline-45                | radius | height gain |           |            | No         |
| 18 | Downline-45              | radius | height loss |           |            | No         |
| 19 | Stall Turn               | radius | height      | direction |            | Yes        |

The "Turnaround" column indicates if the manoeuvre results in a course
reversal, which impacts how it is used in AUTO missions.

## Loading the script

Put the plane_aerobatics.lua script on your microSD card in the
APM/SCRIPTS directory. You can use MAVFtp to do this.

Then set

 - SCR_ENABLE = 1
 - SCR_HEAP_SIZE = 200000
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
tuning is reported as being finished.

## Use In AUTO Missions

To use in an AUTO mission you can create waypoint missions containing
NAV_SCRIPT_TIME elements (shown as SCRIPT_TIME in MissionPlanner). These mission items take the following arguments:

 - the command ID from the table above
 - the timeout in seconds
 - up to four arguments as shown in the above table

The aerobatics system will use the location of the previous and next
waypoints to line up the manoeuvre. You need to plane a normal
waypoint just before the location where you want to start the
manoeuvre, then the NAV_SCRIPT_TIME and then a normal waypoint after
the manoeuvre.

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

Now you need to setup your two control channels. You should have one 3
position switch for activating tricks, and one knob or switch to
select tricks. It is best to use a knob for the trick selector so you can have up to 11 tricks.

Work out which RC input channel you want to use for activation (a 3 position switch) and set

 - RCn_OPTION = 300

Then work out what RC input channel you want to use for selection and set

 - RCn_OPTION = 301

## Flying with tricks

When the activation channel (the one marked with option 300) is in the
middle position then when you move the knob it will display the
currently selected trick.

To activate a trick you move the activation channel to the top
position, and the trick will run.

Moving the activation switch to the bottom position cancels any
running trick and stops the trick system.

Changing flight modes will also cancel any active trick.

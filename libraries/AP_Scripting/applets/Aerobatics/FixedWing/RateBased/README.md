# Sport Aerobatics

The lua script "sport_aerobatics.lua" implements scripted aerobatics,
allowing fixed wing aircraft to execute a number of aerobatic
manoeuvres either in AUTO mission or by triggering using pilot commands
using RC switches.

As always, but particularly with scripted aerobatics, test in SITL until 
you understand the function and behaviour of each manouver. You will need 
an appropriate aircraft, and be ready to take manual control if necessary!

These tricks are rate-based vs the precision trajectory-based manouvers of the script in the TrajectoryBased directory above this one. However, many planes have difficulty completing those tricks unless they can sustain extended vertical climbs and knife-edges, especially in wind. These tricks do not try to maintain a geospatial track, but rather, just attitude rates. Even planes that cannot hold the 90 deg knife edge trick, will probably hold the 180 deg (inverted) one and do loops and rolls, and these tricks can be done in even strong wind, although will not be well shaped.

## Available Manoeuvres

The following table gives the available manoeuvres. Each manoeuvre has
an ID number which is used in the AUTO mission or in the TRIKn_ID
parameters (described below). The present ground track is used as the track for the trick.
The "Turnaround" column indicates if the manoeuvre results in a course reversal, which impacts how it is used in 
AUTO missions. Once the trick is completed, the mode that was being used at the start of the trick is restored. If the mode is CRUISE, its
track and altitude are reset to the values present when the mode is restored. Tricks in AUTO missions require that they be performed between two waypoints to establish
the ground track.

| ID | Name                     | Arg1            | Arg2                       | Turnaround |
| -- | ------------------------ | --------------- | -------------------------- | -----------|
| 1  | Roll(s)                  | rollrate(dps)   | num rolls                  | No         |
| 2  | Loop(s)/TurnAround       | pitchrate(dps)  | num loops or turnaround(0) | if num=0   |
| 3  | Rolling Circle           | yawrate(dps)    | rollrate(dps)              | No         |
| 4  | KnifeEdge                | roll angle(deg) | length(sec)                | No         |
| 5  | Pause                    | length(sec)     | na                         | No         |
| 6  | KnifeEdge Circle         | yawrate(dps)    | na                         | No         |
| 7  | 4 point roll             | rollrate(dps)   | pause in sec at each point | No         |
| 8  | Split-S                  | pitchrate(dps)  | rollrate(dps)              | Yes        |

note: for Rolling Circle, the time it takes to make the circle is 360/yawrate. You should make sure that an integer number of rolls is commanded by the rollrate parameter in that time, ie rollrate should be set to the number of rolls * yawrate. In most cases negative rate, reverses the direction, ie in Rolls -45 for rollrate would roll left at 45dps.

## Loading the script

Put the sport_aerobatics.lua script on your microSD card in the
APM/SCRIPTS directory. You can use MAVFtp to do this.

Then set

 - SCR_ENABLE = 1
 - SCR_HEAP_SIZE = 150000
 - SCR_VM_I_COUNT = 100000

You will need to refresh parameters after setting SCR_ENABLE. Then
reboot to start scripting.

## Aircraft Setup

The aircraft needs to be setup to perform well in ACRO mode. You need
to enable the yaw rate controller by setting if it has a rudder (even if it cannot hold a sustained 90 degree knife-edge:

 - YAW_RATE_ENABLE = 1
 - ACRO_YAW_RATE = 90

The ACRO_YAW_RATE depends on the capabilities of the aircraft, but
around 90 degrees/second is reasonable.

You need to tune the yaw rate controller by flying in AUTOTUNE mode
and moving the rudder stick from side to side until the yaw rate
tuning is reported as being finished. For optimal results, log examination and manual adjustment
of axis tuning will sometimes be required, especially pitch and yaw tuning in order to get precise rolling circles. A future video on this optimization will be produced and posted in the ArduPilot YouTube channel.

## Use In AUTO Missions

To use in an AUTO mission you can create waypoint missions containing
NAV_SCRIPT_TIME elements (shown as SCRIPT_TIME in MissionPlanner). These mission items take the following arguments:

 - the command ID from the table above
 - the timeout in seconds
 - up to two arguments as shown in the above table

The aerobatics system will use the location of the previous and next
waypoints to line up the manoeuvre. You need to plan a normal
waypoint just before the location where you want to start the
manoeuvre, then the NAV_SCRIPT_TIME with the trick or schedule ID, a timeout that is long enough to allow the trick, and then a normal waypoint after the manoeuvre. You can have consecutive tricks between waypoints. You can use the PAUSE trick after a framing waypoint to  move the location of the beginning of the trick or to put some time between consecutive tricks.

## Use with "tricks on a switch"

You can trigger the manoeuvres using RC switches (or using the GCS AUX Function tab), allowing you to have
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

Set TRIKR_COUNT to the number of tricks you want to make available and reboot,
with a maximum of 11. (Why 11 when fewer than that are available? To allow variants, such as different knife-edges, ie 90 deg and 180 deg inverted flight)

After setting TRIKR_COUNT, reboot and refresh parameters. You will find
you will now have 5 parameters per trick,but only three are used at this time.

 - TRIKRn_ID
 - TRIKRn_ARG1
 - TRIKRn_ARG2
 - TRIKRn_ARG3 (unused, future use)
 - TRIKRn_ARG4 (unused, future use)

The ID parameter is the manoeuvre from the above table, and the arguments are the arguments to each manoeuvre.

Note: these parameters, if being loaded from a file, will not be present until scripting is enabled, the the LUA script has been
actually run, since they are created via the script, not the firmware.

Now you need to setup your two control channels. You should have one 3
position switch for activating tricks, and one knob or switch to
select tricks. It is best to use a knob for the trick selector so you can have up to 11 tricks.

Work out which RC input channel you want to use for activation (a 3 position switch) and set

 - RCn_OPTION = 300
 
Note: It is not required to setup the 300 function for activation by the Transmitter switch. You can also activate a trick via the GCS. Mission Planner has an AUX Functions tab to enable setting the "300" function from the GCS.

Then work out what RC input channel you want to use for selection and set

 - RCn_OPTION = 301
 
 TRIKR_COUNT must be non-zero and an RC channel set to function 301 for Tricks on a Switch to function.

## Flying with tricks

When the activation channel (the one assigned option 300 or set via the GCS) is in the
middle position then when you move the trick selection knob/switch, the GCS will display the
currently selected trick.

To activate a trick you move the activation channel to the top (high)
position, and the trick will run.

Moving the activation switch to the bottom position (low) cancels any
running trick and stops the trick system.

Changing flight modes will also cancel any active trick.

## Parameters

There are a number of parameters added by this script to control its control loops. The defaults should be satisfactory, but some of the key parameters are:


 - AEROR_HGT_KE_BIAS: knife-edge boost. Adds immediate rudder as the plane rolls into 90 degree positions rather than waiting of an altitude change
 - AEROR_THR_FF : modulates throttle as pitch increases or decreases

the other parameters control the height and speed PID controllers used in the script

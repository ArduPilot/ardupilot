# Scripted Aerobatics Schedules

This directory contains full aerobatic schedules for the ArduPilot
fixed wing scripted aerobatics system. If you develop your own
schedules please contribute to the project by submitting them for
inclusion in this directory.

## Usage

To use one of these schedules in SITL copy it (or use a symbolic link)
to a file called trickNN.txt in your scripts/ directory (the same
directory that plane_aerobatics.lua goes in). The NN is the trick
number used in the AUTO mission or with tricks on a switch in the
TRIKn_ID parameter.

For example, if you wanted to fly the NewZealand Clubman schedule you
could copy NZClubMan.txt to trick94.txt in your scripts
directory. Then you could set TRIK3_ID to 94 to make this schedule be
trick 3. Or you could put 94 in the NAV_SCRIPT_TIME auto-mission
command.

## Scaling

Some of these schedules are quite large and you may need to shrink
them for your model or flying field. You can use the AEROM_PATH_SCALE
parameter to adjust the size. For example, setting AEROM_PATH_SCALE to
0.5 will halve the size of the schedule. This impacts all tricks and
schedules.

You can also mirror the schedule by setting AEROM_PATH_SCALE to a
negative value. This is good if the schedule is designed to be flown
right-to-left and you want to fly it left-to-right due to the wind
direction. If you set AEROM_PATH_SCALE to -0.5 then it would do a
half-scale mirrored schedule.


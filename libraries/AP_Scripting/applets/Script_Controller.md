# Script Control LUA Script

This script allows the user to have groups of scripts in three subdirectories of the main scripting directory and copy them into the main scripts directory, removing any files in that directory with the .lua extension (except itself) and issue a scripting restart based on the position of a switch with a programmable auxiliary function option (default is "302", but can be changed by user during setup) or using Mission Planner's AUX Function tab. This provides means to have differing scripts depending on the mission or conditions, and use whichever set is desired without having them consume memory all the time, and without requiring a GCS to manage the content of the scripts directory. It also allows a new mission to be loaded associated with those new scripts, if desired.

If the designated RC switch auxiliary option is not assigned to a RC channel via an RCx_OPTION, or the selected subdirectory does not exist, no action is taken, so the script can reside in the scripts directory even if not being used.

# Setup and Operation

The auxiliary function option (300 - 307) that can be assigned to an RC channel and/or used by Mission Planner is set by the scripting parameter SCR_USER6. If not set (this param's default is "0") then function "302" is assigned automatically. Setting an RC channel's option to this value allows RC control of the scripting subdirectory that will be used for scripts (and optionally to load a mission file from within that subdirectory). Mission Planner can control switching also, or in place of the RC channel, using its AUX Function tab.

The auxiliary function (default "302") is used to determine which of the three scripts subdirectories will be used: LOW:scripts/1, MIDDLE:scripts/2, or HIGH:scripts/3. If an RC channel option has been set to this,but no RC has been established during ground start, it will behave as if subdirectory 1 is selected. Changing this switch position either prior to ground start or after (or using Mission Planner), if the selected subdirectory exists, will remove ALL the *.lua files in the scripts directory (except itself), and copy any *.lua files (only) from the desginated subdirectory, load a new mission if a mission file named "mission.txt" exists in the subdirectory, and then issue a scripting restart. If the subdirectory does not exist, then no action is taken and a GCS message issued.

# CAUTIONS

If parameters are being created by scripts be sure that different scripts do not use the same PARAMETER KEY to create their param tables, unless you desire to use the same parameter set in two different scripts. Otherwise, param values will be corrupt when you change scripts.
Be sure to have unique parameter names for different parameter sets. This script does not create any parameters for potential conflict.

Parameter names are only visible and used by Ground Control Stations. While indentically named params in different scripts will operate fine, you will not be able to distinguish them in the GCS displays!

Any .lua script in the scripts directory will be ERASED! when this script is run. In order to keep another .lua script running when using this script, place it in every subdirectory so that it will be copied into the main scripts directory whenever changing directories.



# Param_Controller LUA script

This script allows the user to have different parameters (in files named "params.parm") in three subdirectories of the main scripting directory and load them based on the position of a switch with the auxiliary function of "302". This allows easy, at the field selection of parameter sets. It uses the same switch function and directories as the Scripting_Controller.lua script selector and the two may be used together, if desired.

# Setup and Operation

The user must setup an RCx_OPTION to function 302 to determine which of the three scripts subdirectories will be used: LOW:scripts/1, MIDDLE:scripts/2, or HIGH:scripts/3. If no RC has been established during ground start, it will behave as if subdirectory 1 is selected. Changing this switch position either prior to ground start or after, will load a "params.param" file from the desginated subdirectory (if it and its subdirectory exists). Mission Planner also provides a means of executing the same function as an RC switch assigned to 302 in its AUX Function tab, so a transmitter switch does not necessarily need to be used.

Note that loaded parameter changes are not saved across reboots, that must be done manually.

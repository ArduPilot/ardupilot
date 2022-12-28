# Mission_Controller LUA script

This script allows the user to have different missions (in files named "mission.txt") in three subdirectories of the main scripting directory and load them based on the position of a switch with the auxiliary function of "302" when the script is started at boot or by switch position change. This allows easy, at the field selection of missions, particulary useful for changing autoland/RTL_AUTOLAND directions based on prevailing wind conditions. 

# Setup and Operation

The user must setup an RCx_OPTION to function 302 to determine which of the three scripts subdirectories will be used: LOW:scripts/1, MIDDLE:scripts/2, or HIGH:scripts/3. If no RC has been established during ground start, it will behave as if subdirectory 2 is selected. Changing this switch position will load a "mission.txt" file from the designated subdirectory (if it exists).

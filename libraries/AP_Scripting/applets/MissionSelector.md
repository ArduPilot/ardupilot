# MissionSelector LUA script

This script will select and load one of three mission files upon every arm depending on the state of the AUX FUNCTION switch for Mission Reset (24). This allows easy, at the field selection of missions, particulary useful for changing autoland/RTL_AUTOLAND directions based on prevailing wing conditions. MissionH.txt, MissionM.txt, or MissionL.txt mission files in the SD card root will be loaded based on that switch position on a disarmed to armed transition.

The basic AUX function of resetting the mission pointer to the first waypoint is unaltered when the switch is moved to the high position, as previous.

If the AUX FUNCTION rc switch is not configured, the script is not active.

If the file selected by the switch position is not in the root SD directory, nothing happens, otherwise the current mission will be cleared and the designated mission file will be loaded and the user notified of the mission change.

If the file is available but is not the correct format , the script will abort.

So a user can install the script and if the switch is configured and a file exists for selection it will function, but either can be missing without causing messaging to the user.
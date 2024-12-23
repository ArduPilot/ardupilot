# MissionRotation LUA script for Ardupilot Plane

This LUA script is an evolution on the MissionSelector.LUA.

Allows you to select up to 10 missions, either while arming or in the unarmed state.

Requires that an RCx_OPTION be set to 302 or another value that contemplates scripting, in which case it will also have to be changed within the LUA.

The scripts should be installed in the SCRIPTS folder on the MicroSD, and the mission files in the root.

The mission file called mission0.txt (the default mission) must exist and is loaded at boot, if the script does not find it it stops working.

To load to the next mission file just bring AUX switch from low to high for no more than three seconds, each switch high-low loads the next mission file in numerical order.

If the next mission file numerically does not exist load the next one, at the end of the rotation it returns to load mission0.txt.

If AUX is held at high state for more than three seconds the script will restart by loading mission0.txt.

If a mission switch is attempted in AUTO flight mode, a warning "Cannot switch missions in AUTO mode" will be shown and no operation will be performed.

If the script is loaded while the flight mode is AUTO you will get the error message "The script cannot be loaded in AUTO mode".

Provides for sending messages under any condition, load or error.

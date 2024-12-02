# MissionRotation LUA script

This LUA script is an evolution on the MissionSelector.LUA.

Allows you to select up to 10 missions, either while arming or in the unarmed state.

Requires that an RCx_OPTION be set to 24 (Mission Reset).

The scripts should be installed in the SCRIPTS folder on the MicroSD, and the mission files in the root.

The mission file called mission0.txt (the default mission) must exist and is loaded at boot, if the script does not find it it stops working.

To load to the next mission file just bring AUX switch from low to high for no more than three seconds, each switch high-low loads the next mission file in numerical order.

If the next mission file numerically does not exist load the next one, at the end of the rotation it returns to load mission0.txt.

If AUX is held at high state for more than three seconds the script will restart by loading mission0.txt.

Provides for sending messages under any condition, load or error.

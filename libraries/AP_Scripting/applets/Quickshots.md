# QuickShots
Script creates a Dronie, Rocket or Helix mission.
This is triggered by QGroundControl custom actions.

# INSTRUCTIONS

-setup scripting:
```
SCR_ENABLE 1
```
- Use MP config tab -> MAVFtp to to copy the following scripts to the sd card 
- 'libraries/AP_Scripting/applets/Quickshots.lua APM/scripts 
- 'libraries/AP_Scripting/modules/MAVLink/mavlink_msgs.lua APM/scripts/modules/MAVLink 
- 'libraries/AP_Scripting/modules/MAVLink/mavlink_msg_COMMAND_ACK.lua APM/scripts/modules/MAVLink 
- 'libraries/AP_Scripting/modules/MAVLink/mavlink_msg_COMMAND_LONG.lua APM/scripts/modules/MAVLink 
- 'libraries/AP_Scripting/modules/MAVLink/mavlink_msg_GLOBAL_POSITION_INT.lua APM/scripts/modules/MAVLink 

- Reboot

-setup QGroundControl:
```
Install QGroundControl 5.0 or higher
```

Place the file FlyViewMavlinkActions.json into the MavlinkActions directory 
Windows: ~/Documents/QGroundControl/MavlinkActions 
Android: /QGroundControl/MavlinkActions 

# USAGE

Connect QGroundControl and wait for gps fix. 
Create a mission via custom action. 
Start the mission.

-ROI
If QGroundControl streams its position it is used as ROI. 
Set "Stream GCS Position" to "always" in Application Settings for this to work. 
If QGroundControl does not stream its position it is assumed the ROI (you) is 8m 
in front of the drone, so place the drone 8m away facing you.

# HOW IT WORKS

The script waits for MAVLink message MAV_CMD_USER_1 (31010). 
This message is sent from QGroundControl via custom action. 
Depending on the parameters a Dronie, Rocket or Helix mission is created. 
The GCS position is read if the GCS sends GLOBAL_POSITION_INT messages.

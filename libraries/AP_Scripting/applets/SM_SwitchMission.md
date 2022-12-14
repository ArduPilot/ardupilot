# SM_SwitchMission.lua - ArduPilot Lua script

## GENERAL:

That script is derived from MissionSelector.lua in  
https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets  
and the examples in https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/examples  
  
It is useful to switch between missions that are saved on SD-card.

The amount of missions is theoretically unlimited.

The switching can be done by switch or pushbutton.

Switching is allowed when: 
  * disarmed, 
  * in modes that are not mode-AUTO if they are activated by RC or GCS
  * in mode-AUTO when the current mission is complete

This script is intended to use with yaapu-script in transmitter, because all info and warning messages of the script you can receive herein. If you can accept not receiving any feedback, the usecase with a multi-position-switch is possible also without yaapu-script.
https://github.com/yaapu/FrskyTelemetryScript


**CAUTION: Use this script AT YOUR OWN RISK**

## HOW IT WORKS:
### Initialization:

- add script-specific parameter-table _SM at still existing or first free table_key
- read parameter `SM_POSITIONS` (number of switch positions)
- read parameter `SM_RC_OPTION` (selected scripting-option-switch in range 300..307)
- find correct subdirectory for SITL or SD-card
- count available and check sufficient number of missions
- find channel for mission-switching according to parameter `SM_RC_OPTION`


### Running:

* Get RC input as selected and check if loading a mission is requested
* If loading is requested, 
  * check if changing the mission is allowed:
       * the current Mode is non-AUTO and the ModeReason is RC, GCS or INITIALIZED or
       * the current Mode is non-AUTO and the vehicle isn't armed or
       * the Mission is completed
* If allowed, load corresponding Mission to FC:

## HOW TO USE:

1. Store the Missions `SM_Mission#0.waypoints, SM_Mission#1.waypoints` and so on in the subdir `/missions` where the lua-script has to be placed (e.g. on SD-Card in '/APM/scripts/missions'). Make sure that the # of missions are starting at #0 and ascending with no gap.

2. Put the Scripting-Option-Switch you decided to use (300..307) into the `SM_RC_OPTION` parameter

3. Put the selected Scripting-Option-Switch (300..307) into the parameter `RCx_OPTION` of the RC-Channel `x` of your choice.

4. Put information about the selection-method you want to use for the missions into the `SM_POSITIONS` parameter:
    * if <1 : no action (disable switching of missions)
    * if  1 : the selection is done by a pushbutton: short-push will go through the missions, long-push will load the mission
    * if >1 : the selection is done by a multi-position-switch with the amount of positions are given here
    * if using a multi-position-switch, the whole way of the corresponding RC-Channel is devided into regions e.g. for 6 missions: 

        |Mission: |     |#0  |    |#1 |    |#2 |   |#3 |   |#4 |   |#5  |    |
        |---------|:---:|:--:|:--:|---|:--:|---|:-:|---|:-:|---|:-:|----|:--:|
        |Fraction |     |1/10|    |1/5|    |1/5|   |1/5|   |1/5|   |1/10|    |
        |Threshold|-100%|    |-80%|   |-40%|   |0% |   |40%|   |80%|    |100%|

      * For easy calculation of input-steps it's recommended to set  
      RCx_MIN to 1000us, RCx_TRIM to 1500us and RCx_MAX to 2000us<br><br>
      
5. restart the script (e.g. by restarting the FC)

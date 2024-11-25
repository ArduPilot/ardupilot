
 Allows crop sprayers to automatically turn off sprayer on RTL, LAND, Failsafe triggers(RC_FS etc.)

-- Check https://ardupilot.org/copter/docs/sprayer.html to activate speed dependant automatic spraying
-- Purpose of this script is improving spraying automation ability of ardupilot. 
Normally sprayer won't stop on RTL after mission complete or when RC connection is lost without this script
cause it is only speed dependant.
-- User doesn't need to set sprayer enable on RCx_OPTION parameters 

### How to use
-- Activate scripting SCR_ENABLE = 1
-- Assign any RCx_OPTION parameter to Scripting1(300) to on/off sprayer


# Battery Tag Support

This script implements logging of DroneCAN BatteryTag messages. It is
used in combination with a BatteryTag AP_Periph node to log
information about number of cycles a battery has been through along
with the serial number and number of hours in an armed state.

The data for each battery is logged in the BTAG log message

# Parameters

## BTAG_ENABLE

Allow for enable/disable of the script

## BTAG_MAX_CYCLES

Maximum number of battery cycles to allow arming

## BTAG_CUR_CYCLES

Current maximum of number of cycles from all active BatteryTag
nodes. This can be used by other scripts to adjust battery percentage
estimation at boot based on age of the battery.

# Follow Target Send

Sends the FOLLOW_TARGET mavlink message to allow other vehicles to follow this one

# Parameters

- FOLT_ENABLE : Set to 1 to enable this script
- FOLT_MAV_CHAN : MAVLink channel to which FOLLOW_TARGET should be sent

# How To Use

1. copy this script to the autopilot's "scripts" directory
2. within the "scripts" directory create a "modules" directory
3. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory
4. the FOLLOW_TARGET message will be published at 10hz

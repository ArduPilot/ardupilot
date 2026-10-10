# System ID Watch

Periodically reports the MAVLink system IDs this vehicle can currently
see, and optionally collects received packet statistics for a single
system ID.

Systems are discovered from HEARTBEAT messages received on any MAVLink
channel, so only systems whose messages reach this vehicle (e.g. over a
shared radio link or via MAVLink routing) are reported.  HEARTBEATs
with a bad CRC are ignored.

## Parameters

- SYSW_ENABLE : set to 1 to enable this script.  Setting it to 0 and back to 1 clears the systems seen and the statistics
- SYSW_PERIOD : time in seconds between reports
- SYSW_TIMEOUT : a system or component is dropped from the report once no HEARTBEAT has been received from it for this many seconds
- SYSW_STATS_ID : MAVLink system ID to collect statistics for, 0 disables. Changing it resets the statistics

## Output

Every SYSW_PERIOD seconds a list of system IDs is sent, each followed by
the component IDs seen for that system:

    SYSW: 2 sys: 7[1,191] 255[190]

When SYSW_STATS_ID is non-zero, statistics for that system follow:

    SYSW 7: 12.8pkt/s total 795 chan 1
    SYSW 7: last 0.1s ago HB gap 1.0s badHB 0
    SYSW 7: comp 1:743 191:52
    SYSW 7: HEARTBEAT=2.0 SYS_STATUS=1.0 ATTITUDE=9.8

- pkt/s : packet rate over the last period, total : packets since statistics started, chan : MAVLink channels the system was received on
- last : time since a packet was last received from the system
- HB gap : longest interval between HEARTBEATs from component 1 in the last period, useful for spotting link dropouts
- badHB : HEARTBEATs from this system ID that failed their CRC check
- comp : packets received from each component since statistics started
- per-message rates in Hz over the last period

## Limitations

The scripting MAVLink interface only passes on messages registered by
the script, so statistics only count these common messages: HEARTBEAT,
SYS_STATUS, SYSTEM_TIME, PARAM_VALUE, GPS_RAW_INT, RAW_IMU,
SCALED_PRESSURE, ATTITUDE, GLOBAL_POSITION_INT, SERVO_OUTPUT_RAW,
MISSION_CURRENT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS, VFR_HUD,
COMMAND_INT, COMMAND_LONG, COMMAND_ACK, RADIO_STATUS, TIMESYNC,
BATTERY_STATUS, VIBRATION, HOME_POSITION, EXTENDED_SYS_STATE and
STATUSTEXT.  Only the HEARTBEAT CRC is checked.  Messages targeted at
other systems are not passed to scripts.

These messages are only registered the first time SYSW_STATS_ID is set
non-zero.  If the script's receive queue fills, messages are dropped and
"SYSW: rx queue full" is reported; counts will then be low.

Each script has its own MAVLink receive registrations and queue, so this
script can be run alongside other scripts that receive MAVLink.

"SYSW: unexpected msgid N" followed by "SYSW: no systems seen" while
other systems are present means the MAVLink modules in "scripts/modules"
are probably older than the firmware and are misreading the message
headers.

## How To Use

1. copy this script to the autopilot's "scripts" directory
2. within the "scripts" directory create a "modules" directory
3. copy the MAVLink/mavlink_msgs.lua and MAVLink/mavlink_msg_HEARTBEAT.lua files to the "scripts/modules/MAVLink" directory
4. set SCR_ENABLE = 1 and reboot
5. optionally set SYSW_STATS_ID to the system ID to gather statistics for

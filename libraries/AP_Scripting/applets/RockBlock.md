# RockBlock Lua Script

Lua script to send a recieve very basic MAVLink telemetry over a
Rockblock SBD satellite modem
Requires https://github.com/stephendade/rockblock2mav at the GCS end

Note that this uses MAVLink1 messaging, due to it's smaller message size compared
to MAVLink2.

Messages will only be send or received if High Latency Mode is enabled.

Setup:
This script requires 1 serial port:
- A "Script" port to connect the modem to

Caveats:
- This will *only* send HIGH_LATENCY2 packets via the SBD modem. No heartbeats, no command acknowledgements, no statustexts, no parameters, etc
- A single HIGH_LATENCY2 packet will be sent every RCK_PERIOD sec, in addition to receiving 
   a single MAVLink packet from the GCS. This is known as a mailbox check.
- Any incoming packets on the first mailbox check will be ignored (as these may be from a long time in the past)
- Only 1 command can be sent per mailbox check from the GCS. Any additional commands will overwrite the previous command

# Parameters

The script adds the following parameters:

## RCK_FORCEHL

Automatically enables High Latency mode if not already enabled

## RCK_PERIOD

When in High Latency mode, send Rockblock updates every RCK_PERIOD seconds

## RCK_DEBUG

Sends Rockblock debug text to GCS via statustexts

## RCK_ENABLE

Enables the modem transmission

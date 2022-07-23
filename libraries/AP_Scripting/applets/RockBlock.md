# RockBlock Lua Script

Lua script to send a recieve very basic MAVLink telemetry over a
Rockblock SBD satellite modem
Requires https://github.com/stephendade/rockblock2mav at the GCS end

Note that this uses MAVLink1 messaging, due to it's smaller message size compared
to MAVLink2.

Setup:
This script requires 1 serial port:
- A "Script" port to connect the modem to

Usage:
- Use the MAVLink High Latency Control ("link hl on|off" in MAVProxy) to control whether to send or not
  (or set the "force_hl_enable" variable to true)
- Use the "debug_text" variable to view debugging statustexts at the GCS

Caveats:
- This will *only* send HIGH_LATENCY2 packets via the SBD modem. No heartbeats, no command acknowledgements, no statustexts, no parameters, etc
- A single HIGH_LATENCY2 packet will be sent every 20 sec, in addition to recieving 
   a single MAVLink packet from the GCS. This is known as a mailbox check.
- Any incoming packets on the first mailbox check will be ignored (as these may be from a long time in the past)
- Only 1 command can be sent per mailbox check from the GCS. Any additional commands will overwrite the previous command

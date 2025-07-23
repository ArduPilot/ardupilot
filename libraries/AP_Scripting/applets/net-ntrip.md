# NTRIP Applet

Script for direct connection to an NTRIP caster (Ethernet enabled autopilots). Allows direct receipt of RTCM3 correction data for RTK GPS hardware without the use of a GCS.

## Requirements

Autopilot must be [Ethernet enabled](https://ardupilot.org/copter/docs/common-network.html) and connected to an Internet facing network when operating.

NTRIP caster must accept insecure http connections (not https).

## Configuration:

Configure the autopilot with [DHCP or a static IP, proper gateway, and netmask](https://ardupilot.org/copter/docs/common-network.html).


Open the script in a text editor and edit the  CASTER_IP, CASTER_PORT, MOUNTPOINT, USERNAME, and PASSWORD variables to match your NTRIP caster and credentials.

For example, RTK2Go is available at 3.143.243.81 port 2101. If the username is an email address, many casters require that you replace the `@` symbol with `-at-` (i.e., `user-at-email.com`). RTK2Go does not require a password, so the word `none` suffices for that field.

The caster IP address can be found by using ping on the command line, for example:
```
ping rtk2go.com
```

Message count (by RTCM3 message type) is available as named float values if `send_debug_msg_count = true` and can be viewed in Mission Planner's Quick tab.

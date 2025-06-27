# RockBlock 9704 Lua Script

A Lua script for ArduPilot that enables satellite communication
using the RockBlock 9704 SBD modem to send and receive basic MAVLink telemetry.

Requires https://github.com/stephendade/rockblock2mav (MQTT client) at the GCS end

Messages will be send or received if High Latency Mode is enabled, or as configured
by ``RK9_FORCEHL``.

Setup:
This script requires:
- A SERIALn port, set to Scripting. Connect to the RXD/TXD lines of the Rockblock's header
- A SCR_SDEVn (virtual) port, set to MAVLink High Latency. Requires SCR_SDEV_EN=1
- A relay output connected to the I_EN pin of the Rockblock's header
- A GPIO input connected to the I_BTD pin of the Rockblock's header

The Rockblock's V_IN- and V_IN+ header pins can be connected to the flight controller's GND and +5V pins respectively.

Caveats:
- This will *only* send HIGH_LATENCY2 packets via the SBD modem. No heartbeats, no command acknowledgements, no statustexts, no parameters, etc
- A single HIGH_LATENCY2 packet will be sent every RK9_PERIOD sec

# Parameters

The script has the following parameters:

## RK9_FORCEHL

Mode of operation:
- 0 = start disabled, can be enabled via MAV_CMD_CONTROL_HIGH_LATENCY (default)
- 1 = start enabled, can be disabled via MAV_CMD_CONTROL_HIGH_LATENCY
- 2 = enabled on loss of telemetry (GCS) link for RK9_TIMEOUT seconds

## RK9_PERIOD

When in High Latency mode, send Rockblock updates every RK9_PERIOD seconds. Defaults to 30 seconds.

## RK9_DEBUG

Sends Rockblock debug text to GCS via statustexts. Defaults to 0 (disabled).

## RK9_ENABLE

Enables the modem transmission. Defaults to 1 (enabled).

## RK9_TIMEOUT

If RK9_FORCEHL=2, this is the number of seconds of no-messages from the GCS until High Latency mode is auto-enabled.
Defaults to 5 seconds.

## RK9_SERPORT

Serial port number to which the Rockblock is connected.
This is the index of the SERIALn_ ports that are set to 28 for "scripting". Defaults to 0.

## RK9_SCRPORT

Scripting Serial port number to which the Rockblock is connected for HL2 messages.
This is the index of the SCR_SDEV ports that are set to 2 for "MavlinkHL". Defaults to 0.

## RK9_RELAY

RELAYn output to control Rockblock power. This connects to I_EN on the Rockblock header. Defaults to RELAY1.

## RK9_BOOTED

SERVOn GPIO pin number (usually 50 or greater) that reads the Rockblock booted state.
This connects to I_BTD on the Rockblock header. Requires SERVOn_FUNCTION=-1. Defaults to 52 (SERVO3).


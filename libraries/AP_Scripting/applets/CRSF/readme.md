# Bidirectional CRSF Telemetry Demo Programs


## CRSF-GCS-A.lua and CRSF-GCS-E.lua

A pair of scrips for Ardupilot (CRSF-GCS-A.lua) and EdgeTX (CRSF-GCS-E.lua) to create a simple Ground Control Station.

On EdgeTX use the rotary knob and ENT to select/change the parameters and send the changes to the flight controller. Whilst a parameter is updating it will show as "---", and will remain at "---" if the update/read failed. Clicking "---" will retry to retrieve the parameter value from the FC. Press RTN to cancel an edit. The title bar shows statistics on the number of packages sent/received.

The Ardupilot script shows the parameter reads and writes as status messages. CRSF telemetry is disabled on start of the script, but can be (re)enabled from EdgeTX.


## CRSF-Telem.lua

A script for Ardupilot to send CRSF Telemetry and/or Ardupilot Telemetry messages to the radio controller.


## CRSF-Rate-A.lua and CRSF-Rate-E.lua

A pair of scripts to test the throughput of the data link.


## CRSF-debug.lua

A script for the transmitter to show the received telemetry packets. Press enter to pause/unpause.


## Prerequisites

  - A working EdgeTX <-> ExpressLRS <-> Ardupilot setup

Tested with: Arducopter 4.4.0-DEV + ExpressLRS 3.1.2 100Hz-500Hz Full/Standard 1:2 + EdgeTX 2.8.0


## Ardupilot Installation

 - Set parameters:
   - SERIAL***x***_PROTOCOL = 23 (RCIN)
   - RC_OPTIONS = +256 (check "Use passthrough for CRSF telemetry")
   - SCR_ENABLE = 1 (enable Lua scripting)
 - Copy ***one (1)*** of the Ardupilot Lua Scripts to Ardupilot flight controller folder `/APM/scripts`
 - Reboot or restart FC
 - Monitor the messages in Mission Planner

Lua on Ardupilot needs 70K RAM, some hints to free up RAM:

- TERRAIN_ENABLE = 0  (default=1, frees 21K)
- LOG_FILE_BUFSIZE = 8 (default=16, frees 8K)
- EK3_IMU_MASK = 1 (default=3, frees 7K)
- SRTL_POINTS = 0 (default=300, frees 6K)
- BRD_OPTIONS = +2 (frees 12K, check "Disable mavFTP" on CubeBlack)


## EdgeTX Installation

 - Set ExpressLRS Telemetry Ratio to 1:2
 - Copy all the EdgeTX Lua scripts to transmitter folder `/SCRIPTS/TOOLS`
 - Press SYS button and start script `ExpressLRS-EdgeTX`


## Lua Ardupilot

`result: boolean = crsf.push(cmd: integer, data: string)`  
Returns true if data was placed in the transmit buffer, but this does not mean that the data was also transmitted over the air. Can be called without arguments to check the state of the transmit buffer
Data can contain 0x00. (It is a Lua string, not a C string)

`cmd: integer, data: string = crsf.pop()`
Returns nil if no data was received.


## Lua EdgeTX

`result: boolean = crossfireTelemetryPush(cmd: integer, data: table)`  
Returns true if data was placed in the transmit buffer, but this does not mean that the data was also transmitted over the air. Can be called without arguments to check the state of the transmit buffer

`cmd: integer, data: table = crossfireTelemetryPop()`  
Returns nil if no data was received.


## CSRF Technical Details

Crossfire (CSRF) is a closed source protocol, but has been extensively reverse engineered. The protocol runs over a serial connection, usually at 420K baud. Common is 2-wire non-inverted full duplex, but 1-wire inverted half duplex (S.PORT) is also used.

The CRSF packet data format is: `sync len type destination payload crc`

`sync` 1 byte synchronisation, the value is dependent on where the traffic flows:  
 - 0xEE: from radio transmitter to transmitter module
 - 0xEA: from transmitter module to radio transmitter
 - 0xC8: between flight controller and receiver module
`len` 1 byte length of type + dest + payload + crc, also total packet length - 2  
`type` 1 byte command, see: ardupilot/libraries/AP_RCProtocol/AP_RCProtocol_CRSF.h  
`destination` 1 byte destination address:  
 - 0x00: broadcast
 - 0xEA: radio transmitter
 - 0xC8: flight controller
`payload` up to 59 bytes (note: first byte is officially the `source` address, but is used here for payload data)  
`crc` 1 byte crc

CRSF packets with type < 0x28 do not have the `dest` byte, and are used for stick and telemetry data

Here an example of a serial data from a ExpressLRS receiver module to the flight controller:

```
sync len type dest -----payload--------------                                         crc
C8   0C  14        33 00 64 0D 00 04 01 00 00 00                                      96  --> type 14 = link statistics
C8   18  16        E0 03 1F 2B C0 F7 0B E2 B0 02 7C E0 63 1F FB D8 07 00 00 4C 3C DF  3C  --> type 16 = rc channel data
C8   23  80   EA   51 51 51 51 51 51 51                                               61  --> type 80 = Ardupilot telemetry, dest EA = Flight Controller
```

How the data is transmitted over the air depends on the specific CRSF module, a couple of examples:

TBS Crossfire has 150 Hz up- and downlink frame rate, with fallback to 50 Hz when the link is weak. Uplink frames contain rc channel data plus 9 bytes of telemetry data, downlink frames are smaller and contain 9 bytes of telemetry data. Theoretical telemetry throughput: 1350 bytes/sec uplink and 1350 bytes/sec downlink, whilst maintaining 150 Hz rc channel data rate. See https://www.g3gg0.de/wordpress/fpv/fpv-analysis-of-tbs-crossfire/

ExpressLRS frame rates are 50 to 500 Hz. Selecting 1:2 telemetry ratio will use half the frames for downlink. Each ELRS frame has 10 (full) or 5 (standard) bytes payload. A frame contains either: rc channel data(uplink), link statistics(up/down), or telemetry(up/down). The telemetry data is transmitted over the air as the actual CSRF frame, including 5 bytes CRSF header/crc bytes. The CRSF frame is spread over multiple ELRS frames as needed: up to 5 bytes of CRSF payload fit in one ELRS full frame, and 6-15 bytes fit in 2 ELRS full frames. Theoretical maximum throughput is 1665 bytes/sec up/down and is reached in 333 Hz Full mode. But this throughput will not be reached, as not all frames can be used for telemetry, some frames are used for rc channel and link statistics.

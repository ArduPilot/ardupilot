# LTE Modem Driver

This driver implements support for LTE modems for establishing
cellular data connections. It provides either PPP or a transparent TCP
connectivity to a remote server through the LTE modem, allowing
network communication over LTE networks without using a companion
computer.

The driver best paired with with the ArduPilot remote support server
https://support.ardupilot.org, but can also be used for any other
network service.

If you don't have access to the ArduPilot support server you can
install your own using https://github.com/ArduPilot/UDPProxy

# Supported Hardware

The driver currently supports:

 - all SimCom SIM7600 variants
 - SimCom A7670
 - Quectel EC200
 - Air780

There are some limitations:
 - the Air780 does not support signal level monitoring
 - the SimCom A7670 only supports TCP connections, not PPP

# Parameters

The script uses the following parameters:

## LTE_ENABLE

This must be set to 1 to enable the driver. Set to 0 to disable the
LTE modem driver.

## LTE_PROTOCOL

This controls if a PPP connection will be used or a raw TCP connection
with MAVLink2.

Set LTE_PROTOCOL to 48 for PPP and enable ArduPilot networking with
the NET_ENABLE=1 parameter.

Set LTE_PROTOCOL to 2 for MAVLink2 and enable the LTE_SERVER
parameters for the TCP server you want to connect to.

Note that the LTE_PROTOCOL parameter must match the value of the
SCR_SDEVn_PROTO parameter.

## LTE_SERPORT

This sets the serial port to use for the LTE modem. This is the index
of the SERIALn_ ports that are set to 28 for "scripting".

## LTE_SCRPORT

This sets the scripting serial port to use for the LTE modem. This is
the index of the SCR_SDEVn ports that are set to 2 for "MAVLink2". This
port handles the MAVLink data that will be transmitted over the LTE
connection. You must first set SCR_SDEV_EN to 1 to enable scripting
serial devices.

## LTE_SERVER_IP0

This is the first octet of the server IP address to connect to. The
full IP address is constructed from LTE_SERVER_IP0 through
LTE_SERVER_IP3. Range: 0-255. This is not used with PPP.

## LTE_SERVER_IP1

This is the second octet of the server IP address to connect to.
Range: 0-255. This is not used with PPP.

## LTE_SERVER_IP2

This is the third octet of the server IP address to connect to.
Range: 0-255. This is not used with PPP.

## LTE_SERVER_IP3

This is the fourth octet of the server IP address to connect to.
Range: 0-255. This is not used with PPP.

## LTE_SERVER_PORT

This sets the IPv4 port of the server to connect to. This should match
the port that your ground control station or server is listening on.
Range: 1-65525. This is not used with PPP.

## LTE_BAUD

This sets the baud rate for the serial port to the LTE modem to use
for data transfer. Common values are 115200 or 921600. Default:
115200.

## LTE_IBAUD

The initial baud rate when the modem is powered on. This is normally
115200 but can be changed in the modem using the AT+IREX terminal command.

## LTE_TIMEOUT

This sets the timeout in seconds for the LTE connection. If no data is
received for this time, the connection will be reset and the driver
will attempt to reconnect. Range: 1-60 seconds. Default: 10 seconds.

## LTE_OPTIONS

This sets options for debugging and data display

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - A SERIALn_PROTOCOL should be set to 28 (Scripting) where n matches LTE_SERPORT
 - SCR_SDEV should be set to 48 (PPP) or 2 (MAVLink2) for the port matching LTE_SCRPORT

If using PPP then you also need to ensure PPP support is compiled into
your firmware (you can use https://custom.ardupilot.org to do that)
and set NET_ENABLE=1. You will likely also want to setup some outgoing
UDP or TCP ports with the NET_Pn parameters.

Then the flight controller should be rebooted and parameters should be
refreshed.

Once loaded, the LTE_ parameters will appear and should be configured
according to the parameter list above. The server IP address and port
must be set to match your ground control station or telemetry server.

Here is a full set of required parameter settings assuming you want to
use PPP and make a TCP connection to the ArduPilot support server on
port 20001. It assumes you have the modem on Telem1 (SERIAL1)

 - SCR_ENABLE 1
 - SCR_SDEV_EN 1
 - SCR_SDEV1_PROTO 48
 - SERIAL1_PROTOCOL 28
 - LTE_PROTOCOL 48
 - NET_ENABLE 1
 - NET_P1_TYPE 3
 - NET_P1_IP0 157
 - NET_P1_IP1 245
 - NET_P1_IP2 83
 - NET_P1_IP3 174
 - NET_P1_PORT 20001

here is an alternative configuration where a direct TCP connection to
the support server is used and modem is attached on SERAL1 (no PPP used)

 - SCR_ENABLE 1
 - SCR_SDEV_EN 1
 - SCR_SDEV1_PROTO 2
 - SERIAL1_PROTOCOL 28
 - LTE_PROTOCOL 2
 - LTE_SERVER_IP0 157
 - LTE_SERVER_IP1 245
 - LTE_SERVER_IP2 83
 - LTE_SERVER_IP3 174
 - LTE_SERVER_PORT 20001

# Status Messages

The driver provides status messages through the GCS indicating the current
connection state:

- "LTE_modem: starting" - Driver initialization
- "LTE_modem: found modem" - Modem detected and responding
- "LTE_modem: CREG OK" - Network registration successful
- "LTE_modem: transparent mode set" - Modem configured for transparent operation
- "LTE_modem: network opened" - Network stack ready
- "LTE_modem: connected" - TCP connection established
- "LTE_modem: connection closed, reconnecting" - Connection lost, attempting reconnection
- "LTE_modem: timeout" - No data received within timeout period
- "LTE_modem: error response from modem" - Modem returned an error

# Physical Connections

The modem should be connected to a flight controller serial
port. You may also want to use a serial port with hardware flow
control support and set BRD_SERn_RTSCRS to 1 for that port.

Note that the modems can be quite sensitive to power supply
issues. The power from the serial port will likely not be sufficient.

# Troubleshooting

The driver creates a log file "LTE_modem.log" on the SD card that contains
all communication with the modem. This log can be useful for debugging
connection issues.

Common issues:
- Ensure the SIM card is properly inserted and activated
- Check that the cellular antenna is properly connected
- Verify network coverage at the operating location
- Confirm the server IP address and port are correct and reachable
- Check that the serial port configuration matches between the flight controller and modem

If the connection fails or is lost, the driver will automatically attempt
to reconnect by restarting the connection sequence.

# Logging

A LTE log message is saved in the onboard log. That has signal
strength information and data transfer statistics.

A NAMED_VALUE_FLOAT MAVLink message "LTE_RSSI" is sent with the RSSI
signal strength.

# Notes on specific modems

Each modem has it's own unique behaviour. Some of the key differences
are listed below.

## SIM7070G

The SIM7070G is a CAT-M modem, meaning you must have a special CAT-M
capable SIM. A normal data SIM won't work at all. I recommend the SIMs
from https://hologram.io

The baud rate of the SIM7070G as set by the AP+IPR AT
command is "sticky", it persists over a power cycle. This means if you
change the LTE_BAUD or LTE_IBAUD parameters then you can get stuck
where you can't connect as the script won't be trying the right
baudrate. You should connect on USB with a terminal program and use:

 - AT+IPR?
 - AT+IPR=115200

that will reset the baud rate. Note that the USB port on the SIM7070G
does not care what baud rate you connect on, so you can always use it
for recovery.

Some SIM7070G modems also have a problem with the level shifting of
the UART which can cause issues with reliable communcation. I have
found some modems only work at a max of 230400 baud.

You may also need to enable roaming using:

- AT+CREG=2
- AT+CGREG=2

may need NET_OPTIONS=64 as high load prevents LCP echo replies

Also note that all the SIM7070G modems I have (and all SimCom modems
I've tried) have reversed TX/RX labels on the UART. You need to
connect the flight controller TX to the SIM7070G TX and RX to RX.

## SIM7600

The SIM7600 is a CAT-1 modem, so it can use any data SIM. I recommend
using a multi-carrier SIM (eg. the SIMs from https://hologram.io ) to
maximimise the number of cell towers you can use.

Depending on your SIM and carrier you may need to enable roaming
using:

- AT+CREG=2
- AT+CGREG=2

Also note that all the SIM7600 modems I have (and all SimCom modems
I've tried) have reversed TX/RX labels on the UART. You need to
connect the flight controller TX to the SIM7070G TX and RX to RX.

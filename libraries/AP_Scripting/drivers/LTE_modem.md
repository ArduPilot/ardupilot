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

Currently the only modem that is supported is the SIM76xx series of
modems from SimCom.

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

This sets the baud rate for the serial port to the LTE modem. Common
values are 9600, 57600, or 115200. The modem must be configured to use
the same baud rate. Range: 9600-3686400. Default: 115200.

If using something other than 115200 you need to connect to the modem
with a terminal program and use AT+IPREX=BAUD to set the baud rate and
then save with AT&W. If the modem is wired to a flight controller then
you can use the SERIAL_PASS parameters to give temporary control of
the modem to a USB port so you can use a terminal protocol to
configure the modem.

## LTE_TIMEOUT

This sets the timeout in seconds for the LTE connection. If no data is
received for this time, the connection will be reset and the driver
will attempt to reconnect. Range: 1-60 seconds. Default: 10 seconds.

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
 - LTE_BAUD 115200
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
 - LTE_BAUD 115200
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

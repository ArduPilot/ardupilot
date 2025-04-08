# HobbyWing ESC DataLink Driver

https://www.hobbywing.com/en/products?id=59

This driver implements support the HobbyWing DataLink for HobbyWing
ESCs connected via a UART to an ArduPilot serial port. It supports up
to 8 ESCs.

# Parameters

The script used the following parameters:

## ESC_HW_ENABLE

this must be set to 1 to enable the driver

## ESC_HW_POLES

this should be set to the number of motor poles for eRPM to RPM
scaling. Please confirm the correct RPM using a tachometer

## ESC_HW_OFS

this parameter sets an offset for the first ESC number. It is useful
on vehicles where the first ESC is not the first SERVOn output, for
example on quadplanes. Set to zero for no offset.

# Hardware Setup

Connect the TX1 pin on the DataLink V2 to a RX pin on an ArduPilot
serial port and the GND pin on the DataLink V2 to the GND pin on the
ArduPilot UART.

Connect your ESCs into the 8 ESC connectors marked D1 to D8.

Power the DataLink V2 with a battery as indicated in the DataLink V2 manual.

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - ESC_HW_ENABLE should be set to 1
 - SERIALn_PROTOCOL should be set to 28 for the connected serial port

then the flight controller should rebooted and parameters should be
refreshed.

Once loaded the ESC_HW_ parameters will appear and should be
configured as per the above documentation.

Note that the DataLink does not provide any data unless the motor is
running, so you cannot see any valid data at all until you arm the
motors and they start spinning.


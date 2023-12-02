# InertialLabs INS Driver

This driver implements support for the InertialLabs INS
external AHRS navigation system

 https://inertiallabs.com/

# Parameters

The script used the following parameters:

## ILABS_ENABLE

this must be set to 1 to enable the driver

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - EAHRS_TYPE should be set to 10
 - EAHRS_SENSORS should be set as needed
 - SERIALn_PROTOCOL should be set to 28 for the connected EFI serial
 - GPS_TYPE should be set to 21 for GPS functionality

then the flight controller should rebooted and parameters should be
refreshed.

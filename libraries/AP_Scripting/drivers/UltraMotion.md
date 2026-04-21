# UltraMotion CAN Driver

This driver implements support for the UltraMotion CAN servos

# Parameters

The script used the following parameters:

## UM_SERVO_MASK

Mask of servo channels to transmit using UltraMotion CAN messages

## UM_CANDRV

This sets the CAN scripting driver number to attach to. This is
normally set to 1 to use a CAN driver with CAN_Dx_PROTOCOL=10. To use
the 2nd scripting CAN driver set this to 2 and set CAN_Dx_PROTOCOL=12.

## UM_RATE_HZ

This sets the update rate of the script in Hz (how often it checks for
new data from the ECU). A value of 200 is reasonable.

## UM_OPTIONS

This sets optional features. Set bit 1 for enabling CAN logging. Bit 2
enables telemetry parsing. Bit 3 for sending the position of all
servos as NAMED_VALUE_FLOAT MAVLink packets of name UMPOS_n where n is
the unit ID.

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - UM_SERVO_MASK needs to be set to a mask of servos

It is also strongly recommended that you raise SCR_THD_PRIORITY to 3
to ensure the script gets sufficient priority.

then the flight controller should rebooted and parameters should be
refreshed.

# Telemetry Support

To use telemetry you need to setup the servos with a specific txData
format. The code currently assumes txData is pABCDEFS


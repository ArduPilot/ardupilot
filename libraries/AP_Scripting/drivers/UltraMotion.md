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

This sets the update rate of the script in Hz (how often it sends
commands and checks for new data from the actuator). A value of 200 is
reasonable.

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

# UltraMotion Settings
The following settings need to be changed using the CLI or CONFIG.TXT
  - unitID should be set to the servo number (1-indexed)
  - pMin to 1000
  - pMax to 2000

If using telemetry (remember to set UM_OPTIONS bit 2), the following 
should also be set:
  - txID should be set to match unitID
  - txData should be set to ABCGHEFY
  - txIvl should be set according to your desired telemetry rate
    (e.g., 100 milliseconds for 10Hz)

Commands, assuming you are setting up servo 7, and a telemetry rate of
10Hz
```
id 7
pn 1000
px 2000
ni 7
dt ABCGHEFY
it 100
```

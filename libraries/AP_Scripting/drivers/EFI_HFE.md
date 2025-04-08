# EFI HFE Driver

This driver implements support for the HFE International range of EFI
CAN engine control units. It supports monitoring and control of HFE
engines on fixed wing aircraft. This driver assumes you are using the
ICE subsystem in fixed wing aircraft for engine control.

# Parameters

The script used the following parameters:

## EFI_HFE_ENABLE

this must be set to 1 to enable the driver

## EFI_HFE_CANDRV

This sets the CAN scripting driver number to attach to. This is
normally set to 1 to use a CAN driver with CAN_Dx_PROTOCOL=10. To use
the 2nd scripting CAN driver set this to 2 and set CAN_Dx_PROTOCOL=12.

## EFI_HFE_ECU_IDX

This sets the ECU number on the CAN bus. A value of zero means that
the ECU number is auto-detected based on the first ECU seen on the
bus.

## EFI_HFE_RATE_HZ

This sets the update rate of the driver. A value of 200 is reasonable

## EFI_HFE_FUEL_DTY

This sets the fuel density in grams per litre, for fuel consumption
calculations

## EFI_HFE_REL_IDX

This sets a relay number to use for the ECU enable function. if the
ECU requires a high voltage GPIO to enable then you should set a
RELAY_PIN that the ECU enable is attached to and set the relay number
here.

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - EFI_TYPE should be set to 7
 - ICE_ENABLE should be set to 1

then the flight controller should rebooted and parameters should be
refreshed.

Once loaded the EFI_HFE parameters will appear and should be set
according to the parameter list above.

The ICE start channel will be monitored for starter control.

The GCS will receive EFI_STATUS MAVLink messages which includes RPM,
cylinder head temperature, injection timing, engine load, fuel
consumption rate, throttle position atmospheric pressure and ECU
voltage.

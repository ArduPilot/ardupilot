# EFI JetCat Driver

This driver implements support for the JetCat range of engine
control units. It supports monitoring and control of JetCat engines.

# Parameters

The script used the following parameters:

## EFI_JC_ENABLE

this must be set to 1 to enable the driver

## EFI_JC_CANDRV

This sets the CAN scripting driver number to attach to. This is
normally set to 1 to use a CAN driver with CAN_Dx_PROTOCOL=10. To use
the 2nd scripting CAN driver set this to 2 and set CAN_Dx_PROTOCOL=12.

## EFI_JC_UPDATE_HZ

This sets the update rate of the script in Hz (how often it checks for
new data from the ECU). A value of 200 is reasonable.

## EFI_JC_THR_FN

This sets the SERVOn_FUNCTION number to monitor for throttle
command. For fixed wing forward throttle this should be set to 70. For
heli RSC control this should be set to 31. If set to zero then no
throttle control will be done by the driver.

## EFI_JC_THR_RATE

This is the throttle output rate in Hz. A value of zero will disable
throttle control. A typical rate would be 50Hz.

## EFI_JC_START_FN

This is the RC option to use to monitor start control. This should be
set to one of the scripting RC options (from 300 to 307). Then an
RCn_OPTION should be set to the same value. When this switch goes high
the engine start function will be sent to the ECU. When this switch
goes low a engine stop will be sent. A value of 0 disables the starter
control.

## EFI_JC_MIN_RPM

This is the minimum running RPM. When set to a positive value then the
driver will monitor engine RPM when the engine is started and if it
drops below this value then an engine start will be sent to restart
the engine.

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - EFI_TYPE should be set to 7

then the flight controller should rebooted and parameters should be
refreshed.

Once loaded the EFI_JC parameters will appear and should be set
according to the parameter list above.

A 2 position RC switch should be setup with RCn_OPTION=300 (or the
value of EFI_JC_START_FN) to enable starter control. When that switch
goes high the engine will be started. When it goes low the engine will
be stopped.

The GCS will receive EFI_STATUS MAVLink messages which includes RPM,
cylinder head temperature, exhaust gas temperature, injection timing,
engine load, fuel consumption rate, throttle position atmospheric
pressure and ignition voltage.

Setting EFI_JC_RPM_MIN allows for automatic in-flight engine
restart. If the engine RPM drops below this EFI_JC_RPM_MIN for 2
seconds while the engine should be started then an engine start
command will be sent to restart the engine.

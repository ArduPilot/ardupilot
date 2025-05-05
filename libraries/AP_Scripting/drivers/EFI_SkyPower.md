# EFI SkyPower Driver

This driver implements support for the SkyPower range of EFI engine
control units. It supports monitoring and control of SkyPower engines.

# Parameters

The script used the following parameters:

## EFI_SP_ENABLE

this must be set to 1 to enable the driver

## EFI_SP_CANDRV

This sets the CAN scripting driver number to attach to. This is
normally set to 1 to use a CAN driver with CAN_Dx_PROTOCOL=10. To use
the 2nd scripting CAN driver set this to 2 and set CAN_Dx_PROTOCOL=12.

## EFI_SP_UPDATE_HZ

This sets the update rate of the script in Hz (how often it checks for
new data from the ECU). A value of 200 is reasonable.

## EFI_SP_THR_FN

This sets the SERVOn_FUNCTION number to monitor for throttle
command. For fixed wing forward throttle this should be set to 70. For
heli RSC control this should be set to 31. If set to zero then no
throttle control will be done by the driver.

## EFI_SPI_THR_RATE

This is the throttle output rate in Hz. A value of zero will disable
throttle control. A typical rate would be 50Hz.

## EFI_SP_START_FN

This is the RC option to use to monitor start control. This should be
set to one of the scripting RC options (from 300 to 307). Then an
RCn_OPTION should be set to the same value. When this switch goes high
the engine start function will be sent to the ECU. When this switch
goes low a engine stop will be sent. A value of 0 disables the starter
control.

## EFI_SP_GEN_FN

This is the RC option (auxiliary function) to use for generator
control. This should be set to one of the scripting RC options (from
300 to 307) if generator control is needed. Then an RCn_OPTION should
be set to the same value. When this switch goes high the generator
start function will be sent to the ECU. When this switch goes low a
generator stop will be sent. A value of 0 disables the generator
control.

## EFI_SP_MIN_RPM

This is the minimum running RPM. When set to a positive value then the
driver will monitor engine RPM when the engine is started and if it
drops below this value then an engine start will be sent to restart
the engine.

## EFI_SP_TLM_RT

Telemetry rate. This is the rate at which extra telemetry values
are sent to the GCS.

## EFI_SP_LOG_RT

Log rate. This is the rate at which extra logging of the SkyPower EFI is
performed.

## EFI_SP_ST_DISARM

This controls if starting the engine while disarmed is allowed. 0:Disabled,1:Enabled.

## EFI_SP_MODEL

SkyPower EFI ECU model. 0:SRE_180, 1:SP_275.

## EFI_SP_GEN_CTRL

Enable generator control. 0:Disabled,1:Enabled

## EFI_SP_RST_TIME

SkyPower EFI restart time. If engine should be running and it has stopped for
this amount of time then auto-restart. To disable this feature set this value to zero.

## EFI_SP_THR_MAX

SkyPower EFI maximum throttle command. Use this parameter to limit the maximum demanded throttle, in case your engine power curve drops off past a throttle value. It is recommended that you limit your autopilot throttle limit instead.

## EFI_SP_GEN_AUTO

Enable automatic EFI Generator on/off logic. This will automatically switch
the generator on or off, depending on the engine load.

## EFI_SP_GEN_MIN

EFI Generator will switch ON if engine load is less than this parameter for
`EFI_SP_GEN_TIMER` seconds. Applies when `EFI_SP_GEN_AUTO`=1.

## EFI_SP_GEN_MAX

EFI Generator will switch OFF if engine load is more than this parameter for
`EFI_SP_GEN_TIMER` seconds. Applies when `EFI_SP_GEN_AUTO`=1.

## EFI_SP_GEN_TIMER

EFI Generator load has to be greater than `EFI_SP_GEN_MAX` or less than
`EFI_SP_GEN_MAX` for this many seconds for a switch to happen. Applies when
`EFI_SP_GEN_AUTO`=1.

## EFI_SP_GEN_TOUT

EFI Generator will not be switched on/off if it was previously switched within
this many seconds. Applies when `EFI_SP_GEN_AUTO`=1.

# Operation

This driver should be loaded by placing the lua script in the
APM/SCRIPTS directory on the microSD card, which can be done either
directly or via MAVFTP. The following key parameters should be set:

 - SCR_ENABLE should be set to 1
 - EFI_TYPE should be set to 7

then the flight controller should rebooted and parameters should be
refreshed.

Once loaded the EFI_SP parameters will appear and should be set
according to the parameter list above.

A 2 position RC switch should be setup with RCn_OPTION=300 (or the
value of EFI_SP_START_FN) to enable starter control. When that switch
goes high the engine will be started. When it goes low the engine will
be stopped.

The GCS will receive EFI_STATUS MAVLink messages which includes RPM,
cylinder head temperature, exhaust gas temperature, injection timing,
engine load, fuel consumption rate, throttle position atmospheric
pressure and ignition voltage.

Setting EFI_SP_RPM_MIN allows for automatic in-flight engine
restart. If the engine RPM drops below this EFI_SP_RPM_MIN for 2
seconds while the engine should be started then an engine start
command will be sent to restart the engine.

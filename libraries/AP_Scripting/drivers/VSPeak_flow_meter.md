# VSPeak Modell flow meter Driver

This driver implements support for the VSPeak Modell flow meter sensor.

https://www.vspeak-modell.de/en/flow-meter

# Setup

## Mode selection

There are two ways to operate this sensor.

**A: Save consumed volume**

Under this mode, the sensor will remember how much fuel volume was consumed
across power cycles. This allows you to have an accurate fuel tank level
reading even if you power down or reboot the autopilot.

However, you will have to manually reset the sensor each time you refuel the
vehicle. This is achieved by issuing a PWM signal to the sensor.

To set that mode, you need to set the `Auto Reset` setting to `OFF` in
the `FLOW.txt` sensor settings file.
For more information, read the sensor manual.

**B: Forget consumed volume**

Under this mode, the consumed fuel volume will reset after each power cycle.
This is especially important for fuel-powered vehicles, as there is no
equivalent of a voltage reading to initialize the remaining energy percentage.

On the upside, you do not need to wire a PWM signal to the sensor to reset the
consumed volume measurement. You will have to update the starting volume
manually.

To set that mode, you need to set the `Auto Reset` setting to `Power ON` in
the `FLOW.txt` sensor settings file.
For more information, read the sensor manual.

## Sensor configuration

Calibrate the flow meter according to the manufacturer instructions.

Additionally, configure the sensor to report the consumed fuel volume, by
setting the `FUEL display` to `consumed` in `FLOW.txt`.

## Script parameter settings

Once this is done, perform the following steps.

1. Place this script in the "scripts" directory of the autopilot.
2. Connect the sensor to a serial port (for now referred to as `SERIAL*`)
3. Enable the scripting engine via `SCR_ENABLE`.
4. Set the baud rate to 19200 with `SERIAL*_BAUD = 19`.
5. Set port protocol to scripting with `SERIAL*_PROTOCOL = 28`.
6. Point the script to that serial port by configuring `VSPF_PORT`. Set it to
0 to select the first **scripting** serial port, etc...

Then, decide which battery monitor will be assigned to the sensor (for now
referred to as `BATT*`).

7. Set `BATT*_MONITOR` = 29 (Scripting)

8. Set `BATT*_CAPACITY` to the amount of ml your tank is filled with. This can 
vary from flight to flight.
If operating under Mode A (Save consumed), you do not need to update this
value with what is remaining in the tank after every reboot.
If you are operating under Mode B (Reset consumed), you will have to update
this value after every reboot, if fuel was consumed in the meantime.

9. Tell the script which battery monitor is employed via `VSPF_BAT_IDX`.

10. Set the selected operation mode:
For mode A (save consumed), set `VSPF_MODE=0`.
For mode B (reset consumed), set `VSPF_MODE=1`.

11. Enable the script itself with `VSPF_ENABLE=1`.

# Operation

Once everything is configured correctly, the corresponding battery monitor
will display in the corresponding `BATTERY_STATUS` MAVLink message:
 - The current fuel flow in cl/hr (centiliters per hour) in the `current_battery` field.
 - The current fuel already consumed in ml (milliliters) in the `current_consumed` field.

You can use the parameter `VSPF_CFACT` to compensate for scaling errors in the
sensor setup. 1 is the default value. Set to <1 if the measurements are too high.

## Refueling

When refueling, make sure to set `BATT*CAPACITY` to the actual, new value of
the fuel volume that is in the tank.

If operating under mode A (Save consumed), reset the sensor by triggering the
PWM pulse.

# Parameters

The script used the following parameters:

## VSPF_ENABLE

Setting this to 1 enables the driver.

## VSPF_BAT_IDX

Selects which battery monitor instance will be fed the fuel consumption
information. 1-indexed.

## VSPF_CFACT

This is multiplicative factor to correct the measured flow. Set to <1 if your
sensor measures too high and vice versa.

## VSPF_MODE

Tells the script which mode it is operating in.
0: Mode A, save consumption.
1: Mode B, reset consumption.

## VSPF_PORT

Which Scripting serial port the sensor is connected at.
Set to 0 to select the first serial port, 1 to select the 2nd serial port, etc...
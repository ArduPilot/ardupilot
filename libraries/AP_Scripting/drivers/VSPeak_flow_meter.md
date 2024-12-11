# VSPeak Modell flow meter Driver

This driver implements support for the VSPeak Modell flow meter sensor.

https://www.vspeak-modell.de/en/flow-meter

# Parameters

The script used the following parameters:

## VSPF_ENABLE

Setting this to 1 enables the driver.

# Setup

First of all, calibrate and configure the flow meter according to the
manufacturer instructions. Set your configuration with the `FLOW.txt` file,
placed in the SD card in the sensor itself.
For this script, the consumed/level FUEL display setting is not relevant,
as only the current flow is used.

Once this is done, perform the following steps.

1. Place this script in the "scripts" directory of the autopilot.
2. Connect the sensor to a serial port (for now referred to as `SERIAL*`)
3. Enable the scripting engine via `SCR_ENABLE`.
4. Set the baud rate to 19200 with `SERIAL*_BAUD = 19`.
5. Set port protocol to scripting with `SERIAL*_PROTOCOL = 28`.

Then, decide which battery monitor will be assigned to the sensor (for now
referred to as `BATT*`).
6. Set `BATT*_MONITOR` = 27 (Scripting)
7. Set `BATT*_CAPACITY` to the amount of ml your tank is filled with. This can 
vary from flight to flight.
8. Tell the script which battery monitor is employed via `VSPF_BAT_IDX`.
9. Enable the script itself with `VSPF_ENABLE=1`.

# Operation

Once everything is configured correctly, the corresponding battery monitor
will display in the corresponding `BATTERY_STATUS` MAVLink message:
 - The current fuel flow in cl/hr (centiliters per hour) in the `current_battery` field.
 - The current fuel already consumed in ml (milliliters) in the `current_consumed` field.

You can use the parameter `VSPF_CFACT` to compensate for scaling errors in the
sensor setup. 1 is the default value. Set to <1 if the measurements are too high.

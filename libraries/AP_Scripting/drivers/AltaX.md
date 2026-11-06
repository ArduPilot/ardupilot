# Alta X

This driver implements support for the Alta X Gen1 CAN protocol for ESC Telemetry.

## Parameters

The script used the following parameters:

### ALTAX_MOT_RO

Set to 1 to enable listen only mode for connection with mixed PX4/ArduPilot.
Generally, users should use the default 0.

## Architecture

This driver sends the proprietary configuration messages on bootup.
It also round robin polls the motors for feedback.
Currently, voltage, current and RPM are parsed.
Other data is sent back but the contents are not yet known.

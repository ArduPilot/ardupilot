# Gain Tuner CRSF Menu

This script provides a simple menu on CRSF-compatible transmitters for in-flight tuning of Copter PID gains.

## Functionality

The script creates a root menu named "Gain Tuner" which contains two commands:
* **Increase Gains (+10%)**: Increases the P and I gains for Roll and Pitch by 10% of their current value.
* **Decrease Gains (-10%)**: Decreases the P and I gains for Roll and Pitch by 10% of their current value.

After each adjustment, a confirmation message is sent to the Ground Control Station (GCS) showing the new P gain values for Roll and Pitch.

**Important:** The changes made by this script are temporary and are **not** saved to EEPROM. The PID gains will revert to their saved values when the vehicle is rebooted.

### Tuned Parameters

* `ATC_RAT_RLL_P`
* `ATC_RAT_RLL_I`
* `ATC_RAT_PIT_P`
* `ATC_RAT_PIT_I`

## Setup

1.  Ensure your flight controller has scripting enabled by setting `SCR_ENABLE` to 1.
2.  Upload both `gain_tuner.lua` and the mandatory `crsf_helper.lua` library to the `APM/scripts/` directory on the flight controller's SD card.
3.  Reboot the flight controller.

## Usage

1.  On your CRSF-compatible transmitter, navigate to the Lua scripts or tools menu.
2.  Select the "Gain Tuner" menu.
3.  Select either "Increase Gains" or "Decrease Gains" and execute the command.
4.  Observe the confirmation message in your GCS to see the updated values.